#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import subprocess
import signal
import os
import sys
import time
import hashlib
import json
import numpy as np
from enum import Enum
import psutil

# Allow this module to run directly from the source tree during development.
if not __package__:
    source_package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if source_package_root not in sys.path:
        sys.path.insert(0, source_package_root)

from agent_graph import agent as agent_graph
print("agent_graph 导入成功，类型:", type(agent_graph))  # 添加调试
from langchain_core.messages import HumanMessage, AIMessage, ToolMessage
from autopatrol_robot.tools import ALL_TOOLS
from std_msgs.msg import Float32
from autopatrol_robot.navigation_service import (
    NavigationService,
    set_global_nav_service,
)

from autopatrol_robot.map_renderer import OccupancyMapRenderer

ros_available = threading.Lock()
ros_busy = False

# ============================================================
# 会话并发控制：防止同一 thread_id 的多个 invoke 竞态
# ============================================================
_chat_locks: dict = {}
_chat_locks_guard = threading.Lock()  # 保护 _chat_locks 字典本身


def _acquire_chat_lock(thread_id: str, blocking: bool = False) -> bool:
    """尝试获取指定 thread_id 的会话锁。

    Args:
        thread_id: 会话 ID（如 "web-session"）
        blocking: True=阻塞等待，False=立即返回

    Returns:
        True=获取成功，False=锁被占用
    """
    with _chat_locks_guard:
        if thread_id not in _chat_locks:
            _chat_locks[thread_id] = threading.Lock()
        lock = _chat_locks[thread_id]
    return lock.acquire(blocking=blocking)


def _release_chat_lock(thread_id: str):
    """释放指定 thread_id 的会话锁。"""
    with _chat_locks_guard:
        lock = _chat_locks.get(thread_id)
    if lock is not None:
        try:
            lock.release()
        except RuntimeError:
            pass  # 锁可能已释放（异常路径）


def _repair_orphaned_tool_calls(config: dict):
    """检查并修复 checkpoint 中的孤立 tool_calls。

    场景：前一次 invoke 在 agent_node 返回 AIMessage(tool_calls) 后、
    tools 节点执行前异常中断，导致 checkpoint 中存在孤立的工具调用，
    后续 invoke 时 DeepSeek API 会返回 400 错误。

    修复方式：手动遍历每条 tool_call，用 LangChain 的 tool.invoke() 执行，
    构造对应的 ToolMessage 并通过 update_state 写回 checkpoint。
    """
    try:
        state_snapshot = agent_graph.get_state(config)
        if state_snapshot is None or state_snapshot.values is None:
            return

        messages = list(state_snapshot.values.get("messages", []))
        if not messages:
            return

        last_msg = messages[-1]
        if not isinstance(last_msg, AIMessage) or not hasattr(last_msg, "tool_calls"):
            return
        if not last_msg.tool_calls:
            print(f"[StateRepair] 最后一条无 tool_calls，跳过")
            return

        print(f"[StateRepair] 发现孤立 tool_calls: {[tc.get('name') for tc in last_msg.tool_calls]}")
        print(f"[StateRepair] tool_calls 完整内容: {last_msg.tool_calls}")

        # 构建 tool_name → tool 映射
        tool_map = {t.name: t for t in ALL_TOOLS}

        new_messages = []
        for tc in last_msg.tool_calls:
            tool_name = tc.get("name", "")
            tool_args = tc.get("args", {})
            tool_id = tc.get("id", "")

            found = tool_map.get(tool_name)
            if found is None:
                new_messages.append(ToolMessage(
                    content=f"Tool '{tool_name}' not found",
                    tool_call_id=tool_id))
                continue

            try:
                # 用 LangChain 标准 invoke 调用 StructuredTool
                print(f"[StateRepair] 执行工具: {tool_name}({tool_args})")
                result = found.invoke(tool_args)
                print(f"[StateRepair] 工具返回: {str(result)[:100]}")
                new_messages.append(ToolMessage(
                    content=str(result),
                    tool_call_id=tool_id))
            except Exception as e:
                new_messages.append(ToolMessage(
                    content=f"Error executing {tool_name}: {e}",
                    tool_call_id=tool_id))

        if new_messages:
            agent_graph.update_state(config, {"messages": new_messages})
            print(f"[StateRepair] 已修复 {len(new_messages)} 条 ToolMessage")

            # 验证修复结果
            verify_state = agent_graph.get_state(config)
            if verify_state and verify_state.values:
                vmsgs = list(verify_state.values.get("messages", []))
                print(f"[StateRepair] 修复后 checkpoint 消息数: {len(vmsgs)}")
        else:
            print(f"[StateRepair] 没有生成 ToolMessage")
    except Exception as e:
        print(f"[StateRepair] 修复失败（非致命，将继续执行）: {e}")

# ==================== 状态枚举 ====================
class MapState(Enum):
    NOT_LOADED = "未加载"
    LOADING = "加载中"
    LOADED = "已加载"
    ERROR = "加载失败"

# ==================== 地图管理器 ====================
class MapManager:
    def __init__(self):
        self.current_process = None
        self.current_state = MapState.NOT_LOADED
        self.workspace_dir = os.environ.get('FISHBOT_WS', '/home/ke/chapt9_update/fishbot_ws')
        self.load_timer = None
        self.map_image_base64 = None
        self.map_render_info = None
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        
    def load_map(self, status_callback=None):
        """载入地图"""
        if self.current_state == MapState.LOADING:
            return False, "地图正在加载中"
        
        if self.current_state == MapState.LOADED:
            return False, "地图已加载"
        
        self.current_state = MapState.LOADING
        if status_callback:
            status_callback("loading", "正在加载地图...")
        
        # 构建命令
        command = f"""
        source /opt/ros/humble/setup.bash && \
        source {self.workspace_dir}/install/setup.bash && \
        ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=False
        """
        
        try:
            # 启动进程
            self.current_process = subprocess.Popen(
                ['bash', '-c', command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            
            # 启动定时器，5秒后默认认为加载成功
            def timer_callback():
                if self.current_state == MapState.LOADING:
                    self.current_state = MapState.LOADED
                    if status_callback:
                        status_callback("loaded", "地图加载成功")
            
            self.load_timer = threading.Timer(5.0, timer_callback)
            self.load_timer.start()
            
            # 监控进程是否意外退出
            def monitor_process():
                returncode = self.current_process.wait()
                if returncode != 0 and self.current_state == MapState.LOADING:
                    self.current_state = MapState.ERROR
                    if status_callback:
                        status_callback("error", f"地图加载失败 (退出码: {returncode})")
            
            threading.Thread(target=monitor_process, daemon=True).start()
            
            return True, "地图加载已启动"
            
        except Exception as e:
            self.current_state = MapState.ERROR
            if status_callback:
                status_callback("error", f"启动失败: {str(e)}")
            return False, f"启动失败: {str(e)}"
    
    def close_map(self, status_callback=None):
        """关闭地图"""
        if self.current_state != MapState.LOADED and self.current_state != MapState.LOADING:
            return False, "地图未加载"
        
        # 停止加载定时器
        if self.load_timer:
            self.load_timer.cancel()
            self.load_timer = None
        
        # 杀死地图进程
        if self.current_process and self.current_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGTERM)
                time.sleep(1)
                if self.current_process.poll() is None:
                    os.killpg(os.getpgid(self.current_process.pid), signal.SIGKILL)
            except:
                pass
            
            self.current_process = None
        
        self.current_state = MapState.NOT_LOADED
        self.map_image_base64 = None
        self.map_render_info = None
        
        if status_callback:
            status_callback("closed", "地图已关闭")
        
        return True, "地图已关闭"
    
    def set_map_data(self, image_base64, resolution, origin_x, origin_y, width, height, render_info=None):
        """设置地图数据"""
        self.map_image_base64 = image_base64
        self.map_render_info = render_info
        self.map_resolution = resolution
        self.map_origin_x = origin_x
        self.map_origin_y = origin_y
        self.map_width = width
        self.map_height = height
    
    def get_map_data(self):
        """获取地图数据"""
        if self.map_image_base64:
            return {
                'image': self.map_image_base64,
                'resolution': self.map_resolution,
                'origin_x': self.map_origin_x,
                'origin_y': self.map_origin_y,
                'width': self.map_width,
                'height': self.map_height,
                'render_info': self.map_render_info
            }
        return None
    
    def get_state(self):
        """获取地图状态"""
        return {
            'state': self.current_state.value,
            'is_loaded': self.current_state == MapState.LOADED,
            'has_map': self.map_image_base64 is not None,
            'render_info': self.map_render_info
        }

# ==================== 连接管理函数 ====================
# ==================== 连接管理函数 ====================
def kill_process_tree(pid):
    """杀死进程树 - 增强版"""
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        # 先杀子进程
        for child in children:
            try:
                child.kill()
            except:
                pass
        # 再杀父进程
        try:
            parent.kill()
        except:
            pass
        # 等待进程真正结束
        time.sleep(0.2)
    except psutil.NoSuchProcess:
        pass
    except Exception as e:
        print(f"杀死进程树失败: {e}")

def force_cleanup_all():
    """强制清理所有相关进程"""
    global robot_process, is_connected, map_manager, nav_service
    
    print("=== 开始强制清理 ===")
    
    # 1. 清理机器人连接进程（bringup.launch.py）
    if robot_process and robot_process.poll() is None:
        print(f"清理机器人进程: PID={robot_process.pid}")
        kill_process_tree(robot_process.pid)
        robot_process = None
    
    # 2. 清理地图进程（navigation2.launch.py）
    if map_manager.current_process and map_manager.current_process.poll() is None:
        print(f"清理地图进程: PID={map_manager.current_process.pid}")
        kill_process_tree(map_manager.current_process.pid)
        map_manager.current_process = None
    
    # 3. 清理 NavigationService（取消进行中的导航 + 销毁 PatrolNode + 停止后台 spin 线程）
    if nav_service:
        print("清理 NavigationService...")
        try:
            nav_service.shutdown()
        except Exception as e:
            print(f"NavigationService 清理异常: {e}")
    
    # 4. 清理特定 ROS2 进程（不要用 pkill -9 -f 'ros2'，会误杀系统进程）
    print("清理特定 ROS2 进程...")
    os.system("pkill -9 -f 'micro_ros_agent' 2>/dev/null")
    os.system("pkill -9 -f 'ydlidar' 2>/dev/null")
    os.system("pkill -9 -f 'robot_control_bridge' 2>/dev/null")
    os.system("pkill -9 -f 'fishbot_bringup' 2>/dev/null")
    os.system("pkill -9 -f 'navigation2.launch' 2>/dev/null")
    os.system("pkill -9 -f 'autopatrol.launch' 2>/dev/null")
    
    # 6. 清理端口占用
    print("清理端口占用...")
    os.system("fuser -k 8888/tcp 2>/dev/null")
    os.system("fuser -k 11311/tcp 2>/dev/null")
    
    # 7. 清理串口文件
    if os.path.exists("/tmp/tty_laser"):
        try:
            os.remove("/tmp/tty_laser")
            print("已删除串口锁文件")
        except:
            pass
    
    # 8. 重置全局状态
    is_connected = False
    map_manager.current_state = MapState.NOT_LOADED
    map_manager.map_image_base64 = None
    map_manager.map_render_info = None
    
    # 9. 等待进程完全退出
    time.sleep(1)
    
    print("=== 强制清理完成 ===")
# ==================== ROS2 节点类 ====================
class CarController(Node):
    def __init__(self, map_manager, socketio):
        super().__init__('web_car_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()
        self.last_command_time = time.time()
        self.map_manager = map_manager
        self.socketio = socketio
        self.map_renderer = OccupancyMapRenderer()
        self._map_fingerprint = None
        self._map_render_lock = threading.Lock()
        
        # /map 由 map_server 以 TRANSIENT_LOCAL 发布；匹配该 QoS 才能在网页重启后取得已加载地图。
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
        )
        self.battery_subscription = self.create_subscription(
            Float32,
            '/battery_percent',
            self.battery_callback,
            10
        )
        self.current_battery = 0.0
        # 订阅机器人位姿
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info("ROS2节点初始化完成，已订阅/map和/amcl_pose")
    def battery_callback(self, msg):
        """电池电量回调"""
        self.current_battery = msg.data
        self.socketio.emit('battery_status', {'percent': self.current_battery})
        self.get_logger().debug(f'电池电量: {self.current_battery}%')
    def map_callback(self, msg):
        """Render a cleaned display image without changing the navigation map."""
        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            raw_data = np.asarray(msg.data, dtype=np.int8)
            signature = f"{width}:{height}:{resolution:.9f}".encode("ascii")
            fingerprint = hashlib.blake2b(
                signature + raw_data.tobytes(), digest_size=16
            ).hexdigest()

            with self._map_render_lock:
                if (
                    fingerprint == self._map_fingerprint
                    and self.map_manager.map_image_base64 is not None
                ):
                    return

                self.get_logger().info(
                    f"渲染导入地图: {width}x{height}, 分辨率: {resolution}m/px"
                )
                image_base64, render_info = self.map_renderer.render(
                    raw_data, width, height, resolution
                )
                self._map_fingerprint = fingerprint

            self.map_manager.set_map_data(
                image_base64,
                resolution,
                origin_x,
                origin_y,
                width,
                height,
                render_info,
            )
            self.socketio.emit(
                "map_data",
                {
                    "image": image_base64,
                    "resolution": resolution,
                    "origin_x": origin_x,
                    "origin_y": origin_y,
                    "width": width,
                    "height": height,
                    "render_info": render_info,
                },
            )
            self.get_logger().info(
                "地图展示渲染完成: "
                f"去除孤岛 {render_info['removed_components']} 个, "
                f"补全像素 {render_info['filled_gap_pixels']}"
            )
        except Exception as exc:
            self.get_logger().error(f"地图展示渲染失败: {exc}")
            import traceback
            traceback.print_exc()
    def pose_callback(self, msg):
        """机器人位姿回调"""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            theta = np.arctan2(siny_cosp, cosy_cosp)
            
            self.socketio.emit('robot_pose', {
                'x': x,
                'y': y,
                'theta': theta
            })
            
        except Exception as e:
            self.get_logger().error(f"位姿处理失败: {str(e)}")
        
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """发布速度指令"""
        self.twist_msg.linear.x = linear_x
        self.twist_msg.angular.z = angular_z
        self.publisher.publish(self.twist_msg)
        self.last_command_time = time.time()
        
    def stop(self):
        """停止小车"""
        self.publish_velocity(0.0, 0.0)

# ==================== Flask 应用 ====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# ==================== 导航状态回调（导航完成时推送通知） ====================
# 注册时机：nav_manager 导入后（main 中动态注册，确保 socketio 可用）
_nav_callback_registered = False


def _setup_nav_callbacks():
    """注册导航阶段变更回调到 nav_manager，将阶段变更推送到 Web 前端。"""
    global _nav_callback_registered
    if _nav_callback_registered:
        return

    from autopatrol_robot.nav_manager import register_nav_callback
    from langchain_core.messages import HumanMessage, SystemMessage

    def _on_nav_phase(phase: str, message: str):
        """收到导航阶段变更 → 通过 SocketIO 推送给前端聊天窗口"""
        try:
            # 1. 立即推送原始状态消息
            socketio.emit('chat_reply', {
                'reply': message,
                'nav_phase': phase,
            })
            print(f"[NavPhase] {phase}: {message}")

            # 2. 重要阶段 → 后台生成 AI 主动播报
            _announce_via_ai(phase, message)
        except Exception as e:
            print(f"[NavPhase] 推送失败: {e}")

    def _announce_via_ai(phase: str, message: str):
        """后台线程：调用 LLM 生成拟人化播报，以 AI 助手身份主动告知用户。"""
        announce_phases = {'arrived', 'completed', 'failed', 'returning'}
        if phase not in announce_phases:
            return

        threading.Thread(
            target=_generate_and_emit_announcement,
            args=(phase, message),
            daemon=True,
        ).start()

    register_nav_callback(_on_nav_phase)
    _nav_callback_registered = True
    print("[NavCallback] 已注册导航阶段回调")


def _generate_and_emit_announcement(phase: str, message: str):
    """调用 LLM 生成拟人化主动播报并通过 SocketIO 推送。"""
    try:
        from autopatrol_robot.agent_graph import llm

        announcement_prompt = f"""你是一个 ROS2 机器人助手。请根据以下导航状态，用一句友好、口语化的话主动告知用户。

状态阶段：{phase}
系统消息：{message}

要求：
- 像朋友一样自然地说出来，不要机械地重复系统消息
- 为"arrived"（到达目标）时：可以问用户接下来做什么，或表达到达的喜悦
- 为"returning"（返回起点）时：告知用户正在回家路上
- 为"completed"（全部完成）时：告知任务完成，已回到起点
- 为"failed"（失败）时：用温和的语气告知出了问题
- 短小精悍，10-30字即可"""

        ai_msg = llm.invoke([
            SystemMessage(content="你是机器人的语音助手，说话简洁友好。"),
            HumanMessage(content=announcement_prompt),
        ])
        announcement = ai_msg.content.strip()

        # 推送到前端，作为 AI 助手消息
        socketio.emit('chat_reply', {
            'reply': announcement,
            'nav_phase': f'{phase}_ai',
        })
        print(f"[NavAnnounce] {phase}: {announcement}")

    except Exception as e:
        print(f"[NavAnnounce] 生成播报失败: {e}")
        # 失败时回退：推送原始消息作为 AI 消息
        try:
            socketio.emit('chat_reply', {
                'reply': message,
                'nav_phase': f'{phase}_ai',
            })
        except Exception:
            pass

car_controller = None
nav_service = None  # NavigationService 实例（在 main() 中创建）
map_manager = MapManager()
robot_process = None
is_connected = False
# ==================== 状态回调函数 ====================
def on_map_status(event_type, message):
    """地图状态回调"""
    print(f"[Map] {event_type}: {message}")

def on_nav_status(event_type, message):
    """导航状态回调"""
    print(f"[Navigation] {event_type}: {message}")

# ==================== WebSocket 事件 ====================
@socketio.on('chat_message')
def handle_chat_message(data):
    """处理聊天消息（SocketIO 方式）。

    导航工具已改为异步非阻塞：工具启动导航后立即返回，
    agent_graph.invoke 快速返回（秒级），前端立即看到 "正在前往..."。
    后续到达通知由 _setup_nav_callbacks 注册的回调推送。
    """
    user_message = data.get('message', '')
    thread_id = "web-session"

    print(f"\n{'='*60}")
    print(f"[ChatDebug] === SocketIO chat_message 进入 ===")
    print(f"[ChatDebug] 用户消息: {user_message!r}")
    print(f"[ChatDebug] thread_id: {thread_id}")

    # 获取会话锁，防止同一 session 的并发 invoke 导致状态损坏
    if not _acquire_chat_lock(thread_id, blocking=False):
        print(f"[ChatDebug] ❌ 锁获取失败，返回忙")
        emit('chat_reply', {'reply': '⏳ 上一个 AI 请求仍在处理中，请稍后再试...'})
        return

    print(f"[ChatDebug] ✅ 锁获取成功")

    try:
        config = {"configurable": {"thread_id": thread_id}}

        # --- 调试: 打印修复前的 state ---
        print(f"[ChatDebug] --- 修复前 checkpoint ---")
        pre_state = agent_graph.get_state(config)
        if pre_state and pre_state.values:
            pre_msgs = list(pre_state.values.get("messages", []))
            print(f"[ChatDebug] 历史消息数: {len(pre_msgs)}")
            for i, m in enumerate(pre_msgs):
                mtype = type(m).__name__
                has_tc = hasattr(m, 'tool_calls') and m.tool_calls
                tc_names = [tc.get('name') for tc in m.tool_calls] if has_tc else None
                content_preview = str(getattr(m, 'content', ''))[:80]
                print(f"[ChatDebug]   [{i}] {mtype} tc={tc_names} content={content_preview!r}")
        else:
            print(f"[ChatDebug] (无历史 checkpoint)")

        print(f"[ChatDebug] --- 调用 _repair_orphaned_tool_calls ---")
        _repair_orphaned_tool_calls(config)
        print(f"[ChatDebug] --- 修复后，开始 agent_graph.invoke ---")
        result = agent_graph.invoke(
            {"messages": [HumanMessage(content=user_message)]},
            config=config
        )
        print(f"[ChatDebug] agent_graph.invoke 返回，消息数: {len(result.get('messages', []))}")

        response = result["messages"][-1].content
        print(f"[ChatDebug] 最终回复: {response[:120]!r}")
        emit('chat_reply', {'reply': response})

    except Exception as e:
        import traceback
        print(f"[ChatDebug] ❌ 异常: {e}")
        traceback.print_exc()
        emit('chat_reply', {'reply': f'错误: {str(e)}'})
    finally:
        _release_chat_lock(thread_id)
        print(f"[ChatDebug] === SocketIO 结束 ===\n")
@socketio.on('connect')
def handle_connect():
    print('客户端已连接')
    map_data = map_manager.get_map_data()
    if map_data:
        emit('map_data', map_data)

@socketio.on('disconnect')
def handle_disconnect():
    print('客户端已断开')

# ==================== API 路由 ====================
@app.route('/chat', methods=['POST'])
def chat():
    """
    AI 聊天接口（REST 方式）。

    导航工具已改为异步非阻塞：invoke 快速返回，前端立即看到回复。
    到达通知由 nav_manager 回调通过 SocketIO 推送。
    """
    thread_id = "web-session"

    print(f"\n{'='*60}")
    print(f"[ChatDebug] === REST /chat 进入 ===")
    print(f"[ChatDebug] thread_id: {thread_id}")

    # 获取会话锁，防止同一 session 的并发 invoke 导致状态损坏
    if not _acquire_chat_lock(thread_id, blocking=False):
        print(f"[ChatDebug] ❌ 锁获取失败，返回 429")
        return jsonify({'status': 'busy', 'message': '上一个 AI 请求仍在处理中'}), 429

    print(f"[ChatDebug] ✅ 锁获取成功")

    try:
        data = request.get_json()
        user_message = data.get('message', '')
        print(f"[ChatDebug] 用户消息: {user_message!r}")

        config = {"configurable": {"thread_id": thread_id}}

        # --- 调试: 打印修复前的 state ---
        print(f"[ChatDebug] --- 修复前 checkpoint ---")
        pre_state = agent_graph.get_state(config)
        if pre_state and pre_state.values:
            pre_msgs = list(pre_state.values.get("messages", []))
            print(f"[ChatDebug] 历史消息数: {len(pre_msgs)}")
            for i, m in enumerate(pre_msgs):
                mtype = type(m).__name__
                has_tc = hasattr(m, 'tool_calls') and m.tool_calls
                tc_names = [tc.get('name') for tc in m.tool_calls] if has_tc else None
                content_preview = str(getattr(m, 'content', ''))[:80]
                print(f"[ChatDebug]   [{i}] {mtype} tc={tc_names} content={content_preview!r}")
        else:
            print(f"[ChatDebug] (无历史 checkpoint)")

        print(f"[ChatDebug] --- 调用 _repair_orphaned_tool_calls ---")
        _repair_orphaned_tool_calls(config)
        print(f"[ChatDebug] --- 修复后，开始 agent_graph.invoke ---")
        result = agent_graph.invoke(
            {"messages": [HumanMessage(content=user_message)]},
            config=config
        )
        print(f"[ChatDebug] agent_graph.invoke 返回，消息数: {len(result.get('messages', []))}")
        response = result["messages"][-1].content
        print(f"[ChatDebug] 最终回复: {response[:120]!r}")
        return jsonify({'status': 'success', 'reply': response})

    except Exception as e:
        import traceback
        print(f"[ChatDebug] ❌ 异常: {e}")
        traceback.print_exc()
        return jsonify({'status': 'error', 'message': str(e)}), 500
    finally:
        _release_chat_lock(thread_id)
        print(f"[ChatDebug] === REST /chat 结束 ===\n")
@app.route('/connect_robot', methods=['POST'])
def connect_robot():
    global robot_process, is_connected
    
    try:
        if is_connected:
            return jsonify({'status': 'success', 'message': '机器人已连接', 'connected': True})
        
        # 注意：不要调用 force_cleanup_all()，这会杀掉所有 ROS2 进程
        # 只做简单的清理
        os.system("pkill -9 -f 'micro_ros_agent' 2>/dev/null")
        os.system("pkill -9 -f 'ydlidar' 2>/dev/null")
        
        workspace_dir = os.environ.get('FISHBOT_WS', '/home/ke/chapt9_update/fishbot_ws')
        launch_path = os.path.join(workspace_dir, 'src', 'fishbot_bringup', 'launch', 'bringup.launch.py')
        
        full_command = f"""
        export ROS_DOMAIN_ID=0 && \
        source /opt/ros/humble/setup.bash && \
        source {workspace_dir}/install/setup.bash && \
        ros2 launch {launch_path}
        """
        
        print(f"执行命令: {full_command}")
        
        robot_process = subprocess.Popen(
            ['bash', '-c', full_command],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        
        time.sleep(3)
        
        if robot_process.poll() is None:
            is_connected = True
            print("机器人连接成功")
            return jsonify({'status': 'success', 'message': '连接成功', 'connected': True})
        else:
            print(f"连接失败，进程退出码: {robot_process.returncode}")
            return jsonify({'status': 'error', 'message': f'连接失败 (退出码: {robot_process.returncode})', 'connected': False}), 500
            
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({'status': 'error', 'message': str(e), 'connected': False}), 500
@app.route('/force_cleanup', methods=['POST'])    
def force_cleanup():
    """强制清理"""
    global is_connected
    
    try:
        force_cleanup_all()
        is_connected = False
        
        return jsonify({
            'status': 'success',
            'message': '清理完成',
            'connected': False,
            'map_loaded': False,
            'waypoints': []
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/connection_status', methods=['GET'])
def connection_status():
    """获取连接状态"""
    global is_connected
    return jsonify({
        'status': 'success',
        'connected': is_connected
    })
@app.route('/')
def index():
    """主页面"""
    return render_template('control.html')

@app.route('/control', methods=['POST'])
def control():
    """接收手动控制指令"""
    try:
        data = request.get_json()
        cmd = data.get('cmd', 'stop')
        
        if car_controller:
            if cmd == 'forward':
                car_controller.publish_velocity(0.5, 0.0)
            elif cmd == 'backward':
                car_controller.publish_velocity(-0.5, 0.0)
            elif cmd == 'left':
                car_controller.publish_velocity(0.0, 0.8)
            elif cmd == 'right':
                car_controller.publish_velocity(0.0, -0.8)
            elif cmd == 'stop':
                car_controller.publish_velocity(0.0, 0.0)
            elif cmd == 'custom':
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
                car_controller.publish_velocity(linear, angular)
            
            return jsonify({'status': 'success', 'cmd': cmd})
        else:
            return jsonify({'status': 'error', 'message': 'ROS2 node not initialized'}), 500
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/load_map', methods=['POST'])
def load_map():
    """载入地图"""
    try:
        success, message = map_manager.load_map(on_map_status)
        
        if success:
            return jsonify({
                'status': 'success',
                'message': message,
                'state': map_manager.get_state()
            })
        else:
            return jsonify({'status': 'error', 'message': message}), 400
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500
@app.route('/close_map', methods=['POST'])
def close_map():
    """关闭地图"""
    try:
        success, message = map_manager.close_map(on_map_status)
        
        if success:
            return jsonify({
                'status': 'success',
                'message': message,
                'state': map_manager.get_state()
            })
        else:
            return jsonify({'status': 'error', 'message': message}), 400
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500
@app.route('/map_status', methods=['GET'])
def map_status():
    """获取地图状态"""
    try:
        return jsonify({
            'status': 'success',
            'state': map_manager.get_state()
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/soft_reboot', methods=['POST'])
def soft_reboot():
    """软重启 - 调用ESP32的SetBool服务重启机器人"""
    try:
        full_command = f"""
        export ROS_DOMAIN_ID=0 && \
        source /opt/ros/humble/setup.bash && \
        source {map_manager.workspace_dir}/install/setup.bash && \
        ros2 service call /esp32_command std_srvs/srv/SetBool "{{data: true}}"
        """
        
        process = subprocess.Popen(
            ['bash', '-c', full_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )
        
        # 等待最多5秒获取结果
        try:
            stdout, stderr = process.communicate(timeout=5)
            if process.returncode == 0:
                return jsonify({'status': 'success', 'message': '软重启命令已发送'})
            else:
                return jsonify({'status': 'error', 'message': f'软重启失败: {stderr[:200]}'}), 500
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            return jsonify({'status': 'error', 'message': '软重启命令超时'}), 500
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

def run_flask():
    """运行 Flask 服务器（flask_socketio 内置多线程，无需额外配置）"""
    socketio.run(
        app, host='0.0.0.0', port=5000, debug=False,
        allow_unsafe_werkzeug=True
    )

def main():
    global car_controller, ros_busy, nav_service

    # 注册信号处理器，确保 SIGTERM/SIGINT 触发清理
    def _signal_handler(signum, frame):
        print(f"\n收到信号 {signum}，正在关闭服务...")
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rclpy.init()

    # 创建 NavigationService（统一导航入口，AI 和 Web 共享）
    # 注意：不在此处 wait_until_ready()——Nav2 由用户通过 Web UI 的
    # "加载地图"按钮启动，之后 NavigationService 首次 navigate_to() 时
    # 会懒加载 PatrolNode 并自动等待 Nav2 就绪。
    nav_service = NavigationService(status_callback=on_nav_status)
    set_global_nav_service(nav_service)

    # 注册导航阶段回调（导航完成/失败时推送到 Web 前端聊天窗口）
    _setup_nav_callbacks()

    car_controller = CarController(map_manager, socketio)

    # 启动 Flask 线程
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    print("=" * 50)
    print("小车网页控制服务已启动")
    print(f"请在浏览器中访问: http://<本机IP>:5000")
    print("按 Ctrl+C 停止服务")
    print("=" * 50)

    try:
        while rclpy.ok():
            try:
                # NavigationService 在它自己的后台线程中管理 PatrolNode，
                # 所以主循环只需 spin car_controller（手动控制节点）。
                # ros_busy 不再需要——NavigationService 内部有 threading.Lock。
                rclpy.spin_once(car_controller, timeout_sec=0.1)
            except IndexError as e:
                # 捕获 wait set index too big 错误
                print(f"[ROS2] 执行器错误: {e}，正在尝试恢复...")
                try:
                    car_controller.destroy_node()
                except:
                    pass
                car_controller = CarController(map_manager, socketio)
                print("[ROS2] 节点已重建")
            except Exception as e:
                print(f"[ROS2] 其他错误: {e}")
                time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        print("\n正在关闭服务...")
    finally:
        # ============ 完整清理路径 ============
        print("=== 开始清理所有资源 ===")

        # 1. 停止小车运动
        if car_controller:
            try:
                car_controller.stop()
            except Exception as e:
                print(f"停止小车异常: {e}")

        # 2. 强制清理所有子进程（bringup、navigation2 等）
        force_cleanup_all()

        # 3. 销毁 car_controller ROS2 节点
        if car_controller:
            try:
                car_controller.destroy_node()
            except Exception as e:
                print(f"销毁 car_controller 异常: {e}")

        # 4. shutdown rclpy（NavigationService 已在 force_cleanup_all 中清理）
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"rclpy.shutdown 异常: {e}")

        print("=== 所有资源清理完成 ===")
if __name__ == '__main__':
    main()
