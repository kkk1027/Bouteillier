#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import threading
import subprocess
import signal
import os
import re
import shlex
import time
import base64
import json
import numpy as np
from enum import Enum
from PIL import Image
import io
import psutil
from agent_graph import agent as agent_graph
print("agent_graph 导入成功，类型:", type(agent_graph))  # 添加调试
from langchain_core.messages import HumanMessage, AIMessage, ToolMessage
from autopatrol_robot.tools import ALL_TOOLS
from std_msgs.msg import Float32
from autopatrol_robot.navigation_service import (
    NavigationService,
    set_global_nav_service,
)

ros_available = threading.Lock()
ros_busy = False


def _detect_workspace_dir() -> str:
    """定位 fishbot_ws 工作空间根目录。

    原代码把默认值写死成 '/home/ke/chapt9_update/fishbot_ws'，那是原作者
    机器上的路径。换机器后 bringup.launch.py 找不到，ros2 launch 立即失败，
    前端就显示「连接失败 (退出码: 1)」。

    实际查找逻辑见 autopatrol_robot/paths.py（环境变量 → 文件位置推导 → 兜底）。
    """
    from autopatrol_robot.paths import fishbot_ws
    return fishbot_ws()


WORKSPACE_DIR = _detect_workspace_dir()

# ============================================================
# launch 输出落盘
# ------------------------------------------------------------
# 原来 bringup / navigation 的 stdout 和 stderr 都直接丢进 DEVNULL，
# 节点在 launch 里疯狂 respawn、报错刷屏，网页只显示「连接成功」，
# 日志一片空白，排查时完全无从下手。
#
# 真实踩坑（2026-10-30）：ydlidar_node 因为 launch 参数被
# voice_control.yaml 污染，0.5ms 内失败退出，respawn 重试 1127 次，
# /scan 永久消失，但网页端和终端都看不到任何线索。
# 现在改成写入 log/web/<name>.log，直接 tail 就能看到。
# ============================================================
_launch_logs: dict = {}


def _launch_log_stream(name: str, workspace_dir: str = None):
    """为 launch 进程打开日志文件，返回 (stream, path)。

    打开失败时回退到 DEVNULL，绝不因为写不了日志而影响启动。
    同名日志句柄会先关旧的再开新的，否则反复点「载入地图」会
    泄漏文件描述符（旧句柄没人关，也不在 _launch_logs 里了）。
    """
    base = workspace_dir or WORKSPACE_DIR
    log_dir = os.path.join(base, 'log', 'web')
    _close_launch_log(name)
    try:
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, '%s.log' % name)
        stream = open(path, 'a', buffering=1, encoding='utf-8',
                      errors='replace')
        stream.write('\n===== %s 启动 %s =====\n'
                     % (time.strftime('%Y-%m-%d %H:%M:%S'), name))
        stream.flush()
        _launch_logs[name] = stream
        print('[launch log] %s 输出写入 %s' % (name, path))
        return stream, path
    except Exception as exc:  # noqa: BLE001
        print('[launch log] 无法创建日志文件: %s' % exc)
        return subprocess.DEVNULL, None


def _close_launch_log(name: str):
    """关闭并移除日志文件句柄（清理时调用）。"""
    stream = _launch_logs.pop(name, None)
    if stream is not None and stream is not subprocess.DEVNULL:
        try:
            stream.close()
        except Exception:  # noqa: BLE001
            pass


# ============================================================
# AI 可用性前置校验
# ============================================================
def _api_key_missing_message() -> str:
    """AI 不可用时的可操作提示；可用时返回空字符串。

    注意：现在 AI 有两个来源（本地 Ollama / 远程 DeepSeek），
    所以这里要看 AI_UNAVAILABLE，而不是「有没有 DeepSeek Key」。
    本地模式下没有 Key 也完全能用。
    """
    try:
        from autopatrol_robot import agent_graph as _ag
        unavailable = getattr(_ag, 'AI_UNAVAILABLE', None)
        if unavailable is None:  # 兼容旧版本
            unavailable = getattr(_ag, 'DEEPSEEK_KEY_MISSING', False)
        provider = getattr(_ag, 'LLM_PROVIDER', '')
        model = getattr(_ag, 'LLM_MODEL', '')
        source = getattr(_ag, 'DEEPSEEK_KEY_SOURCE', '')
    except Exception:  # noqa: BLE001
        return ''
    if not unavailable:
        if provider:
            print('[AICheck] 模型来源：%s (%s)' % (provider, model))
        elif source:
            print('[AICheck] 使用 API Key 来源：%s' % source)
        return ''
    return (
        'AI 对话不可用：既没有本地模型，也没有 DeepSeek API Key。\n'
        '（网页控制、导航、地图功能均不受影响）\n\n'
        '方案 A（推荐，免费）：启动本地模型\n'
        '1) ~/.local/ollama/bin/ollama serve &\n'
        '2) ollama pull qwen2.5:3b\n'
        '3) 重启网页服务\n\n'
        '方案 B：配置 DeepSeek API Key（任选一）\n'
        '1) mkdir -p ~/.config/fishbot && '
        'echo "DEEPSEEK_API_KEY=sk-你的key" > ~/.config/fishbot/deepseek.env\n'
        '2) 在工作空间根目录建 .env 文件，内容同上\n'
        '3) 启动前 export DEEPSEEK_API_KEY=sk-你的key\n\n'
        '注意：直接 python3 web_controller.py 启动时不会读 .bashrc，'
        '必须用上面任一种方式。改完需重启 web 服务。'
    )


def _check_api_key_or_none():
    """Key 缺失时返回一个可直接 return 的 Flask 响应，否则返回 None。"""
    msg = _api_key_missing_message()
    if not msg:
        return None
    print('[AICheck] ❌ ' + msg.replace('\n', ' '))
    return jsonify({'status': 'error', 'message': msg}), 503

def _friendly_ai_error(exc) -> str:
    """把 LLM 异常翻译成用户能直接照做的提示。

    原始异常长这样（实际遇到过）：
      Error code: 401 - Authentication Fails, Your api key: ****o-ai is invalid
    其中 `o-ai` 是占位值 ``sk-placeholder-no-ai`` 的尾巴 —— 看到这个尾巴
    几乎可以断定「程序根本没拿到真 Key」，而不是「Key 真的过期了」。
    直接把这句英文 401 甃给用户，很容易让人去反复换 Key，白花时间。
    """
    text = str(exc)
    low = text.lower()

    is_auth = ('401' in text) or ('authentication' in low) \
        or ('invalid api key' in low) or ('api key' in low and 'invalid' in low)
    if is_auth:
        if 'placeholder' in low or 'o-ai' in low:
            return (
                '❌ AI 未配置真 Key：当前用的是占位值。\n'
                ' 程序没有从任何地方读到你的 DEEPSEEK_API_KEY。\n'
                ' 修复（三选一，推荐第 1 种）：\n'
                ' 1) mkdir -p ~/.config/fishbot && '
                'echo "DEEPSEEK_API_KEY=sk-你的key" > '
                '~/.config/fishbot/deepseek.env\n'
                ' 2) 在 fishbot_ws 根目录建 .env，内容同上\n'
                ' 3) 用 ./run_web_controller.sh 启动，'
                '并先 export DEEPSEEK_API_KEY=sk-…\n'
                ' 改完重启 web 服务（网页控制/导航不受影响）')
        return (
            '❌ AI 鉴权失败：Key 存在但被 DeepSeek 拒绝'
            '（可能已失效或余额不足）。\n'
            ' 检查：https://platform.deepseek.com/api_keys\n'
            ' 原始错误：%s' % text)

    if 'insufficient' in low or 'balance' in low or 'quota' in low:
        return '❌ AI 额度不足或余额用尽。请到 DeepSeek 控制台充值。\n原始错误：%s' % text

    if 'connect' in low or 'timeout' in low:
        return ('❌ 连不上 DeepSeek API（网络/代理问题）。\n'
                ' 注意：本程序对 LLM 客户端已关闭 trust_env，会绕过系统代理直连。\n'
                ' 原始错误：%s' % text)

    return '❌ AI 调用失败：%s' % text


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


# 导航固定使用的地图名（maps 目录下不带 .yaml 后缀）。
# 改地图只需改这一行。
DEFAULT_MAP_NAME = 'room1'

# ==================== 地图管理器 ====================
def _maps_dir(workspace_dir: str) -> str:
    return os.path.join(
        workspace_dir, 'src', 'fishbot_navigation2', 'maps')


def list_available_maps(workspace_dir: str = None):
    """列出可用地图（扫 maps 目录下的 *.yaml）。

    返回 [(name, abs_yaml_path, pgm_exists), ...]，按名字排序。
    没有对应 .pgm 的 yaml 无法被 map_server 加载，pgm_exists 标为 False。
    """
    base = workspace_dir or WORKSPACE_DIR
    mdir = _maps_dir(base)
    items = []
    try:
        for fn in os.listdir(mdir):
            if not fn.endswith('.yaml'):
                continue
            yaml_path = os.path.join(mdir, fn)
            name = os.path.splitext(fn)[0]
            pgm = os.path.join(mdir, name + '.pgm')
            items.append((name, yaml_path, os.path.isfile(pgm)))
    except OSError as exc:
        print('[Maps] 读取地图目录失败: %s' % exc)
    items.sort(key=lambda x: x[0])
    return items


def resolve_map_path(name_or_path: str, workspace_dir: str = None):
    """把用户选的地图名/路径解析成绝对 yaml 路径。

    允许只传名字（如 room1）也允许传绝对路径。
    返回 (abs_path, error_message)。
    """
    base = workspace_dir or WORKSPACE_DIR
    raw = (name_or_path or '').strip()
    if not raw:
        return '', '未指定地图'

    # 绝对路径：只校验存在性与后缀
    if os.path.isabs(raw):
        if not os.path.isfile(raw):
            return '', '地图文件不存在: %s' % raw
        return raw, ''

    mdir = _maps_dir(base)
    candidate = raw if raw.endswith('.yaml') else raw + '.yaml'
    # 防目录穿越：只允许纯文件名
    if os.path.basename(candidate) != candidate:
        return '', '非法地图名: %s' % raw
    path = os.path.join(mdir, candidate)
    if not os.path.isfile(path):
        return '', '地图不存在: %s（可用: %s）' % (
            candidate, ', '.join(n for n, _, _ in list_available_maps(base)))
    return path, ''


class MapManager:
    def __init__(self):
        self.current_process = None
        self.current_state = MapState.NOT_LOADED
        self.workspace_dir = WORKSPACE_DIR
        self.load_timer = None
        self.map_image_base64 = None
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        # 当前已载入的地图名，供前端展示与重载
        self.current_map_name = ''
        self.current_map_path = ''
        # 取消 /map 轮询线程（load_timer 现在是 Thread，没有 cancel()）
        self._load_cancel = threading.Event()

        # ------------------------------------------------------------
        # /map 就绪信号 —— 由 CarController.map_callback 回填
        # ------------------------------------------------------------
        # 为什么不用 `ros2 topic echo` 子进程判活（旧实现）：
        #   1) 输出是 "126\n---"，末尾带消息分隔符 `---`，
        #      旧代码 out.isdigit() 恒为 False → 永远报超时，
        #      哪怕地图早就加载好了（2026-10-31 实机抳到）。
        #   2) /map 是 TRANSIENT_LOCAL（latched），
        #      子进程默认 QoS 若落成 VOLATILE 就会永久挂起。
        # 节点自己就订阅了 /map，直接在回调里置标志最可靠。
        self._map_ready = threading.Event()
        self._map_frames = 0
        self.ros_node = None      # 创建节点后由 bind_ros_node 回填

    def bind_ros_node(self, node):
        """绑定 ROS 节点（仅用于日志/诊断，不影响就绪判定）。"""
        self.ros_node = node

    def mark_map_ready(self):
        """收到一帧 /map。由 CarController.map_callback 调用。"""
        self._map_frames += 1
        self._map_ready.set()

    def reset_map_ready(self):
        """开始新一轮加载前清空就绪信号，避免被上一张图的残留命中。"""
        self._map_ready.clear()
        self._map_frames = 0
        
    def load_map(self, status_callback=None, map_name: str = DEFAULT_MAP_NAME):
        """载入地图。

        默认固定加载 room1（见 DEFAULT_MAP_NAME）。

        为什么要显式传 map:= 而不是用 launch 默认值：
        navigation2.launch.py 里写死的默认图是 fishbot_map1.yaml，
        不传参数就会加载错误的图（表现为「点了载入却是张旧图」）。
        所以这里总是把绝对路径算好再传给 launch。
        """
        if self.current_state == MapState.LOADING:
            return False, "地图正在加载中"
        
        if self.current_state == MapState.LOADED:
            return False, "地图已加载（如需换图请先关闭地图）"

        map_path = ''
        if map_name:
            map_path, err = resolve_map_path(map_name, self.workspace_dir)
            if err:
                return False, err
            # yaml 里 image 是相对路径，必须保证同名 pgm 存在，
            # 否则 map_server 会加载失败而前端只看到一个含糊的超时。
            base = os.path.splitext(map_path)[0]
            if not os.path.isfile(base + '.pgm'):
                return False, '地图缺少同名 .pgm 文件: %s.pgm' % base

        if not map_path:
            # 走到这里说明 DEFAULT_MAP_NAME 为空，属于代码错误
            return False, '未指定地图名'
        
        self.current_state = MapState.LOADING
        self.current_map_name = map_name
        self.current_map_path = map_path
        if status_callback:
            status_callback("loading",
                            "正在加载地图 %s..." % (map_name or '默认'))
        
        # 构建命令
        # ★ 必须显式 export ROS_DOMAIN_ID=0，与 connect_robot 保持一致。
        #   否则若 web_controller 从一个带着别的域号的 shell 启动
        #   （如调试时 export ROS_DOMAIN_ID=77），导航会跑在 77 域，
        #   而 bringup 显式跑在 0 域，两边互相看不见：
        #   表现为「导航起来了但找不到机器人/地图」。
        map_arg = ''
        if map_path:
            map_arg = ' map:=%s' % shlex.quote(map_path)
        # ★ 显式声明 auto_localization:=true，不依赖 launch 默认值。
        #   这样「载入地图」按钮的行为是确定的：一定路由到带 AMCL
        #   自动全局定位的导航栈，不会因默认值被改动而静默退化。
        #   大小写用小写 true/false，与 launch 的 ParameterValue
        #   value_type=bool 解析保持一致。
        command = f"""
        export ROS_DOMAIN_ID=0 && \
        source /opt/ros/humble/setup.bash && \
        source {self.workspace_dir}/install/setup.bash && \
        ros2 launch fishbot_navigation2 navigation2.launch.py \
            use_sim_time:=false auto_localization:=true{map_arg}
        """
        
        try:
            # 启动进程
            log_stream, log_path = _launch_log_stream(
                'navigation2', self.workspace_dir)
            self.current_process = subprocess.Popen(
                ['bash', '-c', command],
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            
            # 校验地图是否真的加载成功。
            # 原实现是「5 秒后无条件当作成功」，于是 map_server 加载失败、
            # 甚至进程已经死了，前端依旧显示「已加载」，极具误导性。
            # 现在改为：起一个后台线程等 /map 真到货，拿到真实结果再定状态。
            self._load_cancel.clear()
            # ★ 先清就绪标志，再起导航。顺序反了就会把上一张图的残留
            #   当成新地图已加载。
            self.reset_map_ready()
            self.load_timer = threading.Thread(
                target=self._wait_map_ready, daemon=True,
                args=(status_callback,))
            self.load_timer.start()
            
            # 监控进程是否意外退出
            def monitor_process():
                returncode = self.current_process.wait()
                if self.current_state == MapState.LOADING:
                    self.current_state = MapState.ERROR
                    if status_callback:
                        status_callback(
                            "error",
                            f"导航进程退出 (退出码: {returncode})，"
                            f"详情见 {log_path or '终端'}")
            
            threading.Thread(target=monitor_process, daemon=True).start()
            
            msg = "地图加载已启动"
            if map_path:
                msg += "（%s）" % os.path.basename(map_path)
            if log_path:
                msg += f" 日志: {log_path}"
            return True, msg
            
        except Exception as e:
            self.current_state = MapState.ERROR
            if status_callback:
                status_callback("error", f"启动失败: {str(e)}")
            return False, f"启动失败: {str(e)}"

    def _wait_map_ready(self, status_callback=None, timeout: float = 60.0):
        """等 /map 真的到货后把状态置为 LOADED。

        ★ 2026-10-31 重写。旧实现用子进程
          `ros2 topic echo /map --once --field info.width` 判活，
          有两个致命问题（实机日志实锤）：

          问题 1：判定条件永远是 False
            ros2 topic echo 的输出是 "126\\n---"，
            末尾带着消息分隔符 `---`。
            旧代码 out.strip().isdigit() → False，
            **不管地图加载得多好，60 秒后必定报超时**。
            前端因此永远显示「加载失败」，但 rviz 里地图好好的。

          问题 2：QoS 可能落成 VOLATILE
            /map 的发布者是 TRANSIENT_LOCAL（latched），
            只发一次。VOLATILE 订阅能连上但收不到历史消息，
            于是 `ros2 topic echo` 永久挂起、白等到超时。

        现在直接用节点自己的 /map 订阅（QoS 已修为 TRANSIENT_LOCAL），
        回调里置 Event，这里等 Event。不 spawn 任何子进程 ——
        既快又准，也不用依赖 ros2 daemon 的发现缓存。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._load_cancel.is_set():
                print('[Map] 轮询被取消（用户关闭了地图）')
                return
            if self.current_state != MapState.LOADING:
                return          # 进程已退出，交给 monitor_process 报错

            # 200ms 醒一次：既能快速响应取消，也不空转吃 CPU
            if self._map_ready.wait(timeout=0.2):
                self.current_state = MapState.LOADED
                if status_callback:
                    status_callback(
                        "loaded",
                        "地图已加载（收到 %d 帧 /map）" % self._map_frames)
                print('[Map] /map 就绪，地图加载成功（共 %d 帧）'
                      % self._map_frames)
                return

            # 等不到地图时给一次有用的诊断：是没人发，还是 QoS 不匹配？
            # 每 10 秒提示一次，避免刷屏。
            elapsed = timeout - (deadline - time.time())
            if elapsed >= 10.0 and int(elapsed) % 10 < 1:
                node = self.ros_node
                if node is not None:
                    try:
                        pubs = node.count_publishers('/map')
                    except Exception:              # noqa: BLE001
                        pubs = -1
                    print('[Map] 已等 %.0fs，当前 /map 发布者 %d 个'
                          '（0 个说明 map_server 未激活）'
                          % (elapsed, pubs))

        if self.current_state == MapState.LOADING:
            self.current_state = MapState.ERROR
            if status_callback:
                status_callback(
                    "error",
                    "%.0f 秒内未收到 /map，地图加载失败（检查地图文件、"
                    "map_server 是否 ACTIVE、TF 是否连通）" % timeout)
            print('[Map] /map 超时，判定加载失败')
    
    def close_map(self, status_callback=None):
        """关闭地图"""
        if self.current_state != MapState.LOADED and self.current_state != MapState.LOADING:
            return False, "地图未加载"
        
        # 先通知 /map 轮询线程退出
        self._load_cancel.set()
        if self.load_timer and self.load_timer.is_alive():
            self.load_timer.join(timeout=3.0)
        self.load_timer = None
        
        # 杀死地图进程（整组，含 nav2 容器内的子进程）
        if self.current_process and self.current_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGTERM)
                time.sleep(1)
                if self.current_process.poll() is None:
                    os.killpg(os.getpgid(self.current_process.pid), signal.SIGKILL)
            except:
                pass
            
            self.current_process = None
        
        _close_launch_log('navigation2')
        self.current_state = MapState.NOT_LOADED
        self.map_image_base64 = None
        self.current_map_name = ''
        self.current_map_path = ''
        
        if status_callback:
            status_callback("closed", "地图已关闭")
        
        return True, "地图已关闭"
    
    def reset_after_nav_stop(self):
        """导航栈被外部停掉后复位状态。

        切到建图模式时导航栈会先被停掉，但 map_manager 内部仍记着
        「LOADED」，前端就会继续显示「已加载」并允许点「关闭地图」，
        点了之后会因为进程已不存在而报错。所以这里主动复位。
        """
        self._load_cancel.set()
        self.load_timer = None
        self.current_process = None
        self.current_state = MapState.NOT_LOADED
        self.map_image_base64 = None
        self.current_map_name = ''
        self.current_map_path = ''
        _close_launch_log('navigation2')

    def set_map_data(self, image_base64, resolution, origin_x, origin_y, width, height):
        """设置地图数据"""
        self.map_image_base64 = image_base64
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
                'height': self.map_height
            }
        return None
    
    def get_state(self):
        """获取地图状态"""
        return {
            'state': self.current_state.value,
            'is_loaded': self.current_state == MapState.LOADED,
            'has_map': self.map_image_base64 is not None
        }


# ==================== 建图管理 ====================
class MappingState(Enum):
    IDLE = "未建图"
    STARTING = "启动中"
    MAPPING = "建图中"
    SAVING = "保存中"
    ERROR = "出错"


# 导航栈入口（只停这一组，绝不动 micro-ROS agent）
_NAV2_LAUNCH_MARK = 'ros2 launch fishbot_navigation2 navigation2.launch.py'
# 建图栈入口
_MAPPING_LAUNCH_MARK = 'ros2 launch fishbot_bringup manual_mapping.launch.py'

# 停导航栈时**绝对不能碰**的进程特征。
# 这些属于 bringup：micro-ROS agent 一旦被杀，ESP32 只在开机时连一次，
# 必须人工断电重启才能恢复 /odom 和 /imu —— 用户明确要求保留。
_MAPPING_PROTECTED = (
    'micro_ros_agent',
    'micro-ros-agent',
    'ros_serial2wifi/lib/ros_serial2wifi/tcp_server',
    'ydlidar/lib/ydlidar/ydlidar_node',
    'fishbot_bringup/lib/fishbot_bringup/odom2tf',
    'fishbot_bringup/lib/fishbot_bringup/imu_bias_remover',
    'fishbot_bringup/lib/fishbot_bringup/odom_covariance_injector',
    # EKF 是 bringup 提供的融合里程计，--node 名是 ekf_filter_node，
    # 可执行文件在系统目录下；不保护的话一旦有人把它加进补刀名单
    # 就会直接断掉 odom→base_footprint 的唯一来源。
    'robot_localization/ekf_node',
    'ekf_filter_node',
    'robot_state_publisher',
    'joint_state_publisher',
)

# nav2 专有节点：killpg 之后若还有孤儿残留，按这些特征补刀
_NAV2_ONLY = (
    'component_container_isolated',
    'nav2_container',
    'auto_localizer.py',
    'nav2_amcl/amcl',
    'nav2_map_server/map_server',
    'nav2_planner/planner_server',
    'nav2_controller/controller_server',
    'nav2_bt_navigator/bt_navigator',
    'nav2_behaviors/behavior_server',
    'nav2_waypoint_follower/waypoint_follower',
    'nav2_velocity_smoother/velocity_smoother',
    'nav2_smoother/smoother_server',
    'nav2_util/lifecycle_manager',
    'nav2_lifecycle_manager',
)

# 建图专有节点
_CARTO_ONLY = (
    'cartographer_ros/cartographer_node',
    'cartographer_occupancy_grid_node',
    'manual_mapping_manager.py',
)


def _is_protected(cmdline: str) -> bool:
    return any(mark in cmdline for mark in _MAPPING_PROTECTED)


def _pids_matching(marks, require_ws=True, protect=True):
    """按特征找进程 PID。

    跳过：自己、自己的祖先、受保护进程，以及（默认）不属于本工作
    空间的进程。归属判定复用 _belongs_to_ws，避免误杀别的 ROS 项目。

    protect=False 用于「探测 bringup 是否在跑」这类只读场景 —— 否则
    受保护的 micro_ros_agent 会被跳过，探测结果永远是「没在跑」。
    """
    me = os.getpid()
    ancestors = set()
    pid = me
    while pid and pid != 1:
        ancestors.add(pid)
        try:
            with open('/proc/%d/stat' % pid, 'r') as fp:
                fields = fp.read().rsplit(')', 1)[1].split()
            pid = int(fields[1])
        except (OSError, IndexError, ValueError):
            break

    found = []
    try:
        entries = os.listdir('/proc')
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        p = int(entry)
        if p in ancestors:
            continue
        cmdline = _read_cmdline(p)
        if not cmdline:
            continue
        if not any(mark in cmdline for mark in marks):
            continue
        if protect and _is_protected(cmdline):
            continue
        if require_ws and not _belongs_to_ws(p):
            continue
        found.append(p)
    return found


def _bringup_running():
    """bringup（雷达桥 / 雷达驱动 / micro-ROS agent）是否在跑。

    建图栈复用 bringup：如果它已经在跑，就不要用 include_bringup:=true
    再起一份。否则取消建图时会把 micro-ROS agent 一起杀掉，而 ESP32
    只在开机时连一次 agent，必须人工断电才能恢复。
    """
    marks = (
        'ros_serial2wifi/lib/ros_serial2wifi/tcp_server',
        'ydlidar/lib/ydlidar/ydlidar_node',
        'micro_ros_agent',
        'micro-ros-agent',
    )
    return bool(_pids_matching(marks, require_ws=False, protect=False))


def _stop_stack(launch_mark, only_marks, label, label_cn=None, term_wait=6.0):
    """停掉一个 launch 栈：先整组 SIGTERM，再按特征补刀。

    label 用于关闭对应日志流；label_cn 只用于给人看的提示文字。
    返回 (ok, 说明文字)。
    """
    name_cn = label_cn or label
    launch_pids = _pids_matching((launch_mark,))
    killed_groups = []
    for pid in launch_pids:
        try:
            pgid = os.getpgid(pid)
        except OSError:
            continue
        if pgid == os.getpgid(0):
            continue        # 别杀自己的进程组
        try:
            os.killpg(pgid, signal.SIGTERM)
            killed_groups.append(pid)
        except OSError:
            pass

    deadline = time.time() + term_wait
    while time.time() < deadline:
        if not _pids_matching((launch_mark,) + tuple(only_marks)):
            break
        time.sleep(0.3)

    leftovers = _pids_matching((launch_mark,) + tuple(only_marks))
    if leftovers:
        for pid in leftovers:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(0.5)
        leftovers = _pids_matching((launch_mark,) + tuple(only_marks))

    _close_launch_log(label)
    if leftovers:
        return False, '%s 仍有 %d 个进程未退出: %s' % (
            name_cn, len(leftovers), leftovers)
    return True, '已停止 %s（%d 个进程组）' % (name_cn, len(killed_groups))


class MappingManager:
    """管理手动建图栈：开始 / 取消 / 保存。

    设计要点：
      * 建图与导航互斥 —— Cartographer 和 AMCL 都会发布 map→odom TF，
        同时跑会互相覆盖，定位必然错乱。所以启动建图前先停导航栈。
      * 停导航栈时保留 bringup（micro-ROS agent / 雷达 / EKF），
        否则 ESP32 需要人工断电重启。
    """

    def __init__(self, workspace_dir=None):
        self.workspace_dir = workspace_dir or WORKSPACE_DIR
        self.state = MappingState.IDLE
        self.current_process = None
        self.launch_time = None
        self.last_message = ''
        self.last_saved = None
        self._lock = threading.Lock()

    # ---------- 状态 ----------
    def get_state(self):
        alive = (self.current_process is not None
                 and self.current_process.poll() is None)
        if self.state in (MappingState.MAPPING, MappingState.STARTING) \
                and not alive:
            self.state = MappingState.IDLE
            self.last_message = '建图进程已退出'
        elapsed = 0.0
        if alive and self.launch_time:
            elapsed = time.time() - self.launch_time
        return {
            'state': self.state.value,
            'is_mapping': alive and self.state in (
                MappingState.MAPPING, MappingState.STARTING),
            'elapsed_sec': round(elapsed, 1),
            'message': self.last_message,
            'last_saved': self.last_saved,
        }

    # ---------- 开始 ----------
    def start(self):
        with self._lock:
            if self.current_process is not None \
                    and self.current_process.poll() is None:
                return False, '建图已在运行中（已 %.0fs）' % (
                    time.time() - (self.launch_time or time.time()))

            self.state = MappingState.STARTING
            steps = []

            # 0) 建图必须有 bringup 提供的 /scan、/odom、/imu 与 TF。
            #    先在导航栈还活着的时候探测，避免停完才发现没法建图，
            #    留下一个「导航停了、建图也起不来」的空窗状态。
            if not _bringup_running():
                self.state = MappingState.IDLE
                msg = ('机器人底层未启动（找不到 micro-ROS agent / 雷达桥）。'
                       '请先点「连接机器人」，等雷达和 /odom 正常后再开始建图。')
                self.last_message = msg
                return False, msg

            # 1) 先停导航栈（含 AMCL），避免两套 map→odom 打架
            nav_ok, nav_msg = _stop_stack(
                _NAV2_LAUNCH_MARK, _NAV2_ONLY, 'navigation2', label_cn='导航栈')
            steps.append(nav_msg)
            if not nav_ok:
                self.state = MappingState.ERROR
                self.last_message = nav_msg
                return False, nav_msg

            # 导航停了，地图状态也要复位，否则前端还显示「已加载」
            try:
                map_manager.reset_after_nav_stop()
            except Exception as exc:            # noqa: BLE001
                print('[Mapping] 复位地图状态失败: %s' % exc)

            # 2) 起建图栈。
            #    include_bringup 跟随实际状态：
            #      - bringup 已在跑（连接机器人时启的）→ false，只起
            #        Cartographer。这样取消建图时只杀 Cartographer，
            #        micro-ROS agent / 雷达 / EKF 继续存活。
            #      - 没有 bringup → true，完整启动。
            #    这一条很关键：如果重复启动 bringup，取消建图时
            #    killpg 会把 agent 一起带走 → ESP32 必须断电重启。
            bringup_alive = _bringup_running()
            include_bringup = 'false' if bringup_alive else 'true'
            command = f"""
            export ROS_DOMAIN_ID=0 && \
            source /opt/ros/humble/setup.bash && \
            source {self.workspace_dir}/install/setup.bash && \
            ros2 launch fishbot_bringup manual_mapping.launch.py \
                use_sim_time:=false include_bringup:={include_bringup}
            """
            try:
                log_stream, log_path = _launch_log_stream(
                    'manual_mapping', self.workspace_dir)
                self.current_process = subprocess.Popen(
                    ['bash', '-c', command],
                    stdout=log_stream,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
            except Exception as exc:             # noqa: BLE001
                self.state = MappingState.ERROR
                self.last_message = '启动建图失败: %s' % exc
                return False, self.last_message

            self.launch_time = time.time()
            self.state = MappingState.MAPPING
            if bringup_alive:
                self.last_message = '建图已启动（复用已运行的 bringup）；请缓慢遥控覆盖全屋'
            else:
                self.last_message = '建图已启动（含 bringup）；请缓慢遥控覆盖全屋'

            # 进程意外退出时回写状态
            proc = self.current_process

            def _watch():
                rc = proc.wait()
                if self.current_process is proc:
                    self.state = MappingState.IDLE
                    self.last_message = '建图进程退出 (rc=%s)' % rc
                    print('[Mapping] 建图进程退出 rc=%s' % rc)

            threading.Thread(target=_watch, daemon=True).start()

            msg = '建图已启动。' + '；'.join(steps)
            if log_path:
                msg += ' 日志: %s' % log_path
            return True, msg

    # ---------- 取消 ----------
    def stop(self):
        with self._lock:
            if self.current_process is None \
                    and self.state == MappingState.IDLE:
                # 也允许清理外部启动的建图进程
                leftovers = _pids_matching(
                    (_MAPPING_LAUNCH_MARK,) + _CARTO_ONLY)
                if not leftovers:
                    return False, '当前没有在建图'
            ok, msg = _stop_stack(
                _MAPPING_LAUNCH_MARK, _CARTO_ONLY, 'manual_mapping',
                label_cn='建图栈')
            self.current_process = None
            self.launch_time = None
            self.state = MappingState.IDLE if ok else MappingState.ERROR
            self.last_message = msg
            return ok, msg

    # ---------- 保存 ----------
    def save(self, name='room1', directory='', overwrite=True,
             save_pbstream=True):
        """调用 /manual_mapping/save_map 保存地图。

        服务类型 custom_interfaces/srv/SaveMap，字段：
            name, directory, overwrite, save_pbstream
        """
        if not name or not name.strip():
            return False, '地图名不能为空'
        name = name.strip()
        # 防目录穿越
        if os.path.basename(name) != name:
            return False, '非法地图名: %s' % name

        with self._lock:
            if self.state != MappingState.MAPPING:
                return False, '请先开始建图再保存（当前状态：%s）' % (
                    self.state.value)

        prev_state = self.state
        self.state = MappingState.SAVING
        # 布尔值在 ROS 2 服务 YAML/SRV 语法里必须小写 true/false
        req = (
            "{name: '%s', directory: '%s', overwrite: %s, save_pbstream: %s}"
            % (name, directory, 'true' if overwrite else 'false',
               'true' if save_pbstream else 'false')
        )
        command = f"""
        export ROS_DOMAIN_ID=0 && \
        source /opt/ros/humble/setup.bash && \
        source {self.workspace_dir}/install/setup.bash && \
        timeout 120 ros2 service call /manual_mapping/save_map \
            custom_interfaces/srv/SaveMap "{req}"
        """
        try:
            proc = subprocess.run(
                ['bash', '-c', command],
                capture_output=True, text=True, timeout=140)
        except subprocess.TimeoutExpired:
            self.state = prev_state
            return False, '保存超时（140s），检查 map_saver 是否卡住'
        except Exception as exc:                 # noqa: BLE001
            self.state = prev_state
            return False, '调用保存服务失败: %s' % exc

        out = (proc.stdout or '') + (proc.stderr or '')
        # 服务的 success 字段为 false 或进程非 0 都算失败
        success = (proc.returncode == 0 and 'success=True' in out.replace(' ', ''))
        self.state = prev_state

        if success:
            self.last_saved = {
                'name': name,
                'time': time.strftime('%H:%M:%S'),
            }
            rel = os.path.join(
                'src', 'fishbot_navigation2', 'maps', name + '.yaml')
            return True, '地图已保存: %s' % rel

        # 把服务返回的 message 抽出来给用户看
        detail = ''
        for line in out.splitlines():
            if 'message=' in line:
                detail = line.split('message=', 1)[1].strip().strip("'\"")
                break
        if not detail:
            tail = [ln for ln in out.strip().splitlines() if ln.strip()][-3:]
            detail = ' | '.join(tail) if tail else '无输出'
        return False, '保存失败: %s' % detail


# ==================== 连接管理函数 ====================
# ============================================================
# 精确进程清理
# ------------------------------------------------------------
# 设计要点（每一条都对应一次实际踩坑）：
#   1. 名字独特的可执行文件，直接按路径特征匹配（tcp_server、ydlidar_node、
#      auto_localizer 等），不会撞其他项目。
#   2. 名字太通用的（robot_state_publisher、rviz2、amcl、map_server、
#      component_container_isolated…）**必须二次确认**：该进程的
#      AMENT_PREFIX_PATH 里有本工作空间的 install 目录，或命令行里出现
#      本工作空间路径。否则会误杀别的 ROS 项目 —— 老实现里的
#      `pkill -9 -f 'ydlidar'` 就是这类隐患（还会误杀 respawn 管理的节点）。
#   3. 绝不杀自己和自己的祖先进程，也绝不 killpg 自己的进程组。
#   4. 先 SIGTERM 让节点自己收尾，超时才 SIGKILL。
#   5. 僵尸进程（state=Z）杀不掉，只能由父进程 wait 回收；是自己孩子
#      就 waitpid 掉，否则如实报告。
#   6. 孤儿进程（父进程已被接管为 PID 1 / systemd --user）同样按上面的
#      特征匹配扫出来，一并清理。
#   7. 先杀子孙再杀父，避免父进程又重新拉起子进程。
# ============================================================

# 名字足够独特，看命令行就能确定是本项目的进程
_UNIQUE_MARKS = (
    'ros_serial2wifi/lib/ros_serial2wifi/tcp_server',
    'ydlidar/lib/ydlidar/ydlidar_node',
    'fishbot_voice/lib/fishbot_voice/voice_control_node',
    'fishbot_bringup/lib/fishbot_bringup/',          # odom2tf / 注入 / IMU
    'fishbot_navigation2/lib/fishbot_navigation2/auto_localizer.py',
    'micro_ros_agent',
    'micro-ros-agent',
    # ★ 导航子进程（2026-11-01 补）
    # 这两个由 nav_manager 用 preexec_fn=os.setsid 拉起，自成会话组，
    # web_controller 一重启就彻底失去管控。实测 nav_to_pose 在 nav2 关闭
    # 后卡在 isTaskComplete() 死循环里跑了二十多分钟，一直占着
    # /navigate_to_pose 的 action client，导致后续导航全部异常。
    'fishbot_application/lib/fishbot_application/nav_to_pose',
    'autopatrol_robot/lib/autopatrol_robot/patrol_node',
    # 建图模式（manual_mapping.launch.py）会起这些
    'cartographer_ros/cartographer_node',
    'cartographer_occupancy_grid_node',
)

# 通用名，必须再确认属于本工作空间
_GENERIC_MARKS = (
    # ★ ros2 launch 包装进程必须被匹配到！
    # 否则杀完子节点后，存活的 launch 会立刻把带 respawn=True 的节点
    # （ydlidar、voice）重新拉起，用户看到的就是「点了强制清理但雷达又
    # 自己回来了」。这个靠 _belongs_to_ws 二次确认，不会误杀别的项目。
    'ros2 launch',
    'robot_state_publisher',
    'joint_state_publisher',
    'component_container_isolated',
    'nav2_container',
    'ekf_node',
    'rviz2',
    'map_server',
    'amcl',
    'planner_server',
    'controller_server',
    'bt_navigator',
    'behavior_server',
    'waypoint_follower',
    'velocity_smoother',
    'smoother_server',
    'lifecycle_manager',
)


def _ws_marker():
    """本工作空间 install 目录的绝对路径，用于判定进程归属。"""
    return os.path.join(WORKSPACE_DIR, 'install')


def _read_cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as fp:
            return fp.read().replace(b'\x00', b' ').decode('utf-8',
                                                           'replace')
    except OSError:
        return ''


def _read_environ(pid):
    try:
        with open('/proc/%d/environ' % pid, 'rb') as fp:
            return fp.read().replace(b'\x00', b'\n').decode('utf-8',
                                                            'replace')
    except OSError:
        return ''


def _read_state(pid):
    """读进程状态字母；Z=僵尸。"""
    try:
        with open('/proc/%d/stat' % pid, 'r') as fp:
            data = fp.read()
        # comm 字段可能含空格/括号，取最后一个 ')' 之后的部分
        return data[data.rfind(')') + 2:].split()[0]
    except (OSError, IndexError):
        return ''


def _belongs_to_ws(pid):
    """通用名进程是否属于本工作空间。"""
    marker = _ws_marker()
    if marker in _read_environ(pid):
        return True
    return marker in _read_cmdline(pid) or WORKSPACE_DIR in _read_cmdline(pid)


def _collect_targets():
    """扫描 /proc，返回 (待杀 pid 集合, 僵尸列表, 受保护 pid 集合)。"""
    me = os.getpid()
    # 保护自己和所有祖先，避免把 web_controller 及其终端杀掉
    protected = {me}
    pid = me
    for _ in range(12):
        try:
            with open('/proc/%d/stat' % pid, 'r') as fp:
                data = fp.read()
            ppid = int(data[data.rfind(')') + 2:].split()[1])
        except (OSError, IndexError, ValueError):
            break
        if ppid <= 1:
            break
        protected.add(ppid)
        pid = ppid

    targets, zombies = set(), []
    try:
        pids = [int(d) for d in os.listdir('/proc') if d.isdigit()]
    except OSError:
        pids = []

    for pid in pids:
        if pid in protected:
            continue
        cmd = _read_cmdline(pid)
        if not cmd:
            continue

        hit = any(mark in cmd for mark in _UNIQUE_MARKS)
        if not hit:
            hit = (any(mark in cmd for mark in _GENERIC_MARKS)
                   and _belongs_to_ws(pid))
        if not hit:
            continue

        if _read_state(pid) == 'Z':
            zombies.append((pid, cmd.strip()[:70]))
        else:
            targets.add(pid)
    return targets, zombies, protected


def _descendants_first(pids):
    """按「后代优先」排序，避免父进程先死又拉起子进程。"""
    depth = {}
    for pid in pids:
        d, cur = 0, pid
        for _ in range(16):
            try:
                with open('/proc/%d/stat' % cur, 'r') as fp:
                    data = fp.read()
                ppid = int(data[data.rfind(')') + 2:].split()[1])
            except (OSError, IndexError, ValueError):
                break
            if ppid in pids:
                d += 1
                cur = ppid
            else:
                break
        depth[pid] = d
    return sorted(pids, key=lambda p: depth.get(p, 0), reverse=True)


def _terminate(pids, term_wait=3.0):
    """SIGTERM → 等待 → SIGKILL，返回 (已杀, 仍存活)。"""
    if not pids:
        return [], []

    my_pgid = os.getpgrp()
    for pid in _descendants_first(pids):
        try:
            if os.getpgid(pid) == my_pgid:
                # 同一进程组（说明不是 setsid 起的），只杀单个进程
                os.kill(pid, signal.SIGTERM)
            else:
                os.killpg(os.getpgid(pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    deadline = time.time() + term_wait
    while time.time() < deadline:
        alive = [p for p in pids
                 if os.path.exists('/proc/%d' % p) and _read_state(p) != 'Z']
        if not alive:
            break
        time.sleep(0.15)

    alive = [p for p in pids
             if os.path.exists('/proc/%d' % p) and _read_state(p) != 'Z']
    for pid in _descendants_first(alive):
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass
    time.sleep(0.3)

    still = [p for p in pids
             if os.path.exists('/proc/%d' % p) and _read_state(p) != 'Z']
    return [p for p in pids if p not in still], still


def kill_process_tree(pid, term_wait=3.0):
    """杀掉某个进程及其整棵树（用于我们亲自 Popen 的那两个 launch）。"""
    try:
        parent = psutil.Process(pid)
        procs = parent.children(recursive=True) + [parent]
    except psutil.NoSuchProcess:
        return
    except Exception as exc:  # noqa: BLE001
        print(f"获取进程树失败: {exc}")
        return

    my_pgid = os.getpgrp()
    # 子进程先死
    for proc in procs:
        try:
            if os.getpgid(proc.pid) == my_pgid:
                proc.terminate()
            else:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:  # noqa: BLE001
            pass

    _, alive = psutil.wait_procs(procs, timeout=term_wait)
    for proc in alive:
        try:
            proc.kill()
        except Exception:  # noqa: BLE001
            pass
    psutil.wait_procs(alive, timeout=1.0)


def _reap_zombies(zombies):
    """僵尸进程无法被 kill，只能由父进程 wait 回收。"""
    for pid, desc in zombies:
        reaped = False
        try:
            # 只有自己的孩子才能 wait
            wpid, _ = os.waitpid(pid, os.WNOHANG)
            reaped = (wpid == pid)
        except (ChildProcessError, OSError):
            pass
        if reaped:
            print(f"  已回收僵尸 pid={pid} ({desc})")
        else:
            print(f"  ⚠️ 僵尸 pid={pid} ({desc}) 的父进程不是本服务，"
                  f"需由 systemd 回收；它不占 CPU/内存，不影响重启")


def _cleanup_orphan_shm():
    """清理无人引用的 Fast-DDS 共享内存段。

    为什么需要：Fast DDS 用 /dev/shm/fastrtps_* 做零拷贝传输，正常退出时
    自己删；但被 SIGKILL 的节点会把这些段漏下来。反复重启几十次后
    /dev/shm 里会积上千个段，轻则占内存，重则导致新节点报
    "Failed init_port fastrtps_portXXXX: open_and_lock_file failed"。

    做法（精确，不会误删）：
      扫所有活进程的 /proc/<pid>/maps，把仍被 mmap 的 /dev/shm 文件记下来，
      只删那些没有任何活进程映射的。因此正在跑的 ROS 节点（包括别的
      工作空间的）都不会受影响。
    """
    shm_dir = '/dev/shm'
    try:
        entries = [e for e in os.listdir(shm_dir)
                   if e.startswith('fastrtps')]
    except OSError:
        return 0
    if not entries:
        return 0

    in_use = set()
    try:
        pids = [d for d in os.listdir('/proc') if d.isdigit()]
    except OSError:
        return 0
    for p in pids:
        try:
            with open('/proc/%s/maps' % p, 'r') as fp:
                for line in fp:
                    if '/dev/shm/' in line:
                        in_use.add(os.path.basename(line.split()[-1]))
        except (OSError, IndexError):
            continue

    removed = 0
    for name in entries:
        if name in in_use:
            continue
        try:
            os.remove(os.path.join(shm_dir, name))
            removed += 1
        except OSError:
            pass
    if removed:
        print(f"已清理 {removed} 个孤儿 Fast-DDS 共享内存段")
    return removed


def _cleanup_launch_temps():
    """清理 launch 遗留的临时参数文件（/tmp/launch_params_*）。

    launch_ros 会为每个带参数的 Node 写一个临时 yaml（进程命令行里能看到
    --params-file /tmp/launch_params_xxx），正常退出时由 launch 删除；
    但被 SIGKILL 的会留下来。实测几十次重启能积上百个（本项目当时 74 个）。

    安全做法：只删那些没被任何活进程命令行引用的。
    """
    pat = re.compile(r'/tmp/launch_params_[A-Za-z0-9_]+')
    referenced = set()
    try:
        pids = [d for d in os.listdir('/proc') if d.isdigit()]
    except OSError:
        return 0
    for p in pids:
        try:
            with open('/proc/%s/cmdline' % p, 'rb') as fp:
                raw = fp.read().decode('utf-8', 'replace')
        except OSError:
            continue
        for m in pat.finditer(raw):
            referenced.add(m.group(0))

    removed = 0
    try:
        names = os.listdir('/tmp')
    except OSError:
        return 0
    for name in names:
        if not name.startswith('launch_params_'):
            continue
        path = os.path.join('/tmp', name)
        if path in referenced:
            continue
        try:
            if os.path.isfile(path):
                os.remove(path)
                removed += 1
        except OSError:
            pass
    if removed:
        print(f"已清理 {removed} 个 launch 临时参数文件")
    return removed


def force_cleanup_all():
    """精确清理本项目所有进程，不误杀其他程序。"""
    global robot_process, is_connected, nav_service

    print("=== 开始强制清理 ===")
    killed_total = 0

    # ---------- 0. 先停跟随类进程 ----------
    # 它们会持续往 /cmd_vel 发指令。如果先杀底盘后杀跟随，
    # 跟随会因为 /scan 消失而一直发零速度（无害），
    # 但如果反过来，就会出现「底盘还在、跟随还在跑」的空窗。
    # 所以最优先停掉它们。
    try:
        from autopatrol_robot.tools.lidar_follow_tools import (
            is_lidar_follow_running, stop_lidar_follow_process)
        if is_lidar_follow_running():
            print("清理雷达跟随进程...")
            stop_lidar_follow_process(force=True)
            killed_total += 1
    except Exception as exc:  # noqa: BLE001
        print(f"清理雷达跟随失败: {exc}")

    try:
        from autopatrol_robot.tools.follow_tools import (
            is_follow_mode_running, stop_follow_mode_process)
        if is_follow_mode_running():
            print("清理看护模式进程...")
            stop_follow_mode_process(force=True)
            killed_total += 1
    except Exception as exc:  # noqa: BLE001
        print(f"清理看护模式失败: {exc}")

    # ---------- 1. 我们亲自 Popen 的两个 launch（整组连同子进程）----------
    for label, proc in (('bringup', robot_process),
                        ('navigation2', map_manager.current_process)):
        if proc is not None and proc.poll() is None:
            print(f"清理{label}进程: PID={proc.pid}")
            kill_process_tree(proc.pid)
            killed_total += 1
        if proc is not None:
            # 显式收割，避免留下僵尸（Popen 对象被丢弃时也会这么做，
            # 但显式调用更可控）
            try:
                proc.poll()
            except Exception:  # noqa: BLE001
                pass
    robot_process = None
    map_manager.current_process = None

    # ---------- 2. NavigationService（取消导航 + 销毁节点 + 停 spin 线程）----------
    if nav_service:
        print("清理 NavigationService...")
        try:
            nav_service.shutdown()
        except Exception as exc:  # noqa: BLE001
            print(f"NavigationService 清理异常: {exc}")
        nav_service = None

    # ---------- 2.5 停掉 /map 轮询线程，否则它会在清理后继续跑，
    #              还可能把刚重置的状态又改成「加载失败」----------
    map_manager._load_cancel.set()
    if map_manager.load_timer and map_manager.load_timer.is_alive():
        map_manager.load_timer.join(timeout=2.0)
    map_manager.load_timer = None

    # ---------- 3. 扫 /proc 找出孤儿 / 残留（含上一次运行遗留的）----------
    targets, zombies, protected = _collect_targets()
    print(f"扫描到 {len(targets)} 个待清理进程"
          f"（已保护自身与祖先 {len(protected)} 个）")
    if targets:
        killed, still = _terminate(targets)
        killed_total += len(killed)
        if still:
            print(f"  ⚠️ 仍有 {len(still)} 个进程未退出: {still}")
    if zombies:
        print(f"发现 {len(zombies)} 个僵尸进程：")
        _reap_zombies(zombies)

    # ---------- 4. 清理串口软链 ----------
    # 必须在上面的 tcp_server 都退出后再删，否则它会立刻重建
    if os.path.lexists('/tmp/tty_laser'):
        try:
            os.remove('/tmp/tty_laser')
            print("已删除串口软链 /tmp/tty_laser")
        except OSError as exc:
            print(f"删除 /tmp/tty_laser 失败: {exc}")
    for stale in ('/tmp/tty_t5', '/tmp/tty_test', '/tmp/tty_test2',
                  '/tmp/tty_test3', '/tmp/tty_test4'):
        if os.path.lexists(stale):
            try:
                os.remove(stale)
            except OSError:
                pass

    # ---------- 5. 端口占用核查（只报告，不盲目 fuser -k）----------
    # 老实现用 `fuser -k 8888/tcp`，那是按端口无差别杀进程，
    # 万一端口被别人占用就会误杀。这里只核查并如实报告。
    for port, who in ((8888, 'micro-ROS agent'), (8889, '雷达桥接'),
                      (5000, 'web 服务')):
        holder = _port_holder(port)
        if holder and holder != os.getpid():
            print(f"  ⚠️ 端口 {port}（{who}）仍被 pid={holder} 占用"
                  f"（{_read_cmdline(holder).strip()[:60]}）")

    # ---------- 5.5 关闭 launch 日志句柄 ----------
    for name in list(_launch_logs.keys()):
        _close_launch_log(name)

    # ---------- 5.6 清理残留的 Fast-DDS 共享内存段 ----------
    _cleanup_orphan_shm()

    # ---------- 5.7 清理 launch 遗留的临时参数文件 ----------
    _cleanup_launch_temps()

    # ---------- 6. 重置状态 ----------
    is_connected = False
    map_manager.current_state = MapState.NOT_LOADED
    map_manager.map_image_base64 = None
    map_manager.current_map_name = ''
    map_manager.current_map_path = ''

    time.sleep(0.5)
    print(f"=== 强制清理完成（终止 {killed_total} 个进程组/进程）===")
    return killed_total


def _port_holder(port):
    """返回监听指定 TCP 端口的 pid，没有则 None。"""
    try:
        for conn in psutil.net_connections(kind='tcp'):
            if conn.status == psutil.CONN_LISTEN and conn.laddr \
                    and conn.laddr.port == port:
                return conn.pid
    except Exception:  # noqa: BLE001
        pass
    return None

# ==================== ROS2 节点类 ====================
class CarController(Node):
    def __init__(self, map_manager, socketio):
        super().__init__('web_car_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()
        self.last_command_time = time.time()
        self.map_manager = map_manager
        self.socketio = socketio
        
        # 订阅地图
        #
        # ★ QoS 必须是 TRANSIENT_LOCAL。
        #   /map 的发布者 map_server 用 TRANSIENT_LOCAL（latched），
        #   而且**只在激活时发一次**。
        #
        #   用默认 QoS（RELIABLE + VOLATILE）会怎样：
        #     DDS 层面能连上（publisher 的 durability 比 subscriber 强，
        #     算兼容），但 VOLATILE 订阅者**收不到历史消息**。
        #     而 map_server 之后再也不发 → map_callback 永不触发
        #     → 网页永远拿不到地图图片，且不报任何错。
        #
        #   实测对比（ros2 topic info /map --verbose）：
        #     amcl / global_costmap / auto_localizer / rviz2 全是
        #     TRANSIENT_LOCAL，只有 web_car_controller 是 VOLATILE
        #     —— 所以 rviz 有图、网页没图。
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
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
        """地图回调函数 - 保持原始分辨率"""
        # ★ 先置就绪信号：_wait_map_ready 在等它。
        #   放在 try 外面，即使后面的图像转换炸了，
        #   也不能把「地图确实到达」这个事实丢掉。
        try:
            self.map_manager.mark_map_ready()
        except Exception:  # noqa: BLE001
            pass
        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            
            self.get_logger().info(f"接收地图: {width}x{height}, 分辨率: {resolution}m/px")
            
            img_array = np.zeros((height, width), dtype=np.uint8)
            data = np.array(msg.data, dtype=np.int8)
            data_reshaped = data.reshape((height, width))
            
            img_array = np.where(data_reshaped == -1, 127, img_array)
            img_array = np.where(data_reshaped == 0, 255, img_array)
            img_array = np.where(data_reshaped == 100, 0, img_array)
            
            mask = (data_reshaped > 0) & (data_reshaped < 100)
            img_array[mask] = 255 - (data_reshaped[mask] * 255 / 100).astype(np.uint8)
            
            img = Image.fromarray(img_array, mode='L')
            
            buffer = io.BytesIO()
            img.save(buffer, format='PNG', compress_level=6)
            img_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
            
            self.map_manager.set_map_data(
                img_base64, resolution, origin_x, origin_y, width, height
            )
            
            self.socketio.emit('map_data', {
                'image': img_base64,
                'resolution': resolution,
                'origin_x': origin_x,
                'origin_y': origin_y,
                'width': width,
                'height': height
            })
            
            self.get_logger().info(f"地图已发送: {width}x{height}, 分辨率: {resolution}m/px")
            
        except Exception as e:
            self.get_logger().error(f"地图处理失败: {str(e)}")
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
# ★ 显式指定 static_folder。
#   本文件既可能被 `python3 web_controller.py` 直接运行（此时 __name__
#   是 '__main__'），也可能被别的模块 import，两种情况下 Flask 猜测的
#   root_path 不保证一致。写死绝对路径最稳，否则 /static/icons/*.png
#   可能 404 —— 表现为「图标怎么都不显示、添加到主屏是空白方块」。
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__, static_folder=os.path.join(_BASE_DIR, 'static'))
app.config['SECRET_KEY'] = 'secret!'
# 模板自动重载：Flask 默认在非 debug 模式下会缓存已编译的模板，
# 于是改完 control.html 刷新浏览器看到的还是旧页面（本次实际踩到，
# 排查了很久「改了没生效」）。这个项目就是要在真机上迭代 UI，
# 打开自动重载比每次重启整个服务方便得多。
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.jinja_env.auto_reload = True
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
    # ★ 必须在这个函数里 import。
    # 原来这两行写在 _setup_nav_callbacks() 内部，但本函数是**另一个**
    # 独立函数，Python 作用域不共享 → 运行时 NameError:
    #   name 'SystemMessage' is not defined
    # 表现为每次导航完成都打印「[NavAnnounce] 生成播报失败」，
    # 用户永远听不到"我到家了"这类主动播报。
    from langchain_core.messages import HumanMessage, SystemMessage

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
mapping_manager = MappingManager()
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

    # Key 缺失时直接告知，不要拿占位值去请求 API 再抛 401
    _missing = _api_key_missing_message()
    if _missing:
        emit('chat_reply', {'reply': '❌ ' + _missing, 'nav_phase': ''})
        return

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

    # ★ 没配 Key 就别去请求了。否则 DeepSeek 会返回一句
    #   「401 Authentication Fails, Your api key: ****o-ai is invalid」
    #   那个 `o-ai` 是 placeholder 的尾巴，用户根本看不出真正原因。
    _key_missing = _check_api_key_or_none()
    if _key_missing:
        return _key_missing

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
        return jsonify({'status': 'error',
                        'message': _friendly_ai_error(e)}), 500
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
        #
        # 跟随类进程必须先停：bringup 会重启雷达和底盘，
        # 跟随节点在雷达中断期间会一直发零速度，
        # 更糟的是新底盘连上后它可能立刻开始控制，用户完全没预期。
        try:
            from autopatrol_robot.tools.lidar_follow_tools import (
                is_lidar_follow_running, stop_lidar_follow_process)
            if is_lidar_follow_running():
                print("连接机器人前先停止雷达跟随")
                stop_lidar_follow_process(force=True)
        except Exception as exc:  # noqa: BLE001
            print(f"停止雷达跟随失败（忽略）: {exc}")

        os.system("pkill -9 -f 'micro_ros_agent' 2>/dev/null")
        os.system("pkill -9 -f 'micro-ros-agent' 2>/dev/null")
        # ★ 不要用 pkill -f 'ydlidar'：会把 bringup 里的 ydlidar_node
        #   一起误杀（再由 respawn 拉起，雷达白白中断 5 秒）。
        #   这里只需要清理可能残留的旧 launch 壳进程。
        os.system("pkill -9 -f 'ros2 launch .*ydlidar_launch.py' 2>/dev/null")
        
        workspace_dir = WORKSPACE_DIR
        launch_path = os.path.join(workspace_dir, 'src', 'fishbot_bringup', 'launch', 'bringup.launch.py')

        # 路径不存在时给出明确原因，而不是让 ros2 launch 抛一堆栈后只报退出码 1
        if not os.path.isfile(launch_path):
            msg = (f'找不到启动文件: {launch_path}。'
                   f'请设置环境变量 FISHBOT_WS 指向工作空间根目录'
                   f'（当前识别为 {workspace_dir}）')
            print(msg)
            return jsonify({'status': 'error', 'message': msg, 'connected': False}), 500
        
        full_command = f"""
        export ROS_DOMAIN_ID=0 && \
        source /opt/ros/humble/setup.bash && \
        source {workspace_dir}/install/setup.bash && \
        ros2 launch {launch_path}
        """
        
        print(f"执行命令: {full_command}")
        
        log_stream, log_path = _launch_log_stream('bringup', workspace_dir)
        robot_process = subprocess.Popen(
            ['bash', '-c', full_command],
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        
        time.sleep(3)
        
        if robot_process.poll() is None:
            is_connected = True
            print("机器人连接成功")
            msg = '连接成功'
            if log_path:
                msg += f'（日志: {log_path}）'
            return jsonify({'status': 'success', 'message': msg,
                            'connected': True})
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


# ==================== PWA 资源 ====================
# 为什么不用 /static/xxx 而要在根路径暴露这两个文件：
#   * Service Worker 的作用域默认是它自己的 URL 所在目录。
#     如果从 /static/sw.js 加载，scope 就只有 /static/，无法控制 / 页面。
#     放根路径 + Service-Worker-Allowed 头最省事。
#   * manifest.json 放根路径可以让 scope/start_url 都是 '/'，语义更清楚。
@app.route('/manifest.json')
def pwa_manifest():
    """PWA 应用清单。"""
    resp = send_from_directory(
        app.static_folder, 'manifest.json',
        mimetype='application/manifest+json')
    # 改了 manifest 之后要能立刻生效，不做强缓存
    resp.headers['Cache-Control'] = 'no-cache'
    return resp


@app.route('/sw.js')
def pwa_service_worker():
    """Service Worker 脚本。

    注意：Service Worker 只在 **HTTPS 或 localhost** 这样的 secure
    context 下才会被浏览器注册。当前是 http://<局域网IP>:5000，
    所以 iPhone 上这个 SW 不会生效（浏览器会直接拒绝注册）。
    这不是 bug：iOS 的「添加到主屏幕」并不需要 HTTPS，也不需要 SW，
    靠的是 control.html 里的 apple-mobile-web-app-capable。
    这里照样把文件提供出去，将来若上了 HTTPS 就自动生效。
    """
    resp = send_from_directory(
        app.static_folder, 'sw.js',
        mimetype='application/javascript')
    resp.headers['Service-Worker-Allowed'] = '/'
    resp.headers['Cache-Control'] = 'no-cache'
    return resp

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
    """载入地图。当前固定加载 room1，不接受前端参数。"""
    try:
        success, message = map_manager.load_map(on_map_status)
        
        if success:
            return jsonify({
                'status': 'success',
                'message': message,
                'map': map_manager.current_map_name,
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


# ==================== 建图控制 ====================
@app.route('/mapping/start', methods=['POST'])
def mapping_start():
    """开始建图。

    会先停掉导航栈（Cartographer 与 AMCL 都发布 map→odom，不能并存），
    但保留 bringup 的 micro-ROS agent / 雷达 / EKF，ESP32 无需断电。
    """
    try:
        data = request.get_json(silent=True) or {}
        ok, message = mapping_manager.start()
        payload = {
            'status': 'success' if ok else 'error',
            'message': message,
            'state': mapping_manager.get_state(),
        }
        if not ok:
            return jsonify(payload), 400
        if data.get('announce', True):
            socketio.emit('log', {
                'message': '🗺️ 建图模式已启动，导航栈已停止',
                'type': 'success'})
        return jsonify(payload)
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/mapping/stop', methods=['POST'])
def mapping_stop():
    """取消建图（不保存）。"""
    try:
        ok, message = mapping_manager.stop()
        payload = {
            'status': 'success' if ok else 'error',
            'message': message,
            'state': mapping_manager.get_state(),
        }
        return jsonify(payload) if ok else (jsonify(payload), 400)
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/mapping/save', methods=['POST'])
def mapping_save():
    """保存地图：转调 /manual_mapping/save_map 服务。"""
    try:
        data = request.get_json(silent=True) or {}
        ok, message = mapping_manager.save(
            name=str(data.get('name') or DEFAULT_MAP_NAME),
            directory=str(data.get('directory') or ''),
            overwrite=bool(data.get('overwrite', True)),
            save_pbstream=bool(data.get('save_pbstream', True)),
        )
        payload = {
            'status': 'success' if ok else 'error',
            'message': message,
            'state': mapping_manager.get_state(),
        }
        if ok:
            socketio.emit('log', {
                'message': '💾 ' + message, 'type': 'success'})
        return jsonify(payload) if ok else (jsonify(payload), 400)
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/mapping/status', methods=['GET'])
def mapping_status():
    """查询建图状态。"""
    try:
        return jsonify({
            'status': 'success',
            'state': mapping_manager.get_state(),
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
    # Flask-SocketIO >= 5.3 要求显式声明允许使用 Werkzeug 内置服务器。
    # 本项目是局域网内的本地控制台，不是公网生产服务，用开发服务器没问题。
    # 不加这个参数会直接抛：
    #   RuntimeError: The Werkzeug web server is not designed to run in production
    socketio.run(app, host='0.0.0.0', port=5000, debug=False,
                 allow_unsafe_werkzeug=True)

def main():
    global car_controller, ros_busy, nav_service

    # 注册信号处理器，确保 SIGTERM/SIGINT 触发清理
    def _signal_handler(signum, frame):
        print(f"\n收到信号 {signum}，正在关闭服务...")
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rclpy.init()

    # ============================================================
    # 启动时清理上一轮遗留的导航子进程（2026-11-01 实机事故）
    # ------------------------------------------------------------
    # nav_manager 用 preexec_fn=os.setsid 拉起 nav_to_pose / patrol_node，
    # 它们自成会话组，web_controller 一退出就彻底失去管控（不随父进程
    # 的进程组信号退出）。
    #
    # 实测：nav2 被关掉后 nav_to_pose 卡在 isTaskComplete() 死循环里
    # 跑了二十多分钟、日志每秒刷同一行 ETA，一直占着 /navigate_to_pose
    # 的 action client —— 导致之后所有导航都异常，而且用户以为服务已
    # 经全停了。
    #
    # 所以在**每次启动**时就扫一遍，把上一轮的残骸清掉。
    # ============================================================
    try:
        from autopatrol_robot.nav_manager import (
            _reap_orphan_nav_processes, get_global_nav_lock)
        with get_global_nav_lock():
            n = _reap_orphan_nav_processes()
        if n:
            print("🧹 启动清理：已回收 %d 个上一轮遗留的导航进程" % n)
    except Exception as exc:               # noqa: BLE001 - 不能挡住启动
        print("⚠️ 启动清理导航孤儿进程失败（不影响启动）: %s" % exc)

    # 创建 NavigationService（统一导航入口，AI 和 Web 共享）
    # 注意：不在此处 wait_until_ready()——Nav2 由用户通过 Web UI 的
    # "加载地图"按钮启动，之后 NavigationService 首次 navigate_to() 时
    # 会懒加载 PatrolNode 并自动等待 Nav2 就绪。
    nav_service = NavigationService(status_callback=on_nav_status)
    set_global_nav_service(nav_service)

    # 注册导航阶段回调（导航完成/失败时推送到 Web 前端聊天窗口）
    _setup_nav_callbacks()

    car_controller = CarController(map_manager, socketio)
    # 把节点交给 MapManager，用于 /map 就绪判定和诊断日志
    map_manager.bind_ros_node(car_controller)

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
                map_manager.bind_ros_node(car_controller)
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