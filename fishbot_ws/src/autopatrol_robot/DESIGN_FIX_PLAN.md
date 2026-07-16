# 🏗️ Bouteillier Home 网页端 + AI 助手 — 缺陷修复设计方案

> **日期**：2026-06-01  
> **版本**：v1.0  
> **状态**：方案设计（待确认）  
> **范围**：P0（严重）+ P1（重要）+ P2（中等）缺陷修复，共计 11 项

---

## 目录

1. [架构总览（当前 vs 改造后）](#1-架构总览当前-vs-改造后)
2. [核心数据流图](#2-核心数据流图)
3. [系统拓扑图](#3-系统拓扑图)
4. [P0 方案详解](#4-p0-方案详解)
5. [P1 方案详解](#5-p1-方案详解)
6. [P2 方案详解](#6-p2-方案详解)
7. [改动清单与风险](#7-改动清单与风险)

---

## 1. 架构总览（当前 vs 改造后）

### 1.1 当前架构（问题基线）

```
┌──────────────────────────────────────────────────────────┐
│                    web_controller.py                      │
│                                                           │
│  ┌──────────┐  ┌────────────┐  ┌──────────────────┐      │
│  │MapManager│  │NavManager  │  │  CarController    │      │
│  │ (subproc)│  │ (subproc)  │  │  (rclpy Node)     │      │
│  └────┬─────┘  └─────┬──────┘  │                   │      │
│       │               │         │  /map  → map_cb   │      │
│       │               │         │  /amcl → pose_cb   │      │
│       │               │         │  /bat  → bat_cb    │      │
│       │               │         └────────┬──────────┘      │
│       │               │                  │                  │
│  ┌────┴───────────────┴──────────────────┴─────────┐       │
│  │             全局变量（无锁）                       │       │
│  │  car_controller, ros_busy (bool)                │       │
│  │  robot_process, is_connected                     │       │
│  └──────────────────────────────────────────────────┘       │
│                                                           │
│  ┌──────────────────────────────────────────────────┐     │
│  │             线程模型（问题核心）                    │     │
│  │                                                    │     │
│  │  MainThread:  rclpy.spin_once() 轮询              │     │
│  │                    │                               │     │
│  │                    ▼                               │     │
│  │              ┌──────────┐                         │     │
│  │              │ ros_busy │── bool 竞态 ←──┐       │     │
│  │              └──────────┘                 │       │     │
│  │                    │                      │       │     │
│  │         ┌──────────┴──────────┐           │       │     │
│  │         ▼                     ▼           │       │     │
│  │    spin_once()           time.sleep()     │       │     │
│  │    (已连接时)            (AI忙碌时)        │       │     │
│  │                                              │       │     │
│  │  FlaskThread:  /chat → agent_graph.invoke()  │       │     │
│  │                      ↓ ros_busy = True/False ──┘       │     │
│  │                                                    │     │
│  │  FlaskThread2: /chat → (同时调用，ros_busy=竞争)   │     │
│  │  FlaskThread3: /control → car_controller 无锁访问  │     │
│  │                                                    │     │
│  │  ⚠️ 问题总结：                                      │     │
│  │  ① ros_busy 无原子操作，多线程竞态                     │     │
│  │  ② car_controller 无锁，重建期间可能被访问              │     │
│  │  ③ 导航监控线程与 pause_navigation 竞态                │     │
│  └──────────────────────────────────────────────────┘     │
│                                                           │
│  ┌──────────────────────────────────────────────────┐     │
│  │         双聊天路径                               │     │
│  │  SocketIO.on('chat_message')  ──┐                │     │
│  │  HTTP POST /chat          ──────┤                │     │
│  │                                 ▼                │     │
│  │                      agent_graph.invoke()        │     │
│  │                    (InMemorySaver ≠ 线程安全)     │     │
│  └──────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────┘
```

### 1.2 改造后架构

```
┌──────────────────────────────────────────────────────────┐
│                    web_controller.py                      │
│                                                           │
│  ┌──────────┐  ┌────────────┐  ┌──────────────────┐      │
│  │MapManager│  │NavManager  │  │  CarController    │      │
│  │ (subproc)│  │ (subproc)  │  │  (rclpy Node)     │      │
│  └────┬─────┘  └─────┬──────┘  │                   │      │
│       │               │         │  /map  → map_cb   │      │
│       │               │         │  /amcl → pose_cb   │      │
│       │               │         │  /bat  → bat_cb    │      │
│       │               │         └────────┬──────────┘      │
│       │               │                  │                  │
│  ┌────┴───────────────┴──────────────────┴─────────┐       │
│  │         ✅ 全局变量 + 细粒度锁                      │       │
│  │  _car_controller_lock = Lock()                    │       │
│  │  _chat_processing_lock = Lock()  ← 串行化AI请求   │       │
│  │  _nav_state_lock = Lock()       ← 导航状态保护    │       │
│  │  ros_busy → threading.Event (替代 bool)           │       │
│  └──────────────────────────────────────────────────┘       │
│                                                           │
│  ┌──────────────────────────────────────────────────┐     │
│  │         ✅ 改造后线程模型                           │     │
│  │                                                    │     │
│  │  MainThread:  rclpy.spin_once() 循环              │     │
│  │       │  ├─ 使用 threading.Event.wait()            │     │
│  │       │  │   替代 time.sleep() 轮询                 │     │
│  │       │  └─ 不再使用 ros_busy，改为 Event          │     │
│  │       ▼                                            │     │
│  │  while rclpy.ok():                                 │     │
│  │      ros_busy_event.wait(timeout=0.1)  ← 阻塞等待  │     │
│  │      if ros_busy_event.is_set():                   │     │
│  │          ros_busy_event.clear()  # 等待AI完成       │     │
│  │          ros_busy_event.wait()                     │     │
│  │      rclpy.spin_once(self.node, timeout_sec=0.05)  │     │
│  │                                                    │     │
│  │  FlaskThread: /chat → 获取 _chat_processing_lock   │     │
│  │       │              → ros_busy_event.clear()       │     │
│  │       │              → agent_graph.invoke()         │     │
│  │       │              → ros_busy_event.set()         │     │
│  │       │              → 释放 _chat_processing_lock   │     │
│  │       ▼                                            │     │
│  │  其他 FlaskThread: /load_map, /control 等          │     │
│  │       ├─ car_controller 操作 → 先获取锁             │     │
│  │       └─ 导航相关操作 → 先获取导航锁                │     │
│  └──────────────────────────────────────────────────┘     │
│                                                           │
│  ┌──────────────────────────────────────────────────┐     │
│  │         ✅ 单聊天路径                             │     │
│  │  HTTP POST /chat  ← 唯一入口                      │     │
│  │       │  + _chat_processing_lock 串行化            │     │
│  │       ▼                                           │     │
│  │  SocketIO.on('chat_message') → 已移除              │     │
│  │  agent_graph.invoke() 一次只允许一个调用            │     │
│  └──────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────┘
```

---

## 2. 核心数据流图

### 2.1 聊天指令数据流（改造后）

```
用户输入 ──→ HTTP POST /chat
                │
                ▼
         [_chat_processing_lock.acquire()]
                │
                ▼
         [ros_busy_event.clear()]  ← 通知主线程暂停 spin
                │
                ▼
         agent_graph.invoke({"messages": [...]})
                │
                ├── 需要导航？
                │      └── agent_tools.ALL_TOOLS
                │            │
                │            ├── navigate_to_location()
                │            │     → PatrolNode.goToPose()
                │            │     → 阻塞等待 Navigation2 完成
                │            │
                │            ├── get_current_position()
                │            │     → PatrolNode 查询 TF
                │            │
                │            ├── start_patrol_mode()
                │            │     → subprocess.Popen 启动摄像头
                │            │     → 后台线程监控进程
                │            │
                │            └── cancel_navigation()
                │                  → PatrolNode.cancelTask()
                │
                ├── 需要位置？
                │      └── list_locations() / list_saved_locations()
                │
                └── 返回文本回复
                      │
                      ▼
                [ros_busy_event.set()]  ← 恢复主线程 spin
                      │
                      ▼
                [_chat_processing_lock.release()]
                      │
                      ▼
                return jsonify({'reply': ...})
                      │
                      ▼
               前端 fetch('/chat') 收到回复 → 显示在聊天框
```

### 2.2 手动控制 + 地图数据流

```
摇杆操作 ──→ JavaScript touch/move 事件
                │
                ▼
         sendManualCommand('custom', linear, angular)
                │  (每50ms重复发送)
                ▼
         HTTP POST /control {cmd, linear, angular}
                │
                ▼
         [_car_controller_lock.acquire()]
                │
                ▼
         car_controller.publish_velocity(linear, angular)
                │
                ▼
         [Twist] → /cmd_vel 话题 → ESP32 micro-ROS
                │
                ▼
         [_car_controller_lock.release()]

        ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─

ESP32 传感器数据 ──→ /odom, /battery_percent, /imu
                          │
                          ▼
                   CarController ROS2 回调
                          │
                    [默认回调组 → 串行执行]
                          │  （改造后：地图回调用 ReentrantCallbackGroup）
                          ▼
                   socketio.emit('map_data'/'robot_pose'/'battery_status')
                          │
                          ▼
                   WebSocket → 前端 canvas 渲染
```

### 2.3 导航任务数据流（含竞态修复）

```
用户点击"开始导航" ──→ HTTP POST /start_navigation
                           │
                           ▼
                    nav_manager.start_navigation()
                           │
                           ▼
                    [_nav_state_lock.acquire()]
                           │
                           ▼
                    state = NAVIGATING
                    [_nav_state_lock.release()]
                           │
                           ▼
                    _execute_next_command()
                           │
                           ├── cmd1: 航点一
                           ├── cmd2: 航点二
                           ├── cmd3: 航点三
                           └── cmd4: nav_to_pose (返航)
                                  │
                                  ▼
                           subprocess.Popen + 监控线程
                                  │
                   ┌────────────────┴──────────────┐
                   │                               │
                   ▼                               ▼
           进程正常退出                        用户点击暂停
                   │                               │
                   ▼                               ▼
       [_nav_state_lock]                  POST /pause_navigation
       检查 state 是否仍为                           │
       NAVIGATING                           [_nav_state_lock]
       否 → 不继续执行                       state = PAUSED
       是 → 执行下一命令                      kill 进程
                                            current_process = None
                                            [_nav_state_lock]
                                                    │
                                                    ▼
                                            ✅ 监控线程检查 state != NAVIGATING
                                               跳过剩余队列
```

---

## 3. 系统拓扑图

```
┌─────────────────────────────────────────────────────────────────┐
│                         🏠 家居环境                              │
│                                                                  │
│   ┌─────────────────┐                  ┌─────────────────────┐  │
│   │   💻 控制 PC      │                  │   🤖 机器人本体      │  │
│   │  (Ubuntu 22.04)  │                  │   (ESP32 + ROS2)    │  │
│   │                   │                  │                     │  │
│   │  ┌─────────────┐  │  Ethernet/WiFi  │  ┌───────────────┐  │  │
│   │  │ web_controller│◄├─────HTTP──────►│  │ micro-ROS     │  │  │
│   │  │ (Flask:5000)  │  │                │  │ /cmd_vel       │  │  │
│   │  │ ▲            │  │                │  │ /odom          │  │  │
│   │  │ │ WebSocket  │  │                │  │ /battery       │  │  │
│   │  │ ▼            │  │                │  │ /imu           │  │  │
│   │  │ SocketIO     │  │                │  │                │  │  │
│   │  └──────┬───────┘  │                │  └───────┬───────┘  │  │
│   │         │          │                │          │          │  │
│   │  ┌──────┴───────┐  │                │  ┌───────┴───────┐  │  │
│   │  │ CarController │  │                │  │  MPU6050 IMU  │  │  │
│   │  │ (rclpy Node)  │  │                │  │  PCNT编码器    │  │  │
│   │  │               │  │                │  │  MCPWM电机驱动 │  │  │
│   │  │ 订阅:         │  │                │  │  ADC电池检测   │  │  │
│   │  │  /map         │  │                │  └───────────────┘  │  │
│   │  │  /amcl_pose   │  │                │                     │  │
│   │  │  /battery     │  │                │  ┌───────────────┐  │  │
│   │  │ 发布:         │  │                │  │  YDLIDAR      │  │  │
│   │  │  /cmd_vel     │  │                │  │  激光雷达     │  │  │
│   │  └──────┬───────┘  │                │  └───────────────┘  │  │
│   │         │          │                │                     │  │
│   │  ┌──────┴───────┐  │                │  ┌───────────────┐  │  │
│   │  │ Navigation2   │  │                │  │  ESP32-CAM    │  │  │
│   │  │ AMCL定位      │  │                │  │  (detection)  │  │  │
│   │  │ Nav2规划器    │  │                │  │  HTTP:81/stream│  │  │
│   │  │               │  │                │  └───────────────┘  │  │
│   │  │ 发布: /map    │  │                │                     │  │
│   │  │ 发布:/amcl_pose│  │                └─────────────────────┘  │
│   │  └───────────────┘  │                                         │
│   │                     │          ┌─────────────────────────┐    │
│   │  ┌─────────────┐    │  WiFi    │  📱 控制终端（手机/平板） │    │
│   │  │ Agent 智能体 │    │◄────────│                         │    │
│   │  │ LangGraph    │    │ HTTP    │  浏览器访问              │    │
│   │  │ DeepSeek API │    │ WebSkt  │  http://PC_IP:5000       │    │
│   │  └──────┬──────┘    │         │  - 虚拟摇杆              │    │
│   │         │            │         │  - AI 聊天               │    │
│   │  ┌──────┴──────┐    │         │  - 实时地图              │    │
│   │  │ Internet    │    │         │  - 摄像头画面            │    │
│   │  │ DeepSeek    │    │         └─────────────────────────┘    │
│   │  │ API Server  │    │                                         │
│   │  └─────────────┘    │                                         │
│   └─────────────────────┘                                         │
└─────────────────────────────────────────────────────────────────┘

进程边界：
┌─────────────────────────────────────────────────────────────────┐
│ 同一 Python 进程内                                            │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ 主线程: CarController (rclpy Node) + 主 spin 循环        │  │
│  │ Flask 线程池: HTTP 请求处理                               │  │
│  │ SocketIO 线程: WebSocket 推送                             │  │
│  │ Agent 子线程池: LLM 调用（受全局锁保护）                  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│ 外部子进程（subprocess.Popen）：                                 │
│  ┌──────────┐  ┌───────────────┐  ┌────────────────────┐      │
│  │ bringup  │  │ navigation2   │  │ ESP32-CAM 跟随     │      │
│  │ launch   │  │ launch        │  │ launch             │      │
│  └──────────┘  └───────────────┘  └────────────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. P0 方案详解

### 🔴 P0-1：`ros_busy` 竞态条件 → 替换为 `threading.Event`

**现状问题**：

```python
ros_busy = False  # 全局 bool，非原子操作

# 线程A（Flask处理/chat请求）
ros_busy = True              # ← 设置为 True
time.sleep(0.1)
agent.invoke(...)            # ← 长时间运行
ros_busy = False             # ← 如果线程B也设置了True，这里会错误地重置

# 线程B（Flask处理另一个/chat请求）
ros_busy = True              # ← 覆盖，但线程A不知道
agent.invoke(...)
ros_busy = False             # ← 线程A已经设置过False了！

# 主线程
if not ros_busy:
    rclpy.spin_once(...)     # ← 可能在AI正在使用时spin！
```

**改造方案**：

```python
# 方案：threading.Event + 引用计数锁
_ros_busy_event = threading.Event()
_ros_busy_event.set()  # 初始状态：ROS可以spin
_ros_busy_lock = threading.Lock()
_ros_busy_count = 0

def acquire_ros_busy():
    """AI处理前调用：阻止主线程spin"""
    global _ros_busy_count
    with _ros_busy_lock:
        if _ros_busy_count == 0:
            _ros_busy_event.clear()
        _ros_busy_count += 1

def release_ros_busy():
    """AI处理后调用：恢复主线程spin"""
    global _ros_busy_count
    with _ros_busy_lock:
        _ros_busy_count -= 1
        if _ros_busy_count == 0:
            _ros_busy_event.set()

# 主线程循环
while rclpy.ok():
    _ros_busy_event.wait(timeout=0.1)  # 阻塞等待，不轮询
    if not _ros_busy_event.is_set():
        # Event 被清除 → AI正在处理
        continue
    rclpy.spin_once(car_controller, timeout_sec=0.05)

# Flask /chat 路由
def chat():
    acquire_ros_busy()        # 引用计数 +1
    try:
        result = agent.invoke(...)
        return jsonify({'reply': result})
    finally:
        release_ros_busy()    # 引用计数 -1，只有全部释放才恢复spin
```

**数据流图**：

```
时间轴 ────────────────────────────────────────────────────→

请求1 ──→ acquire_ros_busy()  count: 0→1, Event=CLEAR
请求1 ──→ agent.invoke()...
请求2 ──→ acquire_ros_busy()  count: 1→2（Event 保持 CLEAR）
请求1 ──→ release_ros_busy()  count: 2→1（Event 保持 CLEAR ←✅ 正确！）
请求2 ──→ agent.invoke()...（安全，主线程不会spin）
请求2 ──→ release_ros_busy()  count: 1→0, Event=SET  ←✅ 只有最后一个才能恢复
主线程──→ Event.wait() 返回 → spin_once() 恢复
```

---

### 🔴 P0-2：双聊天路径冲突 → 统一为 HTTP 路径

**改造方案**：

1. **移除** `@socketio.on('chat_message')` 处理器
2. **保留** HTTP `POST /chat` 作为唯一聊天入口
3. **前端**只使用 `fetch('/chat', ...)` 方式（当前前端已经这样做了，无需改动）
4. **加全局串行锁**防止并发 AI 调用损坏 `InMemorySaver`

```python
_chat_lock = threading.Lock()

@app.route('/chat', methods=['POST'])
def chat():
    with _chat_lock:  # ← 一次只允许一个AI对话
        acquire_ros_busy()
        try:
            config = {"configurable": {"thread_id": "web-session"}}
            result = agent_graph.invoke(
                {"messages": [HumanMessage(content=user_message)]},
                config=config
            )
            return jsonify({'status': 'success', 'reply': result["messages"][-1].content})
        except Exception as e:
            from agent_tools import reset_patrol_node
            reset_patrol_node()
            # 重试一次
            result = agent_graph.invoke(...)
            return jsonify({'status': 'success', 'reply': ...})
        finally:
            release_ros_busy()
```

---

### 🔴 P0-3：`car_controller` 无锁访问 → 加锁

**改造方案**：

```python
_car_controller_lock = threading.Lock()

def get_car_controller():
    """线程安全地获取 car_controller 引用"""
    with _car_controller_lock:
        return car_controller

# 所有访问 car_controller 的地方改为：
@app.route('/control', methods=['POST'])
def control():
    cc = get_car_controller()
    if cc:
        cc.publish_velocity(...)

# 主线程异常处理：
except IndexError as e:
    with _car_controller_lock:
        try:
            car_controller.destroy_node()
        except:
            pass
        car_controller = CarController(map_manager, socketio)
```

---

## 5. P1 方案详解

### 🟠 P1-4：导航进程监控竞态 → 状态检查 + 线程安全

**改造方案**：

```python
_nav_state_lock = threading.Lock()

class NavigationManager:
    def _execute_next_command(self, status_callback=None):
        # ... 启动进程 ...
        
        def monitor_process():
            returncode = self.current_process.wait()
            
            with _nav_state_lock:  # ← 加锁检查状态
                if self.current_state != NavState.NAVIGATING:
                    # 已被 pause_navigation() 或 force_cleanup() 中断
                    print(f"[Navigation] 跳过剩余队列，当前状态: {self.current_state}")
                    return
                self.current_process = None
            
            if returncode == 0:
                with _nav_state_lock:
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index < len(self.command_queue):
                        # 继续执行下一命令
                        self._execute_next_command(status_callback)
                    else:
                        self.current_state = NavState.COMPLETED
                        status_callback("completed", "所有任务完成")
            else:
                with _nav_state_lock:
                    self.current_state = NavState.IDLE
                    status_callback("error", f"命令失败")

    def pause_navigation(self):
        with _nav_state_lock:
            if self.current_state != NavState.NAVIGATING:
                return False, "没有正在进行的导航"
            
            if self.current_process and self.current_process.poll() is None:
                # ... kill 进程 ...
                self.current_process = None
            self.current_state = NavState.PAUSED
        return True, "导航已暂停"
```

**竞态场景解决后的时间线**：

```
时间轴 ───────────────────────────────────────────────→

监控线程: 子进程退出，returncode=0
监控线程: 获取 _nav_state_lock...
          ─── 暂停线程同时获取 _nav_state_lock（监控线程阻塞）
暂停线程: state = PAUSED, process = None
暂停线程: 释放 _nav_state_lock
          ─── 监控线程获取锁
监控线程: 检查 state → PAUSED（不是 NAVIGATING）
监控线程: return ✅（放弃执行下一命令）
```

---

### 🟠 P1-5：地图盲加载 → 事件驱动

**改造方案**：

```python
class MapManager:
    def __init__(self):
        # ...
        self._map_loaded_event = threading.Event()  # ← 新增
    
    def load_map(self, status_callback=None):
        # ... 启动 navigation2.launch 子进程 ...
        
        # 不再使用固定5秒定时器！
        # 等待 CarController.map_callback 设置事件
        def wait_for_map():
            if self._map_loaded_event.wait(timeout=30.0):  # ← 最长等30秒
                self.current_state = MapState.LOADED
                if status_callback:
                    status_callback("loaded", "地图加载成功")
            else:
                self.current_state = MapState.ERROR
                if status_callback:
                    status_callback("error", "地图加载超时（30秒）")
        
        threading.Thread(target=wait_for_map, daemon=True).start()
        return True, "地图加载已启动"

class CarController(Node):
    def map_callback(self, msg):
        # ... 处理地图 ...
        self.map_manager._map_loaded_event.set()  # ← 通知 MapManager
```

**对比**：

| 指标 | 当前（盲加载） | 改造后（事件驱动） |
|------|---------------|-------------------|
| 加载判定依据 | 固定5秒计时器 | 实际收到 `/map` 话题 |
| 超时处理 | 无条件标记"已加载" | 等待30秒后报错 |
| 虚假成功 | ❌ 可能导航未就绪就判成功 | ✅ 确认真实地图数据到达 |
| 超时灵活性 | 前后端不一致(5s/8s) | 统一后端30秒 |

---

### 🟠 P1-6：主循环 busy-wait → Event.wait()

**已在 P0-1 方案中覆盖** — `_ros_busy_event.wait(timeout=0.1)` 替代 `time.sleep(0.01)` 轮询。

---

### 🟠 P1-7：Agent 工具全局状态非线程安全 → 加锁

**改造方案**：

```python
# agent_tools.py
_patrol_node = None
_patrol_node_error = False
_nav_finished = True
_rclpy_initialized = False

# 新增锁
_patrol_lock = threading.Lock()
_nav_state_lock = threading.Lock()  # 注意：与 web_controller 的锁分开

def get_patrol_node():
    """线程安全地获取 PatrolNode 单例"""
    global _patrol_node, _patrol_node_error
    with _patrol_lock:
        if _patrol_node is not None and not _patrol_node_error:
            return _patrol_node
        # 创建新实例...
        _patrol_node = PatrolNode()
        return _patrol_node

def reset_patrol_node():
    global _patrol_node, _patrol_node_error
    with _patrol_lock:
        if _patrol_node is not None:
            try:
                _patrol_node.publish_nav_status(False)
                _patrol_node.destroy_node()
            except:
                pass
            _patrol_node = None
        _patrol_node_error = False

# 所有使用 _patrol_node 的工具函数都要通过 get_patrol_node() 获取
```

---

## 6. P2 方案详解

### 🟡 P2-8：延迟导入 → 移到文件顶部

```python
# web_controller.py 顶部（约第15行）
from agent_tools import cleanup_ros, reset_patrol_node  # ← 提前导入

# /chat 路由中不再有延迟导入
except Exception as e:
    reset_patrol_node()  # ← 直接调用，无需导入
```

---

### 🟡 P2-9：回调组隔离 → ReentrantCallbackGroup

```python
class CarController(Node):
    def __init__(self, map_manager, socketio):
        super().__init__('web_car_controller')
        
        # 地图回调单独使用 ReentrantCallbackGroup
        map_cb_group = ReentrantCallbackGroup()
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10,
            callback_group=map_cb_group  # ← 隔离
        )
        
        # 其他回调保持默认（MutuallyExclusive）
        self.battery_subscription = self.create_subscription(
            Float32, '/battery_percent', self.battery_callback, 10
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10
        )
```

**效果**：地图处理（图片编码+SocketIO发送）不再阻塞位姿和电量的实时更新。

---

### 🟡 P2-11：API Key 硬编码 → 环境变量

```python
# agent_graph.py
DEEPSEEK_API_KEY = os.environ.get('DEEPSEEK_API_KEY', '')
if not DEEPSEEK_API_KEY:
    raise RuntimeError("请设置环境变量 DEEPSEEK_API_KEY")
DEEPSEEK_BASE_URL = os.environ.get(
    'DEEPSEEK_BASE_URL', 
    'https://api.deepseek.com/v1'
)
```

---

### 🟡 P2-12：`location.reload()` → 精细化状态重置

```javascript
async function forceCleanup() {
    if (!confirm('强制清理将终止所有进程，确认？')) return;
    const btn = document.getElementById('force-cleanup');
    btn.disabled = true;
    btn.textContent = '⏳ 清理中...';
    
    try {
        await fetch('/force_cleanup', { method: 'POST' });
        // ✅ 不再使用 location.reload()
        // 精细化重置 UI 状态
        isConnected = false;
        document.getElementById('conn-status-dot').classList.remove('connected');
        document.getElementById('conn-status-text').textContent = '未连接';
        document.getElementById('map-state-text').textContent = '未加载';
        document.getElementById('close-map').disabled = true;
        document.getElementById('battery-percent').textContent = '--%';
        document.getElementById('battery-level').textContent = '--%';
        document.getElementById('robot-position').textContent = '等待数据...';
        mapImage = null;
        drawMap();
        addLog('清理完成', 'success');
    } catch(err) {
        addLog('清理网络错误: ' + err.message, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = '🧹 强制清理';
    }
}
```

---

## 7. 改动清单与风险

### 7.1 改动文件清单

| 文件 | 改动内容 | 风险等级 |
|------|----------|---------|
| `web_controller.py` | `ros_busy` → `threading.Event` + 引用计数 | ⚠️ 中等 |
| `web_controller.py` | 新增 `_chat_lock`, `_car_controller_lock` | ⚠️ 中等 |
| `web_controller.py` | 移除 SocketIO `chat_message` 处理器 | ⚠️ 低 |
| `web_controller.py` | 主循环改为 `Event.wait()` | ⚠️ 低 |
| `web_controller.py` | 导航监控线程加状态检查 | ⚠️ 低 |
| `web_controller.py` | 地图加载改为事件驱动 | ✅ 安全 |
| `web_controller.py` | 导入移到文件顶部 | ✅ 安全 |
| `web_controller.py` | `CarController` 加回调组隔离 | ✅ 安全 |
| `agent_tools.py` | PatrolNode 相关全局变量加锁 | ⚠️ 中等 |
| `agent_graph.py` | API Key 改为环境变量 | ✅ 安全 |
| `control.html` | `forceCleanup` 改为精细化重置 | ✅ 安全 |

### 7.2 回归测试要点

| 测试场景 | 预期行为 | 涉及改动 |
|----------|---------|---------|
| 连续快速点击 AI 发送 | 串行处理，不丢失响应 | `_chat_lock` |
| AI 处理中手动摇杆 | 速度指令正常发送 | Event 分离, car_controller 锁 |
| AI 处理中查看地图 | 地图更新不卡顿 | Event 分离, 回调组隔离 |
| 导航中快速暂停/恢复 | 状态正确，不意外跳转航点 | `_nav_state_lock` |
| 同时打开两个浏览器标签页 | 各自独立工作 | 全局锁不冲突 |
| 地图加载后快速关闭 | 计时器/事件正确处理 | 事件驱动替代计时器 |
| `force_cleanup` 后 UI | 状态完全重置，页面不刷新 | `location.reload` 移除 |

### 7.3 回滚方案

如果改造后出现问题，可以快速回滚的方式：

1. **每个改动独立提交**（11个独立 commit），可逐条回滚
2. **保留原始文件备份** — 当前目录已有 `control copy.html` 和 `patrol_node（backup）.py`
3. **核心安全网**：`_chat_lock` 和 Event 机制是单独加层，不破坏原有逻辑通路

---

## 附录：实时数据流对比汇总

### 当前（改造前）交互流程

```
用户                            Web前端                     web_controller.py                ROS2系统
 │                               │                              │                              │
 ├─拖动摇杆─────────────────────►│                              │                              │
 │                               ├─POST /control──────────────►│                              │
 │                               │                              ├─car_controller               │
 │                               │                              │ .publish_velocity()─────────►│/cmd_vel
 │                               │                              │                              │
 ├─AI对话───────────────────────►│                              │                              │
 │                               ├─POST /chat ────────────────►│ (线程A)                       │
 │  (再次快速对话)                │                              │ ros_busy=True                 │
 │                               ├─POST /chat ────────────────►│ (线程B)                       │
 │                               │                              │ ros_busy=True ← 覆盖          │
 │                               │                              │ agent.invoke()                │
 │                               │                              │          ...                  │
 │                               │                              │ (线程A结束) ros_busy=False    │
 │                               │                              │ ←⚠️ 线程B还在运行！            │
 │                               │                              │ 主线程: spin_once() ←⚠️ 冲突  │
```

### 改造后交互流程

```
用户                            Web前端                     web_controller.py                ROS2系统
 │                               │                              │                              │
 ├─拖动摇杆─────────────────────►│                              │                              │
 │                               ├─POST /control──────────────►│                              │
 │                               │                              │ [_car_controller_lock]       │
 │                               │                              │ car_controller               │
 │                               │                              │ .publish_velocity()─────────►│/cmd_vel
 │                               │                              │ [_car_controller_lock]       │
 │                               │                              │                              │
 ├─AI对话───────────────────────►│                              │                              │
 │                               ├─POST /chat ────────────────►│ [_chat_lock] ← 排队等待       │
 │  (再次快速对话)                │                              │ acquire_ros_busy()            │
 │                               ├─POST /chat ────────────────►│ count: 0→1, Event=CLEAR      │
 │                               │                              │ agent.invoke()                │
 │                               │                              │    ...                        │
 │                               │                              │ (同时第二个请求到达)           │
 │                               │                              │ [_chat_lock] ← 阻塞等待       │
 │                               │                              │    ... 等待中...              │
 │                               │                              │ agent.invoke() 完成           │
 │                               │                              │ release_ros_busy()            │
 │                               │                              │ count: 1→0, Event=SET        │
 │                               │                              │ [_chat_lock] 释放             │
 │                               │                              │ ← 第二个请求开始处理           │
 │                               │                              │                              │
 │                               │                              │ 主线程: Event.wait()          │
 │                               │                              │ 收到SET → spin_once() ←✅ 安全│
```

---

> **💡 设计原则总结**：所有改造围绕"**线程安全 + 事件驱动**"两个核心原则展开。不改变业务逻辑，只加固并发安全边界。使用 `threading.Lock` 保护共享状态，`threading.Event` 替代忙等待轮询，加锁范围尽可能小以避免死锁。
