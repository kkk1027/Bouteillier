# fishbot_voice

语音控制功能包。

## 职责边界

| | 范围 |
|---|---|
| ✅ 负责 | 语音文本 -> 运动意图的翻译、动作时序控制、`/cmd_vel` 下发 |
| ❌ 不负责 | 串口读取（在 ESP32 固件）、导航、建图、Web 交互、语音播报 |

**设计原则：固件只做搬运，解释权在本包。**
ESP32 把 ASR-01 的串口文本原样发到 `/voice_event`，不做任何解析。
因此新增或修改一条语音命令，只需改本包配置并重启节点（约 1 秒），
不需要重新编译烧录固件（约 30 秒）。

## 接口

### 订阅

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/voice_event` | `std_msgs/String` | **BEST_EFFORT** | ASR-01 原始文本，如 `MOVE_FWD` |
| `/voice_module_status` | `std_msgs/Bool` | **BEST_EFFORT** | 模块在线状态（3 秒无数据 = 离线） |
| `/nav_node_status` | `std_msgs/Bool` | RELIABLE | 导航运行中则忽略运动指令（门控） |

> ⚠️ QoS 必须匹配：ESP32 侧发布器是 `best_effort`，订阅端若用默认
> `RELIABLE` 会因 QoS 不兼容而**收不到任何数据**，且不会报错，只表现为
> 「话题存在但收不到」。

### 发布

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | BEST_EFFORT | 20 Hz 定时连发 |

## 命令词表

在 `config/voice_control.yaml` 的 `command_map` 里配置，格式 `"文本:动作"`。

| 语音文本 | 动作 | 行为 |
|---|---|---|
| `WAKE` | `NONE` | 仅记日志 |
| `MOVE_FWD` | `FORWARD` | 前进 5 秒 |
| `MOVE_BACK` | `BACKWARD` | 后退 5 秒 |
| `MOVE_LEFT` | `TURN_LEFT` | 原地左转 5 秒 |
| `MOVE_RIGHT` | `TURN_RIGHT` | 原地右转 5 秒 |
| `FOLLOW_OFF` | `STOP` | 立即停车（最高优先级） |

动作取值：`NONE` / `FORWARD` / `BACKWARD` / `TURN_LEFT` / `TURN_RIGHT` / `STOP`

## 控制模型

```
语音事件 ──► 命令词表 ──► 动作状态机 ──► 20Hz 定时发布 ──► /cmd_vel
                          │
                          ├─ 方向冲突时插入 pre_stop 过渡（默认 0.2s）
                          └─ 动作结束后补发 stop_burst 帧零速度
```

**为什么必须定时连发，不能只发一次：**
ESP32 固件有「3 秒无 `/cmd_vel` 即自动停车」的保护（防上位机崩溃后小车
冲出去）。只发一次 Twist，车会跑不到 3 秒就自己停。

**为什么方向切换要插入停车过渡：**
前进中直接收到左转指令若不减速，会变成行进中转向，容易打滑、撞墙。
先发 0.2 秒零速度让车停稳，再开始原地转。

## 优先级

```
STOP  >  运动指令  >  导航门控
```

- `STOP`（`FOLLOW_OFF`）：任何时候立即生效，**不受去抖限制、不受导航门控**
- 其余运动指令：可互相打断（后到覆盖先到），但导航运行时被忽略

## 运行

### 单体启动（调试用）

```bash
# 干跑（默认，车不会动）
ros2 launch fishbot_voice voice_control.launch.py

# 实车
ros2 launch fishbot_voice voice_control.launch.py enable_motion:=true
```

### 随 bringup 一起启动（推荐）

`fishbot_bringup` 的 `bringup.launch.py` 已集成本节点，默认启动且实车模式：

```bash
ros2 launch fishbot_bringup bringup.launch.py
#   -> 语音控制、micro-ROS agent、雷达、TF 全部就绪，喊一句就动
```

可用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `enable_voice` | true | 是否启动语音控制节点 |
| `enable_voice_motion` | true | 语音是否驱动底盘 |

```bash
# 只跑链路不动车
ros2 launch fishbot_bringup bringup.launch.py enable_voice_motion:=false

# 不启动语音
ros2 launch fishbot_bringup bringup.launch.py enable_voice:=false
```

> ⚠️ **bringup 默认 `enable_voice_motion:=true`，喊一句车就真的动。**
> 只想想验证串口/话题链路时，务必加 `enable_voice_motion:=false`。

## 不喊话也能测

```bash
# 前进
ros2 topic pub --once /voice_event std_msgs/String "{data: MOVE_FWD}"
# 停车
ros2 topic pub --once /voice_event std_msgs/String "{data: FOLLOW_OFF}"
# 看下发
ros2 topic echo /cmd_vel
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `enable_motion` | **false** | 安全开关。false=干跑只打日志，true=实车驱动 |
| `linear_speed` | 0.2 | 直行线速度 (m/s) |
| `angular_speed` | 1.0 | 原地转向角速度 (rad/s) |
| `action_duration` | 5.0 | 动作持续时间 (s) |
| `publish_rate` | 20.0 | `/cmd_vel` 频率 (Hz) |
| `pre_stop_duration` | 0.2 | 方向切换前停车过渡 (s)，0 = 关闭 |
| `stop_burst` | 3 | 动作结束补发零速度帧数 |
| `min_repeat_interval` | 0.4 | 同命令去抖间隔 (s)，STOP 不受限 |
| `allow_motion_during_nav` | false | 导航中是否允许语音运动指令 |
| `command_map` | 见 yaml | 命令词表 |

## 距离与角度换算

| 动作 | 公式 | 默认值结果 |
|---|---|---|
| 直行距离 | `linear_speed × action_duration` | `0.2 × 5.0 = 1.0 米` |
| 旋转角度 | `angular_speed × action_duration` | `1.0 × 5.0 = 5.0 rad = 286°` |

室内桌面演示建议缩短时长：`linear_speed: 0.15` + `action_duration: 3.0`
→ 直行 0.45 米、旋转 172°。

> 觉得转过头了请缩短 `action_duration`，不要只降 `angular_speed`——
> 角速度太低会让小车转得慢吞吞，体验差。

## 已知限制

- 车端**没有语音播报能力**：ASR-01 只有音频输入，没有输出。
  若毕设需要「小车语音应答」，需另加 I2S 功放（如 MAX98357A）并使用独立引脚。
- `FOLLOW_ON`（跟随）尚未实现：依赖的 `detection_ws` 工作空间本机不存在。
