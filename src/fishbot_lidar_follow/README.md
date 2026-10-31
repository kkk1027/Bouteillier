# fishbot_lidar_follow

单线激光雷达目标跟随。让小车跟着前方最近的障碍物走，保持约 50cm 距离。

独立 ROS 2 功能包，不依赖 `autopatrol_robot`（那个包只是通过 AI 工具调用它）。

---

## 一、包内容

```
fishbot_lidar_follow/
├── fishbot_lidar_follow/
│   ├── lidar_follow.py        核心节点：装配 + 状态机 + 控制输出
│   ├── geometry.py            点云提取/聚类/选择（纯函数，无 ROS 依赖）
│   ├── tracker.py             卡尔曼滤波与目标跟踪器
│   ├── mutual_exclusion.py    与导航互斥的锁文件
│   └── sim_node.py            仿真雷达（离线验证用）
├── config/lidar_follow.yaml   全部参数
├── launch/
│   ├── lidar_follow.launch.py      启动跟随节点
│   └── lidar_follow_sim.launch.py  一键启动仿真（不需要雷达）
└── test/
    ├── test_lidar_follow_logic.py  纯逻辑测试 42 项
    └── test_lidar_follow_sim.py    离线仿真测试 26 项
```

**模块划分原则**：`geometry` 和 `tracker` 不 import rclpy，
所以能脱离 ROS 单测，改参数时不用起整个节点。

---

## 二、快速开始

### 方式 A：离线验证（不需要雷达、不需要底盘）

```bash
colcon build --packages-select fishbot_lidar_follow --symlink-install
source install/setup.bash

# 一键跑：假雷达 + 跟随节点
ros2 launch fishbot_lidar_follow lidar_follow_sim.launch.py

# 换个场景
ros2 launch fishbot_lidar_follow lidar_follow_sim.launch.py scenario:=sideways
```

仿真节点默认**只打印速度，不发布到真实 /cmd_vel**，所以车不会动。

| 场景 | 演示什么 |
|---|---|
| `walk_away` | 目标后退 → 前进追赶 |
| `walk_close` | 目标逼近 → 后退 |
| `sideways` | 目标横移 → 跟着转 |
| `dropout` | 目标消失 → 停车，再出现继续跟 |
| `static` | 目标静止在死区 → 完全不动 |
| `legs_walk` | **两条腿各 1 个点**，先退后进 → 双腿配对锁定 |
| `legs_side` | 两条腿整体左右转 ±25° → 跟转 |
| `legs_occlude` | 每 4 帧有 1 帧只剩一条腿 → 遮挡容忍 |
| `legs_single` | 双腿锁定后只剩一条腿 → 单腿兜底 |
| `noise` | 真实杂散点（固定虚假回波 + 偶发飞点）→ **不应该锁定** |
| `noise_random` | 压力测试：每帧全扇区重随机散点 |

所有场景开头都有 4.5 秒的「锁定窗口」。
因为首次锁定要连续确认 5 帧（1 秒），再加上节点启动时间和余量。
这个窗口之后才开始各场景自己的动作。

> `legs_*` 场景里每条腿只占**1 个点**，这正是实机上人腿的样子。
> 单簇逻辑的 `min_cluster_points=5` 会把它们全部丢掉，
> 所以必须靠双腿配对才能锁上 —— 这就是这些场景要验证的东西。

### 方式 B：真实雷达

```bash
# 1. 确认雷达在跑
ros2 topic hz /scan          # 应该约 5Hz

# 2. 启动跟随
ros2 launch fishbot_lidar_follow lidar_follow.launch.py

# 3. 看状态
ros2 topic echo /lidar_follow/state

# 4. 停止（Ctrl+C，节点会先发零速度再退出）
```

### 方式 C：语音 / AI 对话

`autopatrol_robot` 里有对应的 AI 工具：

- 「开始雷达跟随」/「跟着我走」/「用雷达跟着我」
- 「停止雷达跟随」/「别跟着我了」
- 「雷达跟随开着吗」

---

## 三、行为规则

### 两段式锁定

```
[首次锁定]  目标必须出现在 0.50m 内   ← 不会把 1 米外的墙当目标
     ↓
[持续跟踪]  锁定后放宽到 1.20m        ← 目标后退也继续跟
     ↓
d > 0.65  → 前进追赶
d < 0.50  → 后退
0.55~0.65 → 停着不动（死区）
d > 1.20 或出 ±30° → 丢失，立即停车
```

> 为什么要两段式：用户要求「雷达前方 40cm 内的障碍物才是目标」，
> 但同时要求「超过 65cm 开始追赶」。如果只在 40cm 内认目标，
> 就永远追不到 65cm 外的。所以锁定门槛紧、跟踪范围宽。
>
> 为什么门槛是 0.50 而不是 0.40：见下文「十二、实机验证记录」。
> 雷达装在车体正中心（`laser_link` 相对 `base_link` 为 `0 0 0.075`，
> x/y 偏移都是 0），用户量的「车头前方 40cm」换算到雷达坐标系
> 要加上车体半径 0.10m，也就是 0.50m。

### 三态滞回

```
  0.30   0.50      0.55        0.65        1.20
   │  RETREAT │  HOLD  │  APPROACH  │
   │          │        │            │
   │    后退到│0.55 停 │0.55 前进到停│
   │          │        │            │
   └ 进入后退: d<0.50   └ 进入前进: d>0.65
```

进入和退出用不同阈值，形成 10cm 滞回带。
如果都用 0.50，目标在 0.50 附近抖动时会
`RETREAT → HOLD → RETREAT → HOLD` 疯狂翻转，电机来回抽。

---

## 四、参数

配置文件 `config/lidar_follow.yaml`，也可以启动时覆盖：

```bash
ros2 launch fishbot_lidar_follow lidar_follow.launch.py \
    params_file:=/path/to/custom.yaml
```

### 识别与跟踪

| 参数 | 默认 | 说明 |
|---|---|---|
| `lock_max_range` | 0.50 | 首次锁定门槛（车头前方 40cm + 车体半径） |
| `track_max_range` | 1.20 | 锁定后跟踪上限 |
| `sector_half_angle_deg` | 30 | 前方扇区半角 |
| `cluster_angle_gap_deg` | 2.5 | 聚类角度阈值 |
| `cluster_range_gap` | 0.15 | 聚类距离阈值 |
| `min_cluster_points` | 5 | 点数少于此值的**单簇**直接忽略（防噪声） |
| `max_target_jump` | 0.30 | 目标距离跳变超过此值判为换物体 |
| `max_angle_jump_deg` | 25 | 目标角度跳变超过此值判为换物体 |
| `lock_sector_half_angle_deg` | 75 | 未锁定时的搜索扇区半角（总计 150°） |
| `lock_confirm_frames` | 5 | 首次锁定要连续确认的帧数 |
| `lock_confirm_tol_dist` | 0.10 | 确认期间中点距离容差 |
| `lock_confirm_tol_angle_deg` | 6 | 确认期间中点角度容差 |
| `lock_confirm_tol_sep` | 0.20 | 确认期间两腿间距容差 |
| `lock_settle_time` | 1.0 | 锁定后只观察不动作的时长（秒） |
| `max_target_speed` | 1.50 | 卡尔曼估计的径向速度超过此值就停车 |

#### 双腿配对

| 参数 | 默认 | 说明 |
|---|---|---|
| `pair_enable` | true | 双腿成对锁定总开关 |
| `pair_lock_max_range` | 1.00 | 两点各自的距离上限（锁定前） |
| `pair_track_max_range` | 1.20 | 两点各自的距离上限（锁定后） |
| `pair_max_sep` | 0.60 | 两点直线距离上限 |
| `pair_min_sep` | 0.05 | 两点直线距离下限 |
| `pair_max_dist_diff` | 0.22 | 两点深度差上限 |
| `pair_max_lateral_sep` | 0.50 | 两点横向间距上限（腿距） |
| `pair_max_along_sep` | 0.25 | 两点纵向错位上限 |
| `pair_min_points` | 2 | 两点点数之和下限 |
| `pair_max_span_deg` | 45 | 单个簇的角度宽度上限 |
| `leg_fallback_enable` | true | 单腿兜底开关（仅已锁定时生效） |
| `leg_max_points` | 4 | 兜底只认小簇 |
| `leg_max_dist_diff` | 0.35 | 兜底距离与预测的容差 |

> `min_cluster_points` 和 `max_target_jump` 是**实机验证后新增**的，
> 见下面「十二、实机验证记录」。两个都不能关。

### 死区

| 参数 | 默认 | 说明 |
|---|---|---|
| `approach_enter` | 0.65 | 超过则前进 |
| `approach_exit` | 0.55 | 前进到此停止 |
| `retreat_enter` | 0.50 | 小于则后退 |
| `retreat_exit` | 0.55 | 后退到此停止 |
| `desired_distance` | 0.575 | 距离环目标值 |
| `distance_deadband` | 0.02 | 距离误差死区 |
| `angle_deadband_deg` | 2.0 | 角度误差死区 |
| `align_before_move_deg` | 15.0 | 超过此角度先转向再前进 |

### 速度

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_linear` | 0.22 | 前进上限 |
| `max_reverse` | 0.12 | 后退上限（更慢，安全） |
| `max_angular` | 0.80 | 转向上限 |
| `kp_linear` / `kd_linear` | 1.10 / 0.18 | 距离环 PD |
| `kp_angular` / `kd_angular` | 1.60 / 0.12 | 角度环 PD |
| `linear_accel_limit` | 0.80 | 线加速度限幅 |
| `angular_accel_limit` | 2.50 | 角加速度限幅 |

### 安全

| 参数 | 默认 | 说明 |
|---|---|---|
| `rear_check_range` | 0.40 | 后方检查距离 |
| `rear_sector_half_angle_deg` | 40 | 后方扇区半角 |
| `front_guard_range` | 0.32 | 前方急停距离 |
| `scan_timeout` | 0.60 | 雷达超时判定 |
| `lost_frame_threshold` | 3 | 连续丢帧判丢失 |

---

## 五、安全机制

| 机制 | 行为 |
|---|---|
| 雷达超时 0.6s | 停车 |
| 目标丢失 | 立即停车等待（不原地转找） |
| 后方 40cm 有障碍 | 拒绝后退 |
| 前方 32cm 有突现障碍 | 拒绝前进 |
| 加速度限幅 | 起步不突跳 |
| **SIGINT/SIGTERM/SIGHUP** | 先发 3 次零速度，再删锁文件 |
| ESP32 保护 | 3 秒无 Twist 自动停车（最后防线） |
| 与导航互斥 | 写锁文件，导航拒绝启动 |
| 与视觉看护互斥 | 互相检测并停止对方 |

### 关于信号处理（踩过的坑）

只用 `try/finally + KeyboardInterrupt` **是不够的**：

- `timeout` 发的是 `SIGTERM`
- `ros2 launch` 退出时转发的是 `SIGINT`

两者都不会让 `rclpy.spin()` 抛 `KeyboardInterrupt`，
所以 `finally` 里的「发零速度 + 删锁文件」根本不执行，
会留下**孤儿进程**和**残留锁**（残留锁会让导航永远无法启动）。

现在显式注册了 `SIGINT / SIGTERM / SIGHUP` 的 handler，
收到信号先安全停车再退出。验证方法：

```bash
python3 src/fishbot_lidar_follow/fishbot_lidar_follow/lidar_follow.py &
PID=$!
sleep 4
kill -TERM $PID
sleep 2
tail -3 /tmp/xxx.log          # 应该看到「已停车并移除跟随锁」
ls /tmp/fishbot_lidar_follow.lock   # 应该不存在
```

> 注意不要用 `ros2 run` 测这个 —— `ros2 run` 会多套一层进程，
> `kill` 打到的是包装层，节点进程收不到信号。

---

## 六、与导航互斥

本项目**没有 `twist_mux`**，所有节点直接发 `/cmd_vel`。
跟随、导航、看护、遥控都能发，两个一起跑会互相覆盖导致车原地抽搐。

约定：
- 跟随启动时写 `/tmp/fishbot_lidar_follow.lock`，退出时删
- `autopatrol_robot/nav_manager.py` 的 4 个导航入口检查该锁，
  存在就拒绝并提示先关跟随
- `cancel_navigation` **故意不检查** —— 取消操作应该永远可用
- 锁在但进程没了 → 判定为陈旧锁，自动清理（防止 `kill -9` 后导航永久失效）

---

## 七、测试

```bash
source install/setup.bash

# 纯逻辑：状态机、卡尔曼、聚类、安全检查、扇区过滤
python3 src/fishbot_lidar_follow/test/test_lidar_follow_logic.py    # 42 项

# 离线仿真：mock 掉 ROS 节点，喂假雷达数据查 cmd_vel
python3 src/fishbot_lidar_follow/test/test_lidar_follow_sim.py      # 26 项
```

两套都**不创建 ROS 节点、不发布真实消息、不杀任何进程**，
可以放心在没有小车时跑。

---

## 八、已知限制

### 1. 无法区分人和椅子

纯雷达只看「最近的东西」。人旁边有把椅子且椅子更近，就会锁到椅子上。
**这是物理限制，不是 bug。** 要解决需要视觉/雷达融合。

### 2. 雷达量程下限 0.3m

`ydlidar.yaml` 里 `range_min: 0.3`。
目标贴到 30cm 以内时雷达**直接丢点**（不是报 0.3，是整个点消失）。
正常情况不会触发（车会退到 55cm），但人突然冲向车侧可能瞬间丢失目标。

### 3. 只有 5Hz

`frequency: 5.0`。目标快速横移时 200ms 内可能出扇区，表现为「跟丢了」。
卡尔曼能缓解但无法根治。如嫌慢可把 `ydlidar.yaml` 的 `frequency`
调到 8~10Hz（需确认硬件支持）。

### 4. 单线雷达看不到的东西

- **透明物体**（玻璃门、玻璃桌）—— 激光穿透，等于不存在
- **低于雷达高度**（`laser_link` 在 `base_link` 上方 0.075m）
- **反光/黑色吸光物体** —— 可能测距失败
- **悬空物体** —— 不在激光平面内

### 5. 目标必须是激光能扫到的东西

人的**腿**可以，但穿长裙、宽松裤子时点云会变稀疏。
建议联调时让人站在车正前方 40~80cm 处先确认能锁定。

### 6. 只看到一条腿时，角度不可靠

人走路时两条腿互相遮挡，经常只有一条腿可见。
这时走「单腿兜底」：**只用距离，角度交给卡尔曼预测**。

一条腿偏离身体中心约 11cm，在 0.6m 处相当于 ±10°。
如果直接用它反角度，车会左右抽 —— 这比「角度暂时不准」更糟。
所以遮挡期间车的朝向不会主动调整，等两条腿重新出现再修。
遮挡超过 3 帧（0.6 秒）仍然会判丢失停车，不会盲跟。

### 7. 首次锁定必须成对，单条腿锁不上

未锁定时**不允许**靠一条腿建立锁定。
因为一条腿无法区分「人」和「一根桌腿 / 一个垃圾桶腿」，
锁错了就会直接朝它开过去。

这是故意设计成不可调的：`leg_fallback` 只在已锁定时生效。
想让人在车前被认出来，必须两条腿都露出来（正常站姿就行）。

### 8. 极端随机散点仍有理论误锁可能

`noise_random` 场景（每帧在整个扇区重新随机撒点）在短时间
（约 25 帧 / 5 秒）内误锁率统计为 0%，但跑更久仍可能偶尔命中。
几何判据对这种噪声的单帧命中率接近 100%，
完全靠 5 帧时间确认 + 1 秒稳定观察期兜底。

**真实雷达不是这种噪声**：真实杂散点是墙角、玻璃、镜面多重反射的
固定虚假回波（`noise` 场景模拟的就是这个），位置稳定、不会每帧乱跳，
靠几何判据就能全部拦住。

即便真的误锁，1 秒观察期内噪声会自己跳走，
触发丢失保护 → 停车，车不会动起来。

---

## 九、联调步骤（连上小车后）

### 第 1 步：确认雷达数据

```bash
ros2 topic hz /scan                 # 约 5Hz
ros2 topic echo /scan --once | head -20
```

本机配置：`angle_min: -180°`, `range_min: 0.3`, `range_max: 64.0`。

### 第 2 步：空跑（车不动）

把 `max_linear` 和 `max_reverse` 临时改成 `0.0`，启动节点，然后：

```bash
ros2 topic echo /lidar_follow/state
```

人走近/走远，看状态机是否正确切换 APPROACH / HOLD / RETREAT。
这一步车不会动，最安全。

### 第 3 步：限速试跑

`max_linear: 0.08`、`max_reverse: 0.05`，找块空地，
人站在车前 70cm 处慢慢后退，看车是否平稳跟随。

**重点观察**：

- 车会不会冲过头 → 调大 `approach_exit`
- 停下来有没有抖动 → 调大 `distance_deadband`
- 转向是否过冲 → 调小 `kp_angular`、调大 `kd_angular`

### 第 4 步：逐步放开

确认稳定后调回 `0.22 / 0.12`。

### 调试

```bash
# [距离, 角度, 距离变化率, 角度变化率, 簇数量, 状态码]
# 状态码 0=IDLE 1=SEARCH 2=APPROACH 3=HOLD 4=RETREAT
ros2 topic echo /lidar_follow/debug

# 详细日志
ros2 launch fishbot_lidar_follow lidar_follow.launch.py log_level:=debug
```

---

## 十、为什么用卡尔曼

雷达只有 **5Hz**（一帧 200ms）。人走路 0.5m/s，
这 200ms 里就移动了 10cm —— 直接拿原始测量做控制必然滞后。

对 **(距离 d, 方位角 θ)** 各用一个匀速(CV)模型卡尔曼，状态 `[位置, 速度]`：

- 平滑测量噪声（实测：3cm 随机噪声 → 滤波后波动 1.5cm）
- 估计变化率，用于 PD 的 D 项阻尼（实测：0.2m/s 匀速，速度估计误差 < 0.001）
- 短暂丢帧时用预测值撑住（最多 3 帧 = 600ms）

**为什么跟踪 (d, θ) 而不是 (x, y)**

故意不补偿本车运动。d 和 θ 正是要控制的量：
本车前进导致 d 减小，本来就该反映进控制环。
若换算到世界坐标再折算回来，反而多绕一圈、还引入 TF 延迟。
代价是速度估计里混入本车速度分量，
但这恰好让 D 项起阻尼作用，是有益的。

---

## 十一、参考的开源方案

调研了 `wheeltec_ros2` 里的实现，结论：**都不能直接用**。

| 文件 | 内容 | 问题 |
|---|---|---|
| `simple_follower_ros2/radarfollower.py` | 裸 PID | 依赖 `turn_on_wheeltec_robot.msg`（本项目没有）、无状态机、无后退 |
| `simple_follower_ros2/radarTracker.py` | 毫米波雷达 `RadarScan` | 不是激光雷达、无卡尔曼 |
| `simple_follower_ros2/laserfollower.py` | 裸 PID | 只有 `distance > 0.3` 简单判断 |
| `wheeltec_bodyreader/.../follower.cpp` | 低通 + PD + 误差死区 + 输出死区 + 丢帧保护 | 视觉方案，但**这几条工程经验值得借鉴** |

本实现借鉴了 `follower.cpp` 的：误差死区、输出死区、
丢帧时不更新 `last_error`（避免 D 项突变）、输出限幅 + 加速度限幅。

状态机、卡尔曼、互斥、安全检查是本项目新写的。

---

## 十二、实机验证记录（2026-10-31）

第一次接真雷达测试，锁定功能正常，但暴露出三个问题。
全部已修复，并固化在 `test/test_real_data_regression.py`（17 项）。

### 验证用的只读工具

```bash
# 一次性分析：能不能锁定、物体在哪
python3 src/fishbot_lidar_follow/test/probe_scan_readonly.py

# 全景：360° 点云分布，找出物体实际方位
python3 src/fishbot_lidar_follow/test/probe_scan_panorama.py

# 实时监视：移动物体时实时看能不能锁定（30秒）
python3 src/fishbot_lidar_follow/test/probe_scan_live.py 30
```

三个脚本都**只订阅 /scan，不发布任何话题，不启动跟随节点**，
小车完全不会动。它们调用包里真实的几何函数，
所以判断结果和正式运行一致。

### 问题 1：目标推进 30cm 内，雷达整个丢点

```
16:21:55  0.30m +13.0° 36点   ← 刚好在 range_min 边界
16:21:57  💤 【丢失】前方扇区内无目标
```

`range_min: 0.3` 是硬件限制，软件无法修复。
这也是为什么死区下限定在 0.50m —— 不能指望雷达看到 30cm 内的东西。

### 问题 2：切换到 2 个点的噪声簇（危险）

```
16:21:57  ✅ 锁定 0.89m -21.6° 2点 | APPROACH 前进
```

近处目标消失后，逻辑选中了一个「0.87m / -20°，只有 2 个点」的簇，
判定 APPROACH —— 如果车在跑，它会朝那个方向冲过去。
那 2 个点其实是柜子边缘的反光。

**修复**：`min_cluster_points: 5`，点数不足的簇直接忽略。

### 问题 3：目标跳变被静默接受

同一个瞬间，当前目标是 0.33m，候选是 0.87m，差 0.54m。
旧逻辑的关联阈值是 0.60m，于是「平滑地切换」到了另一个物体，
用户完全无感。

**修复**：`max_target_jump: 0.30`，跳变超过阈值就当作本帧没目标，
交给丢帧逻辑（先停车，3 帧后重锁最近的）。

> 0.30m/帧 在 5Hz 下相当于 1.5m/s，比跑步还快，
> 正常人的移动不会被误拦。

### 实测数据（修复后）

物体稳定停在 0.44m，24~26 个点，单簇，无跳动：

```
（门槛 0.40 时）
16:26:41 ⬜ 未锁定  前方 26 点 1 簇 | 后方空 ← 最近 0.44m 超过 0.40m 锁定门槛
16:26:47 ⬜ 未锁定  前方 26 点 1 簇 | 后方空 ← 最近 0.44m 超过 0.40m 锁定门槛
```

检测本身非常稳定（0.44m 一直保持 24~28 点、单簇、无跳动），
只是 0.44m 比 0.40m 门槛多 4cm —— 这是门槛标定的问题，不是识别的
问题。把物体往前挪到 40cm 内立即可锁定（16:21:40 的记录已验证：
0.39m / 28 点，瞬间锁定）。

### 0.40 → 0.50：门槛标定修正

**根因**：雷达装在车体正中心。`laser_link` 相对 `base_link` 的
关节原点为 `xyz="0 0 0.075"` —— x/y 都没有偏移，也就是雷达光心
和车体旋转中心重合。用户量的「前方 40cm」是从**车头边缘**算的，
而雷达读数是从**车体中心**算的，两者相差一个车体半径：

```
车体半径 0.10m
     ↓
[车尾]──────[中心/雷达]──────[车头]
                              └─ 用户从这里量 40cm
              └─ 雷达从这里量，所以读数是 40 + 10 = 50cm
```

**修正**：`lock_max_range: 0.40 → 0.50`。

**边界约束**（这个值不能随便往上调）：

| 关系 | 后果 |
|---|---|
| `lock_max_range ≤ retreat_enter (0.50)` | ✅ 锁定后不会立刻后退 |
| `lock_max_range > retreat_enter` | ❌ 一锁定就满足「太近了」，车立刻后退 |

所以 0.50 刚好是上限 —— 和 `retreat_enter` 相等，锁定瞬间判定为
`HOLD`（因为后退条件是严格小于 `d < 0.50`）。再往上调就会和
后退逻辑冲突，节点启动时会打警告。

**验证**（只读探针，未发任何指令）：

```
16:32:49 ✅ 锁定 0.38m +0.9° 30点 | RETREAT 后退 | 后方空 | 前方 近0.38m
16:32:53 ✅ 锁定 0.38m +0.4° 28点 | RETREAT 后退 | 后方空 | 前方 近0.38m
16:32:58 ✅ 锁定 0.38m -0.4° 26点 | RETREAT 后退 | 后方空 | 前方 近0.38m
```

物体被挪到 0.38m，稳定锁定，状态为 `RETREAT` —— 因为 0.38 < 0.50
确实太近了，后退是对的。这条记录同时验证了「锁定」和「后退」两个
分支。

### 如果觉得 50cm 太严格 / 太宽松

改 `config/lidar_follow.yaml`：

```yaml
lock_max_range: 0.45    # 收紧（但 0.44m 的物体又会锁不上）
lock_max_range: 0.58    # 放宽（警告：超过 retreat_enter，锁定即后退）
```

权衡：放得越宽，越容易把 1 米外的墙、家具当成目标；收得越紧，
越需要把物体摆到正前方贴近处。0.50 是「车头外 40cm」这个原始需求
在雷达坐标系下的等效值，且刚好不触发后退。
