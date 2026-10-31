# 基线：2026-10-31 回起点成功

> **tag**: `baseline-20261031-return-home-ok`
> **判定依据**：`controller_server: Reached the goal!` → `bt_navigator: Goal succeeded`

这是目前实测效果最好的一次运行，后续任何改动如果变差，回滚到这里。

---

## 文件

| 文件 | 内容 |
|---|---|
| `nav2_amcl_params_20261031.txt` | 当时生效的 AMCL 参数快照 |
| `trace_20261031_2145_回起点成功.csv` | `amcl_tracer` 完整轨迹（含事件行） |
| `navigation2_20261031_回起点_摘录.log` | 关键日志摘录（已剔除刷屏） |

## 实测表现

```
21:37:02  载入地图 room1 (126x207)
21:37:29  导航栈启动
21:38:41  全局重撒开始
21:39:39  ✅ 定位收敛（用时 89.0s）位姿 (0.29,-0.26,-82.0°)
          σ_xy=0.074m σ_yaw=2.0°  位姿位于已知空闲区域（栅格值 0）
21:45:18  回起点 → Reached the goal! → Goal succeeded
```

## 当时生效的关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| `update_min_d` / `update_min_a` | `0.15` / `0.15` | 静止抖动不触发更新 |
| `resample_interval` | `2` | 重采样频率减半，防塌缩 |
| `recovery_alpha_slow` | `0.002` | 随机粒子不再成批接管 |
| `transform_tolerance` | **`0.2`** | ★ 偏小，见下方问题 ①② |
| `set_initial_pose` | `False` | 靠 auto_localizer 全局重撒 |

## 已知遗留问题（本次基线仍未解决）

1. **AMCL 每分钟丢一帧激光** — `Message Filter dropping message: frame
   'laser_link' ... timestamp is earlier than all the data in the
   transform cache`。雷达经 WiFi 传输有延迟。
2. **controller_server TF 外推失败 5 次** — `map→odom` 滞后 0.87s，
   `transform_tolerance: 0.2` 撑不住；其中一次导致 `Controller
   patience exceeded → follow_path Aborting`。
3. **σ 长期 DIVERGE** — 0.26~0.32m，从未回到收敛态。
   导航靠 action 层成功判定，实际到点精度可能偏差 30cm 以上。
4. **`nav_to_pose` 孤儿进程泄漏** — 实测存活 3 小时 10 分。
5. **快捷指令 UI 被浮层遮挡** — z-index 倒挂（45 < 47 < 50），
   且点击后零反馈。
6. **tracer 把正常旋转误报成 `JUMP[EKF]`** — `jump_ob` 不该判 yaw。

> 问题 1~3 同根：`stop_nomotion_after_converge=True` 让 AMCL 收敛后
> 只在激光到达时才发 TF，而激光实际有效率约 0.86Hz → 间隔必然 >1s。

## 回滚方式

```bash
git checkout baseline-20261031-return-home-ok -- src/
colcon build --packages-select fishbot_navigation2 --symlink-install
```
