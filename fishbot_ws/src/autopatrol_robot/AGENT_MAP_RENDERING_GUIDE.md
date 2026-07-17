# 智能体地图渲染工具使用指南

## 1. 目标与边界

本功能包将 ROS 2D 占据栅格地图转换为**仅供展示**的 PNG、SVG、Canvas JSON 工件。适用于网页端、小程序端和地图建图过程中的可视化。

必须遵守：

- 只处理展示副本，绝不修改导航、定位或 SLAM 使用的 `/map`、地图 YAML、PGM/PNG 原图。
- 不做房间语义、标签、分区、路线或清扫轨迹推断。
- 外轮廓与内部障碍物分层处理；不得让内部障碍物算法改变已确认的主体外轮廓。
- 对不确定的短线、孤点或形状，不要强行补成闭环；宁可剔除，也不要制造不存在的障碍物。

## 2. 先选处理模式

### 已保存的地图

输入为标准 Nav2 地图：一个 `.yaml` 和其引用的 `.pgm` 或 `.png`。

优先使用静态导出命令：

```bash
source /opt/ros/humble/setup.bash
source ~/chapt9_update/fishbot_ws/install/setup.bash
ros2 run autopatrol_robot map_artifact_export \
  --map-yaml /absolute/path/to/map.yaml \
  --output-dir /absolute/path/to/output \
  --name cleaned_map
```

只有确认 YAML、原图和输出目录后才执行。输出目录允许写入；输入地图目录默认只读。

### 建图过程中的 `/map`

使用实时节点，不要在回调中直接写入导航地图：

```bash
source /opt/ros/humble/setup.bash
source ~/chapt9_update/fishbot_ws/install/setup.bash
ros2 launch autopatrol_robot map_artifact.launch.py \
  input_topic:=/map \
  output_dir:=~/.ros/map_artifacts \
  minimum_update_interval_s:=0.75
```

实时节点只在地图内容变化时处理，并限制处理频率。它发布：

- `/map_artifact/image`：`sensor_msgs/CompressedImage`，PNG。
- `/map_artifact/manifest`：`std_msgs/String`，JSON 工件描述。

## 3. 渲染流水线

调用入口为 `OccupancyMapArtifactProcessor.process()`，不要绕开它自行拼接 PNG 或 JSON。

1. `OccupancyMapRenderer` 根据自由区域或墙体推断主体，生成平滑、连续、闭合的主体边线。
2. `InteriorObstacleRenderer` 只在主体内部安全区域运行：
   - 距离外轮廓过近的像素直接排除。
   - 删除小面积、短长度和孤立噪点。
   - 仅在局部小距离内补齐同一候选的断口。
   - 按严格条件重建标准圆、椭圆、矩形、线段或保守的简化多边形。
   - 对已确认闭合的圆、椭圆、矩形和多边形，以与边线相同的颜色填充；开放线段不填充。
3. `MapArtifact` 统一导出 PNG、SVG 和 JSON；三者必须来自同一次处理结果。

不要调用已保留的旧私有辅助函数（例如 `_draw_vector_walls`）作为新的业务入口；它们只用于历史兼容测试。

## 4. 结果解释与质量门槛

工件 JSON 的 `quality` 字段必须先检查再使用：

- `primary_contour_closed` 必须为 `true`，否则不应作为最终已建图展示结果发布。
- `boundary_mode` 为 `free-space` 或 `wall-inference`。后者表示输入 `/map` 没有可用的未知区域，使用了墙体推断，应记录该事实。
- `interior_*` 字段记录内部候选、剔除量和各形状类别数量；`interior_filled_shapes` 记录已填充的闭合障碍物数量。
- `removed_components`、`filled_gap_pixels`、`bridged_gaps` 用于排查是否出现过度清理或过度补线。

建议验收规则：

1. 主体外轮廓连续、闭合，且没有主体外孤立线段。
2. 内部圆形障碍显示为标准圆；矩形不得被误识别为圆；椭圆不得被降级为圆。
3. 外墙附近的障碍物候选不应单独绘制，避免与主体边线重复。
4. 若 `interior_obstacle_candidates` 异常偏高，先查看原始地图，不要盲目降低去噪阈值。
5. 浏览器 Canvas 与 PNG 背景必须都是 `#0a0e1a`，不得出现 PNG 矩形底色边界。

## 5. 网页与小程序接入

### Flask 网页端

`web_controller.py` 在加载已保存地图时，使用同一个 YAML 生成静态展示工件，并通过 Socket.IO 的 `map_data` 推送。

必须保留：

- `artifact.render.width`、`artifact.render.height`：Canvas 实际绘制尺寸。
- `artifact.render.scale`：世界坐标转渲染像素时的倍率。
- `map.origin`、`map.resolution`：ROS 坐标换算依据。
- `map_source == "static"` 时，不允许后续运行时 `/map` 覆盖静态展示结果。

可读取接口：

- `GET /map_artifact`：JSON 清单，适合小程序 Canvas。
- `GET /map_artifact.png`：PNG。
- `GET /map_artifact.svg`：自包含 SVG。

### Canvas 坐标换算

地图渲染像素的原点在左上角。将 ROS 世界坐标换为渲染像素时使用：

```text
pixel_x = (world_x - origin_x) / resolution * render.scale
pixel_y = (world_y - origin_y) / resolution * render.scale
```

不要使用原始栅格宽高替代 `artifact.render.width` 和 `artifact.render.height`；输出 PNG 按 `render.scale` 放大。

## 6. 修改算法时的工作顺序

1. 先保留一份当前有效的 PNG、SVG、JSON，作为对照产物。
2. 在 `test/test_map_renderer.py` 或 `test/test_interior_obstacle_renderer.py` 添加最小复现地图。
3. 只调整对应层：外轮廓问题改 `map_renderer.py`；内部障碍问题改 `interior_obstacle_renderer.py`。
4. 不要为修复内部障碍而降低主体轮廓的闭合、去孤岛或外墙裁剪规则。
5. 重新导出 `principal_boundary_preview` 中的样例，再做视觉验收。
6. 最后才重启网页服务；不要在未验证前覆盖运行中的展示结果。

## 7. 必须执行的验证

在功能包根目录执行：

```bash
source /opt/ros/humble/setup.bash
source ~/chapt9_update/fishbot_ws/install/setup.bash
cd ~/chapt9_update/fishbot_ws/src/autopatrol_robot

PYTHONPATH=. python3 -m unittest discover -s test -p 'test_map_*.py'
PYTHONPATH=. python3 -m unittest discover -s test -p 'test_interior_*.py'
python3 -m flake8 --ignore=E501,W503 \
  autopatrol_robot/interior_obstacle_renderer.py \
  autopatrol_robot/map_renderer.py \
  autopatrol_robot/map_artifact.py \
  test/test_interior_obstacle_renderer.py \
  test/test_map_renderer.py \
  test/test_map_artifact.py

cd ~/chapt9_update/fishbot_ws
colcon build --packages-select autopatrol_robot --symlink-install
```

全量 `unittest discover -s test -p 'test_*.py'` 若因系统未安装 `ament_copyright`、`ament_flake8` 或 `ament_pep257` 失败，应明确记录为环境依赖缺失，不得误判为地图功能失败。

## 8. 常见故障处理

| 现象 | 优先检查 | 正确处理 |
| --- | --- | --- |
| 网页显示空白矩形 | 静态 YAML 与运行时 `/map` 是否不一致 | 使用 `load_static_map_artifact()` 生成工件，并阻止静态结果被 `/map` 覆盖 |
| 地图有明显矩形底色 | PNG 与 Canvas 背景色 | 两端统一为 `#0a0e1a`，并检查实际解码 PNG 像素 |
| 外墙断开 | `primary_contour_closed` 与 `boundary_mode` | 调整主体轮廓闭合逻辑，不要用内部障碍算法补外墙 |
| 方形被画成圆 | `interior_circles` 与 `interior_rectangles` | 收紧圆形长宽比、圆度和圆盘比；保留矩形优先级 |
| 内部障碍太多 | `interior_obstacle_candidates` 与原图 | 优先提高最小面积、最短长度或边界安全距离 |
| 内部障碍丢失 | 原始障碍是否处于主体安全区 | 仅在确认是真实内部障碍后适度放宽局部候选阈值 |

## 9. 提交前检查

提交代码时仅包含功能包源码、测试、启动文件和文档。`fishbot_navigation2/maps/principal_boundary_preview` 与 `processed_preview` 是生成预览，除非任务明确要求更新样例资产，否则不要自动提交。
