# 通用 2D 栅格地图工件工具

该工具只生成展示工件，不修改导航使用的 `/map`，不做房间语义、标签、分区或轨迹推断。

## 静态地图一键导出

输入为 Nav2/ROS 标准的 `.yaml + .pgm/.png` 地图文件：

```bash
source /opt/ros/humble/setup.bash
source ~/chapt9_update/fishbot_ws/install/setup.bash
ros2 run autopatrol_robot map_artifact_export \
  --map-yaml /path/to/map.yaml \
  --output-dir ~/.ros/map_artifacts \
  --name cleaned_map
```

输出同名三份文件：

- `cleaned_map.png`：可直接用于网页与小程序的图片组件。
- `cleaned_map.svg`：自包含的可缩放展示资产。
- `cleaned_map.json`：Canvas 用的墙体多边形、尺寸、分辨率、原点和处理质量数据。

## 建图过程实时处理

```bash
source /opt/ros/humble/setup.bash
source ~/chapt9_update/fishbot_ws/install/setup.bash
ros2 launch autopatrol_robot map_artifact.launch.py \
  input_topic:=/map \
  output_dir:=~/.ros/map_artifacts \
  minimum_update_interval_s:=0.75
```

节点订阅 `/map`（可靠、瞬态本地 QoS），只处理内容发生变化的最新栅格，并以最大频率限制避免阻塞 SLAM。

- `/map_artifact/image`：`sensor_msgs/CompressedImage`，PNG。
- `/map_artifact/manifest`：`std_msgs/String`，JSON 工件描述。
- 默认将最新结果覆盖写入 `~/.ros/map_artifacts/live_map.{png,svg,json}`。

## 网页和小程序接口

现有 Flask 服务收到地图后也会生成相同工件：

- `GET /map_artifact`：JSON 工件描述，适合小程序 Canvas。
- `GET /map_artifact.png`：PNG。
- `GET /map_artifact.svg`：SVG。
- Socket.IO `map_data` 事件继续包含原有字段，并增加 `artifact` 字段。

JSON 的 `layers.walls.polygons` 坐标使用渲染像素坐标系，原点在左上；`map.origin` 与 `map.resolution` 用于转换回 ROS 地图坐标。

## 处理范围

1. 删除主体外的小孤岛、短线和重复平行边。
2. 补足满足距离和方向约束的墙体缺口。
3. 合并并规整为连续、统一线宽的墙体。
4. 保留内部障碍物，符合阈值的类圆障碍物规整为标准圆。
5. 生成 PNG、SVG、JSON 三种展示形式。

