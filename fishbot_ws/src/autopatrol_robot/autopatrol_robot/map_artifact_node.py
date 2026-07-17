"""ROS2 node that turns /map into portable display artifacts."""
from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from autopatrol_robot.map_artifact import OccupancyMapArtifactProcessor


@dataclass(frozen=True)
class GridSnapshot:
    """A copy of a map message safe to render outside its subscription callback."""

    data: np.ndarray
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    stamp: object
    fingerprint: str


class MapArtifactNode(Node):
    """Publish a cleaned display map while keeping the navigation map untouched."""

    def __init__(self) -> None:
        super().__init__("map_artifact_processor")
        self.declare_parameter("input_topic", "/map")
        self.declare_parameter("image_topic", "/map_artifact/image")
        self.declare_parameter("manifest_topic", "/map_artifact/manifest")
        self.declare_parameter("output_dir", "~/.ros/map_artifacts")
        self.declare_parameter("artifact_name", "live_map")
        self.declare_parameter("export_files", True)
        self.declare_parameter("minimum_update_interval_s", 0.75)

        self.processor = OccupancyMapArtifactProcessor()
        self._pending: GridSnapshot | None = None
        self._last_fingerprint: str | None = None
        self._last_processed_at = 0.0

        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_topic = str(self.get_parameter("input_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        manifest_topic = str(self.get_parameter("manifest_topic").value)
        self.create_subscription(
            OccupancyGrid, input_topic, self._map_callback, input_qos
        )
        self.image_publisher = self.create_publisher(
            CompressedImage, image_topic, output_qos
        )
        self.manifest_publisher = self.create_publisher(
            String, manifest_topic, output_qos
        )
        self.create_timer(0.1, self._process_pending)
        self.get_logger().info(
            f"地图工件节点就绪: {input_topic} -> {image_topic}, {manifest_topic}"
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Remember only the latest map; rendering is rate-limited by the timer."""
        raw_data = np.asarray(msg.data, dtype=np.int8).copy()
        signature = (
            f"{msg.info.width}:{msg.info.height}:{msg.info.resolution:.9f}".encode(
                "ascii"
            )
        )
        fingerprint = hashlib.blake2b(
            signature + raw_data.tobytes(), digest_size=16
        ).hexdigest()
        if fingerprint == self._last_fingerprint:
            return
        self._pending = GridSnapshot(
            data=raw_data,
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            frame_id=msg.header.frame_id or "map",
            stamp=msg.header.stamp,
            fingerprint=fingerprint,
        )

    def _process_pending(self) -> None:
        """Render at a bounded rate so a live SLAM map cannot starve ROS callbacks."""
        snapshot = self._pending
        if snapshot is None:
            return
        minimum_interval = float(
            self.get_parameter("minimum_update_interval_s").value
        )
        if time.monotonic() - self._last_processed_at < minimum_interval:
            return

        self._pending = None
        try:
            artifact = self.processor.process(
                snapshot.data,
                snapshot.width,
                snapshot.height,
                snapshot.resolution,
                snapshot.origin_x,
                snapshot.origin_y,
                snapshot.frame_id,
            )
            self._publish(snapshot, artifact)
            self._last_fingerprint = snapshot.fingerprint
            self._last_processed_at = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"地图工件生成失败: {exc}")

    def _publish(self, snapshot: GridSnapshot, artifact) -> None:
        image = CompressedImage()
        image.header.frame_id = snapshot.frame_id
        image.header.stamp = snapshot.stamp
        image.format = "png"
        image.data = artifact.png_bytes
        self.image_publisher.publish(image)

        manifest = dict(artifact.manifest)
        if bool(self.get_parameter("export_files").value):
            output_dir = Path(
                str(self.get_parameter("output_dir").value)
            ).expanduser()
            artifact_name = str(self.get_parameter("artifact_name").value)
            paths = self.processor.export(artifact, output_dir, artifact_name)
            manifest["assets"] = {
                key: str(path) for key, path in paths.items()
            }
        message = String()
        message.data = json.dumps(manifest, ensure_ascii=False, separators=(",", ":"))
        self.manifest_publisher.publish(message)
        self.get_logger().info(
            "地图工件已发布: "
            f"{manifest['render']['width']}x{manifest['render']['height']}, "
            f"墙体多边形 {len(manifest['layers']['walls']['polygons'])}"
        )


def main(args=None) -> None:
    """Run the live OccupancyGrid processor."""
    rclpy.init(args=args)
    node = MapArtifactNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
