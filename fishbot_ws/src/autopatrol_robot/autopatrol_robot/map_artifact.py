"""Portable artifacts for cleaned 2D occupancy-grid maps.

This module is display-only: it never changes the map consumed by navigation.
"""
from __future__ import annotations

import argparse
import base64
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np
import yaml

from autopatrol_robot.map_renderer import OccupancyMapRenderer


ARTIFACT_SCHEMA = "autopatrol.map-artifact/v1"


@dataclass(frozen=True)
class StaticOccupancyMap:
    """Occupancy data loaded from a ROS/Nav2 map YAML file."""

    data: np.ndarray
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "map"


@dataclass(frozen=True)
class MapArtifact:
    """One artifact shared by web, mini program and ROS clients."""

    png_bytes: bytes
    svg: str
    manifest: dict[str, Any]

    @property
    def png_base64(self) -> str:
        return base64.b64encode(self.png_bytes).decode("ascii")

    def client_payload(self) -> dict[str, Any]:
        """Return a Socket.IO and HTTP friendly payload."""
        payload = dict(self.manifest)
        payload["image"] = self.png_base64
        return payload


class OccupancyMapArtifactProcessor:
    """Build PNG, SVG and Canvas JSON from raw OccupancyGrid data."""

    def __init__(self, renderer: OccupancyMapRenderer | None = None):
        self.renderer = renderer or OccupancyMapRenderer()

    def process(
        self,
        data: Sequence[int] | np.ndarray,
        width: int,
        height: int,
        resolution: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        frame_id: str = "map",
    ) -> MapArtifact:
        """Create a portable display artifact without changing the input grid."""
        encoded, quality = self.renderer.render(data, width, height, resolution)
        png_bytes = base64.b64decode(encoded)
        image = cv2.imdecode(
            np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            raise RuntimeError("无法读取刚生成的地图 PNG")

        polygons = self._extract_wall_polygons(image)
        render_width = int(quality["output_width"])
        render_height = int(quality["output_height"])
        manifest = {
            "schema": ARTIFACT_SCHEMA,
            "map": {
                "frame_id": frame_id,
                "width": int(width),
                "height": int(height),
                "resolution": float(resolution),
                "origin": {"x": float(origin_x), "y": float(origin_y)},
                "image_y_axis": "down",
            },
            "render": {
                "width": render_width,
                "height": render_height,
                "format": "png",
                "scale": self.renderer.settings.render_scale,
                "background": "#0a0e1a",
                "floor": "#e8f2ec",
                "wall": "#4c4c31",
                "wall_halo": "#c1c7b3",
            },
            "layers": {
                "walls": {
                    "kind": "filled-polygons",
                    "coordinate_space": "render-pixels",
                    "polygons": polygons,
                }
            },
            "quality": quality,
        }
        return MapArtifact(
            png_bytes=png_bytes,
            svg=self._build_svg(encoded, render_width, render_height, polygons),
            manifest=manifest,
        )

    def export(
        self,
        artifact: MapArtifact,
        output_dir: str | Path,
        name: str = "map_artifact",
    ) -> dict[str, Path]:
        """Write PNG, SVG and Canvas JSON using one stable file prefix."""
        destination = Path(output_dir).expanduser()
        destination.mkdir(parents=True, exist_ok=True)
        safe_name = self._safe_name(name)
        paths = {
            "png": destination / f"{safe_name}.png",
            "svg": destination / f"{safe_name}.svg",
            "json": destination / f"{safe_name}.json",
        }
        paths["png"].write_bytes(artifact.png_bytes)
        paths["svg"].write_text(artifact.svg, encoding="utf-8")

        manifest = json.loads(json.dumps(artifact.manifest))
        manifest["assets"] = {key: path.name for key, path in paths.items()}
        paths["json"].write_text(
            json.dumps(manifest, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        return paths

    @staticmethod
    def _safe_name(name: str) -> str:
        normalized = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip())
        return normalized.strip("._") or "map_artifact"

    @staticmethod
    def _wall_mask(image: np.ndarray) -> np.ndarray:
        """Select dark wall strokes but exclude the unknown-space background."""
        blue, green, red = cv2.split(image)
        mask = (
            (blue >= 30)
            & (blue <= 105)
            & (green >= 52)
            & (green <= 125)
            & (red >= 30)
            & (red <= 125)
        ).astype(np.uint8) * 255
        return cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )

    def _extract_wall_polygons(self, image: np.ndarray) -> list[list[list[int]]]:
        """Extract simplified filled wall polygons for browser and mobile Canvas."""
        mask = self._wall_mask(image)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS
        )
        minimum_area = max(12.0, image.shape[0] * image.shape[1] * 0.000002)
        polygons: list[list[list[int]]] = []
        for contour in contours:
            if abs(cv2.contourArea(contour)) < minimum_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            approximation = cv2.approxPolyDP(
                contour, max(0.8, perimeter * 0.002), True
            )
            points = approximation.reshape(-1, 2)
            if len(points) >= 3:
                polygons.append([[int(x), int(y)] for x, y in points])
        return polygons

    @staticmethod
    def _build_svg(
        png_base64: str,
        width: int,
        height: int,
        wall_polygons: list[list[list[int]]],
    ) -> str:
        """Create a portable SVG; JSON carries native Canvas polygons."""
        paths = []
        for polygon in wall_polygons:
            start_x, start_y = polygon[0]
            commands = [f"M {start_x} {start_y}"]
            commands.extend(f"L {x} {y}" for x, y in polygon[1:])
            commands.append("Z")
            paths.append(
                '<path d="{}" fill="#4c4c31"/>'.format(" ".join(commands))
            )
        return (
            '<?xml version="1.0" encoding="UTF-8"?>'
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
            f'width="{width}" height="{height}">'
            f'<image href="data:image/png;base64,{png_base64}" '
            f'width="{width}" height="{height}"/>'
            f'<g id="wall-vectors" opacity="0">{"".join(paths)}</g>'
            "</svg>"
        )


def load_nav2_map(map_yaml: str | Path) -> StaticOccupancyMap:
    """Read a standard ROS/Nav2 map YAML plus image without map_server."""
    yaml_path = Path(map_yaml).expanduser().resolve()
    with yaml_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    image_value = config.get("image")
    if not image_value:
        raise ValueError(f"地图配置缺少 image: {yaml_path}")
    image_path = Path(str(image_value))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(f"无法读取地图图片: {image_path}")
    if image.ndim == 3:
        grayscale = cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2GRAY)
        alpha = image[:, :, 3] if image.shape[2] == 4 else None
    else:
        grayscale, alpha = image, None

    occupied_threshold = float(config.get("occupied_thresh", 0.65))
    free_threshold = float(config.get("free_thresh", 0.25))
    probability = grayscale.astype(np.float32) / 255.0
    if int(config.get("negate", 0)) == 0:
        probability = 1.0 - probability
    grid = np.full(grayscale.shape, -1, dtype=np.int8)
    grid[probability >= occupied_threshold] = 100
    grid[probability <= free_threshold] = 0
    if alpha is not None:
        grid[alpha < 255] = -1
    grid[grayscale == 205] = -1

    origin = config.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError(f"地图 origin 非法: {yaml_path}")
    return StaticOccupancyMap(
        data=grid.ravel(),
        width=int(grid.shape[1]),
        height=int(grid.shape[0]),
        resolution=float(config.get("resolution", 0.05)),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )


def export_nav2_map(
    map_yaml: str | Path,
    output_dir: str | Path,
    name: str | None = None,
) -> dict[str, Path]:
    """Process a saved map through the same core used by the live ROS node."""
    source = load_nav2_map(map_yaml)
    processor = OccupancyMapArtifactProcessor()
    artifact = processor.process(
        source.data,
        source.width,
        source.height,
        source.resolution,
        source.origin_x,
        source.origin_y,
        source.frame_id,
    )
    return processor.export(artifact, output_dir, name or Path(map_yaml).stem)


def main_export() -> None:
    """Run the map_artifact_export console command."""
    parser = argparse.ArgumentParser(
        description="将 ROS/Nav2 栅格地图导出为 PNG、SVG 和 JSON 展示工件"
    )
    parser.add_argument("--map-yaml", required=True, help="输入地图 YAML 文件")
    parser.add_argument(
        "--output-dir",
        default="~/.ros/map_artifacts",
        help="输出目录，默认 ~/.ros/map_artifacts",
    )
    parser.add_argument("--name", help="输出文件名称前缀")
    args = parser.parse_args()
    paths = export_nav2_map(args.map_yaml, args.output_dir, args.name)
    print(json.dumps({key: str(path) for key, path in paths.items()}, ensure_ascii=False))


if __name__ == "__main__":
    main_export()
