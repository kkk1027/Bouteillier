"""Display-only cleanup and floor-plan rendering for static occupancy maps."""
from __future__ import annotations

import base64
import math
from dataclasses import asdict, dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class MapRenderSettings:
    """Metric thresholds keep rendering behaviour consistent across map resolutions."""

    occupied_threshold: int = 65
    min_island_area_m2: float = 0.015
    min_island_length_m: float = 0.22
    max_gap_m: float = 0.18
    contour_epsilon_m: float = 0.08
    wall_width_m: float = 0.07
    render_scale: int = 2


@dataclass
class MapRenderStats:
    removed_components: int = 0
    filled_gap_pixels: int = 0
    regularized_shapes: int = 0
    source_width: int = 0
    source_height: int = 0
    output_width: int = 0
    output_height: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


class OccupancyMapRenderer:
    """Converts an occupancy grid into a polished PNG without modifying navigation data."""

    def __init__(self, settings: MapRenderSettings | None = None):
        self.settings = settings or MapRenderSettings()

    def render(
        self,
        data: Sequence[int] | np.ndarray,
        width: int,
        height: int,
        resolution: float,
    ) -> tuple[str, dict]:
        grid = np.asarray(data, dtype=np.int8)
        if grid.size != width * height:
            raise ValueError(f"地图尺寸不匹配: {grid.size} != {width} x {height}")
        if resolution <= 0:
            raise ValueError(f"非法地图分辨率: {resolution}")

        grid = grid.reshape((height, width))
        known = (grid >= 0).astype(np.uint8) * 255
        free = (grid == 0).astype(np.uint8) * 255
        obstacles = (grid >= self.settings.occupied_threshold).astype(np.uint8) * 255

        cleaned, stats = self._clean_obstacles(obstacles, resolution)
        image = self._render_floorplan(known, free, cleaned, resolution, stats)
        success, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 6])
        if not success:
            raise RuntimeError("地图 PNG 编码失败")

        return base64.b64encode(encoded.tobytes()).decode("ascii"), stats.to_dict()

    def _clean_obstacles(
        self, obstacles: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, MapRenderStats]:
        filtered, removed = self._filter_islands(obstacles, resolution)
        radius = max(1, int(round(self.settings.max_gap_m / (2.0 * resolution))))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1))
        closed = cv2.morphologyEx(filtered, cv2.MORPH_CLOSE, kernel)
        closed, removed_after_close = self._filter_islands(closed, resolution)

        stats = MapRenderStats(
            removed_components=removed + removed_after_close,
            filled_gap_pixels=max(0, int(cv2.countNonZero(closed) - cv2.countNonZero(filtered))),
        )
        return closed, stats

    def _filter_islands(
        self, mask: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, int]:
        count, labels, components, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        min_area = max(4, int(round(self.settings.min_island_area_m2 / (resolution * resolution))))
        min_length = max(4, int(round(self.settings.min_island_length_m / resolution)))
        kept = np.zeros_like(mask)
        removed = 0

        for label in range(1, count):
            area = int(components[label, cv2.CC_STAT_AREA])
            component_width = int(components[label, cv2.CC_STAT_WIDTH])
            component_height = int(components[label, cv2.CC_STAT_HEIGHT])
            diagonal = math.hypot(component_width, component_height)
            if area >= min_area or diagonal >= min_length:
                kept[labels == label] = 255
            else:
                removed += 1

        return kept, removed

    def _render_floorplan(
        self,
        known: np.ndarray,
        free: np.ndarray,
        obstacles: np.ndarray,
        resolution: float,
        stats: MapRenderStats,
    ) -> np.ndarray:
        scale = self.settings.render_scale
        height, width = obstacles.shape
        output_size = (width * scale, height * scale)
        stats.source_width = width
        stats.source_height = height
        stats.output_width, stats.output_height = output_size

        known_large = cv2.resize(known, output_size, interpolation=cv2.INTER_NEAREST)
        free_large = cv2.resize(free, output_size, interpolation=cv2.INTER_NEAREST)
        canvas = np.full((height * scale, width * scale, 3), (45, 40, 31), dtype=np.uint8)
        canvas[known_large > 0] = (205, 220, 215)
        canvas[free_large > 0] = (232, 242, 236)

        wall_layer = self._draw_regularized_walls(obstacles, resolution, scale, stats)
        halo = cv2.dilate(wall_layer, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        canvas[halo > 0] = (179, 199, 193)
        canvas[wall_layer > 0] = (49, 76, 76)

        return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    def _draw_regularized_walls(
        self,
        obstacles: np.ndarray,
        resolution: float,
        scale: int,
        stats: MapRenderStats,
    ) -> np.ndarray:
        skeleton = self._skeletonize(obstacles)
        height, width = obstacles.shape
        output = np.zeros((height * scale, width * scale), dtype=np.uint8)
        wall_width = max(2, int(round(self.settings.wall_width_m / resolution * scale)))

        residual = cv2.resize(
            skeleton,
            (width * scale, height * scale),
            interpolation=cv2.INTER_NEAREST,
        )
        residual = cv2.dilate(
            residual,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (wall_width, wall_width)),
        )
        output = cv2.bitwise_or(output, residual)

        min_line = max(10, int(round(0.55 / resolution)))
        max_gap = max(2, int(round(self.settings.max_gap_m / resolution)))
        lines = cv2.HoughLinesP(
            skeleton,
            rho=1,
            theta=np.pi / 180,
            threshold=max(8, min_line // 2),
            minLineLength=min_line,
            maxLineGap=max_gap,
        )
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(
                    output,
                    (int(x1 * scale), int(y1 * scale)),
                    (int(x2 * scale), int(y2 * scale)),
                    255,
                    wall_width,
                    cv2.LINE_AA,
                )

        stats.regularized_shapes = self._overlay_standard_shapes(
            skeleton, output, resolution, scale, wall_width
        )
        return output

    def _overlay_standard_shapes(
        self,
        skeleton: np.ndarray,
        output: np.ndarray,
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> int:
        """Replace recognized closed outlines instead of drawing over them."""
        contours, _ = cv2.findContours(skeleton, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        epsilon = max(1.0, self.settings.contour_epsilon_m / resolution)
        regularized = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if area < 25 or perimeter < 20:
                continue
            approximation = cv2.approxPolyDP(contour, epsilon, True)
            if len(approximation) < 3:
                continue

            circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
            if circularity > 0.82:
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                if radius >= 5:
                    self._erase_contour_region(contour, skeleton, output, scale, wall_width)
                    cv2.circle(
                        output,
                        (int(cx * scale), int(cy * scale)),
                        int(radius * scale),
                        255,
                        wall_width,
                        cv2.LINE_AA,
                    )
                    regularized += 1
                continue

            rectangle = cv2.minAreaRect(contour)
            rectangle_area = rectangle[1][0] * rectangle[1][1]
            rectangularity = area / rectangle_area if rectangle_area > 1 else 0.0
            if len(approximation) <= 5 and rectangularity > 0.78:
                self._erase_contour_region(contour, skeleton, output, scale, wall_width)
                box = np.round(cv2.boxPoints(rectangle) * scale).astype(np.int32)
                cv2.polylines(output, [box], True, 255, wall_width, cv2.LINE_AA)
                regularized += 1

        return regularized

    @staticmethod
    def _erase_contour_region(
        contour: np.ndarray,
        skeleton: np.ndarray,
        output: np.ndarray,
        scale: int,
        wall_width: int,
    ) -> None:
        """Clear the source trace so a regularized outline is the only visible wall."""
        source_mask = np.zeros_like(skeleton)
        clear_width = max(3, int(math.ceil(wall_width / scale)) + 2)
        cv2.drawContours(source_mask, [contour], -1, 255, clear_width)
        source_mask = cv2.dilate(
            source_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        )
        clear_mask = cv2.resize(
            source_mask,
            (output.shape[1], output.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
        output[clear_mask > 0] = 0

    @staticmethod
    def _skeletonize(mask: np.ndarray) -> np.ndarray:
        if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
            return cv2.ximgproc.thinning(mask)

        source = mask.copy()
        skeleton = np.zeros_like(source)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while cv2.countNonZero(source):
            eroded = cv2.erode(source, element)
            opened = cv2.dilate(eroded, element)
            skeleton = cv2.bitwise_or(skeleton, cv2.subtract(source, opened))
            source = eroded
        return skeleton
