"""Aggressive display-only vector rendering for static occupancy maps."""
from __future__ import annotations

import base64
import math
from collections import defaultdict
from dataclasses import asdict, dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class MapRenderSettings:
    """Metric thresholds for aggressive, display-only floor-plan cleanup."""

    occupied_threshold: int = 65
    min_island_area_m2: float = 0.025
    min_island_length_m: float = 0.35
    max_gap_m: float = 0.32
    min_line_length_m: float = 0.38
    line_merge_gap_m: float = 0.42
    line_offset_m: float = 0.10
    contour_epsilon_m: float = 0.12
    wall_width_m: float = 0.08
    endpoint_direction_probe_px: int = 5
    render_scale: int = 2


@dataclass
class MapRenderStats:
    removed_components: int = 0
    filled_gap_pixels: int = 0
    bridged_gaps: int = 0
    vector_segments: int = 0
    regularized_shapes: int = 0
    source_width: int = 0
    source_height: int = 0
    output_width: int = 0
    output_height: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


class OccupancyMapRenderer:
    """Creates a clean viewing layer while navigation keeps the raw map data."""

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
        free = (
            (grid >= 0) & (grid < self.settings.occupied_threshold)
        ).astype(np.uint8) * 255
        obstacles = (
            grid >= self.settings.occupied_threshold
        ).astype(np.uint8) * 255

        cleaned, stats = self._clean_obstacles(obstacles, resolution)
        image = self._render_floorplan(known, free, cleaned, resolution, stats)
        success, encoded = cv2.imencode(
            ".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 6]
        )
        if not success:
            raise RuntimeError("地图 PNG 编码失败")

        return base64.b64encode(encoded.tobytes()).decode("ascii"), stats.to_dict()

    def _clean_obstacles(
        self, obstacles: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, MapRenderStats]:
        filtered, removed = self._filter_islands(obstacles, resolution)
        radius = max(
            1, int(round(self.settings.max_gap_m / (2.0 * resolution)))
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1)
        )
        closed = cv2.morphologyEx(filtered, cv2.MORPH_CLOSE, kernel)
        closed, removed_after_close = self._filter_islands(closed, resolution)

        stats = MapRenderStats(
            removed_components=removed + removed_after_close,
            filled_gap_pixels=max(
                0, int(cv2.countNonZero(closed) - cv2.countNonZero(filtered))
            ),
        )
        return closed, stats

    def _filter_islands(
        self, mask: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, int]:
        count, labels, components, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        min_area = max(
            4, int(round(self.settings.min_island_area_m2 / (resolution * resolution)))
        )
        min_length = max(
            4, int(round(self.settings.min_island_length_m / resolution))
        )
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

        wall_layer = self._draw_vector_walls(obstacles, resolution, scale, stats)
        halo = cv2.dilate(
            wall_layer, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        )
        canvas[halo > 0] = (179, 199, 193)
        canvas[wall_layer > 0] = (49, 76, 76)

        return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    def _draw_vector_walls(
        self,
        obstacles: np.ndarray,
        resolution: float,
        scale: int,
        stats: MapRenderStats,
    ) -> np.ndarray:
        skeleton = self._skeletonize(obstacles)
        skeleton, bridges, bridge_pixels = self._bridge_skeleton_gaps(
            skeleton, resolution
        )
        skeleton, dropped = self._drop_short_strokes(skeleton, resolution)
        stats.bridged_gaps = bridges
        stats.filled_gap_pixels += bridge_pixels
        stats.removed_components += dropped

        height, width = skeleton.shape
        output = np.zeros((height * scale, width * scale), dtype=np.uint8)
        wall_width = max(
            2, int(round(self.settings.wall_width_m / resolution * scale))
        )
        shape_mask = np.zeros_like(skeleton)
        stats.regularized_shapes = self._draw_standard_shapes(
            skeleton, output, shape_mask, resolution, scale, wall_width
        )

        remaining = cv2.bitwise_and(skeleton, cv2.bitwise_not(shape_mask))
        segments = self._merge_line_segments(remaining, resolution)
        for start, end in segments:
            cv2.line(
                output,
                (int(round(start[0] * scale)), int(round(start[1] * scale))),
                (int(round(end[0] * scale)), int(round(end[1] * scale))),
                255,
                wall_width,
                cv2.LINE_AA,
            )
        stats.vector_segments = len(segments)

        if not segments and not stats.regularized_shapes:
            self._draw_simplified_paths(output, remaining, resolution, scale, wall_width)

        return output

    def _bridge_skeleton_gaps(
        self, skeleton: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, int, int]:
        neighbors = cv2.filter2D(
            (skeleton > 0).astype(np.uint8), -1, np.ones((3, 3), dtype=np.uint8)
        )
        endpoints = np.argwhere((skeleton > 0) & (neighbors == 2))
        if len(endpoints) < 2:
            return skeleton, 0, 0

        max_distance = max(2.0, self.settings.max_gap_m / resolution)
        candidates = []
        for first in range(len(endpoints) - 1):
            y1, x1 = endpoints[first]
            inward_first = self._endpoint_inward_vector(skeleton, x1, y1)
            if inward_first is None:
                continue
            for second in range(first + 1, len(endpoints)):
                y2, x2 = endpoints[second]
                delta = np.array([x2 - x1, y2 - y1], dtype=float)
                distance = float(np.linalg.norm(delta))
                if distance < 2.0 or distance > max_distance:
                    continue
                inward_second = self._endpoint_inward_vector(skeleton, x2, y2)
                if inward_second is None:
                    continue
                direction = delta / distance
                if (
                    float(np.dot(inward_first, direction)) < -0.55
                    and float(np.dot(inward_second, -direction)) < -0.55
                ):
                    candidates.append((distance, first, second))

        stitched = skeleton.copy()
        used = set()
        bridges = 0
        bridge_pixels = 0
        for distance, first, second in sorted(candidates):
            if first in used or second in used:
                continue
            y1, x1 = endpoints[first]
            y2, x2 = endpoints[second]
            cv2.line(stitched, (int(x1), int(y1)), (int(x2), int(y2)), 255, 1)
            used.update((first, second))
            bridges += 1
            bridge_pixels += int(round(distance))

        return stitched, bridges, bridge_pixels

    def _endpoint_inward_vector(
        self, skeleton: np.ndarray, x: int, y: int
    ) -> np.ndarray | None:
        radius = self.settings.endpoint_direction_probe_px
        height, width = skeleton.shape
        best_vector = None
        best_distance = 0.0
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx == 0 and dy == 0:
                    continue
                target_x = x + dx
                target_y = y + dy
                if not (0 <= target_x < width and 0 <= target_y < height):
                    continue
                if skeleton[target_y, target_x] == 0:
                    continue
                distance = math.hypot(dx, dy)
                if distance > best_distance:
                    best_distance = distance
                    best_vector = np.array([dx, dy], dtype=float) / distance
        return best_vector

    def _drop_short_strokes(
        self, skeleton: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, int]:
        count, labels, components, _ = cv2.connectedComponentsWithStats(
            skeleton, connectivity=8
        )
        min_length = max(
            4, int(round(self.settings.min_line_length_m / resolution))
        )
        kept = np.zeros_like(skeleton)
        removed = 0
        for label in range(1, count):
            component_width = int(components[label, cv2.CC_STAT_WIDTH])
            component_height = int(components[label, cv2.CC_STAT_HEIGHT])
            if math.hypot(component_width, component_height) >= min_length:
                kept[labels == label] = 255
            else:
                removed += 1
        return kept, removed

    def _draw_standard_shapes(
        self,
        skeleton: np.ndarray,
        output: np.ndarray,
        shape_mask: np.ndarray,
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> int:
        contours, _ = cv2.findContours(skeleton, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        epsilon = max(1.0, self.settings.contour_epsilon_m / resolution)
        seen = set()
        regularized = self._draw_hough_circles(
            skeleton, output, shape_mask, resolution, scale, wall_width, seen
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if area < 25 or perimeter < 20:
                continue
            approximation = cv2.approxPolyDP(contour, epsilon, True)
            points = contour.reshape(-1, 2).astype(float)
            center, radius = cv2.minEnclosingCircle(contour)
            distances = np.linalg.norm(points - np.array(center), axis=1)
            radial_error = float(np.std(distances) / max(np.mean(distances), 1.0))
            circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)

            if circularity > 0.52 and radial_error < 0.16 and radius >= 5:
                key = ("circle", round(center[0] / 8), round(center[1] / 8), round(radius / 8))
                if key in seen:
                    continue
                seen.add(key)
                self._mask_source_contour(shape_mask, contour)
                cv2.circle(
                    shape_mask,
                    (int(round(center[0])), int(round(center[1]))),
                    int(round(radius)),
                    255,
                    3,
                    cv2.LINE_AA,
                )
                cv2.circle(
                    output,
                    (int(round(center[0] * scale)), int(round(center[1] * scale))),
                    int(round(radius * scale)),
                    255,
                    wall_width,
                    cv2.LINE_AA,
                )
                regularized += 1
                continue

            rectangle = cv2.minAreaRect(contour)
            rectangle_area = rectangle[1][0] * rectangle[1][1]
            rectangularity = area / rectangle_area if rectangle_area > 1 else 0.0
            if len(approximation) <= 6 and rectangularity > 0.68:
                key = (
                    "rectangle",
                    round(rectangle[0][0] / 4),
                    round(rectangle[0][1] / 4),
                    round(rectangle[1][0] / 4),
                    round(rectangle[1][1] / 4),
                )
                if key in seen:
                    continue
                seen.add(key)
                self._mask_source_contour(shape_mask, contour)
                box = np.round(cv2.boxPoints(rectangle)).astype(np.int32)
                cv2.polylines(shape_mask, [box], True, 255, 3, cv2.LINE_AA)
                scaled_box = np.round(box * scale).astype(np.int32)
                cv2.polylines(output, [scaled_box], True, 255, wall_width, cv2.LINE_AA)
                regularized += 1

        return regularized

    def _draw_hough_circles(
        self,
        skeleton: np.ndarray,
        output: np.ndarray,
        shape_mask: np.ndarray,
        resolution: float,
        scale: int,
        wall_width: int,
        seen: set,
    ) -> int:
        """Rebuild sufficiently supported arcs as exact circles."""
        min_radius = max(5, int(round(0.20 / resolution)))
        max_radius = max(min_radius + 1, min(skeleton.shape) // 2)
        blurred = cv2.GaussianBlur(skeleton, (3, 3), 0)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(12, min_radius * 2),
            param1=50,
            param2=12,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is None:
            return 0

        distance = cv2.distanceTransform(
            cv2.bitwise_not(skeleton), cv2.DIST_L2, 3
        )
        support_distance = max(2.0, 0.10 / resolution)
        regularized = 0
        accepted: list[tuple[int, int, int]] = []
        for center_x, center_y, radius in np.round(circles[0]).astype(int):
            key = (
                "circle",
                round(center_x / 8),
                round(center_y / 8),
                round(radius / 8),
            )
            if key in seen:
                continue
            duplicate = any(
                math.hypot(center_x - x, center_y - y)
                < max(5, radius * 0.20)
                and abs(radius - saved_radius) < max(4, radius * 0.20)
                for x, y, saved_radius in accepted
            )
            if duplicate:
                continue
            sample_count = max(48, int(round(2 * math.pi * radius)))
            angles = np.linspace(0, 2 * math.pi, sample_count, endpoint=False)
            sample_x = np.clip(
                np.round(center_x + radius * np.cos(angles)).astype(int),
                0,
                skeleton.shape[1] - 1,
            )
            sample_y = np.clip(
                np.round(center_y + radius * np.sin(angles)).astype(int),
                0,
                skeleton.shape[0] - 1,
            )
            support = float(np.mean(distance[sample_y, sample_x] <= support_distance))
            if support < 0.70:
                continue
            seen.add(key)
            accepted.append((center_x, center_y, radius))
            cv2.circle(shape_mask, (center_x, center_y), radius, 255, 5, cv2.LINE_AA)
            cv2.circle(
                output,
                (center_x * scale, center_y * scale),
                radius * scale,
                255,
                wall_width,
                cv2.LINE_AA,
            )
            regularized += 1

        return regularized

    @staticmethod
    def _mask_source_contour(shape_mask: np.ndarray, contour: np.ndarray) -> None:
        cv2.drawContours(shape_mask, [contour], -1, 255, 5, cv2.LINE_AA)

    def _merge_line_segments(
        self, skeleton: np.ndarray, resolution: float
    ) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        min_length = max(
            8, int(round(self.settings.min_line_length_m / resolution))
        )
        merge_gap = max(
            2.0, self.settings.line_merge_gap_m / resolution
        )
        lines = cv2.HoughLinesP(
            skeleton,
            rho=1,
            theta=np.pi / 180,
            threshold=max(8, min_length // 3),
            minLineLength=max(8, int(round(min_length * 0.60))),
            maxLineGap=int(round(merge_gap)),
        )
        if lines is None:
            return []

        orientation_step = math.radians(5)
        offset_step = max(2.0, self.settings.line_offset_m / resolution)
        groups: dict[tuple[int, int], list[tuple[float, float, float]]] = defaultdict(list)
        for x1, y1, x2, y2 in lines[:, 0]:
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.hypot(dx, dy)
            if length < min_length * 0.60:
                continue
            angle = self._snap_angle(math.atan2(dy, dx) % math.pi)
            direction = np.array([math.cos(angle), math.sin(angle)])
            normal = np.array([-direction[1], direction[0]])
            midpoint = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0])
            offset = float(np.dot(midpoint, normal))
            first_projection = float(np.dot(np.array([x1, y1]), direction))
            second_projection = float(np.dot(np.array([x2, y2]), direction))
            key = (
                int(round(angle / orientation_step)),
                int(round(offset / offset_step)),
            )
            groups[key].append((
                min(first_projection, second_projection),
                max(first_projection, second_projection),
                offset,
            ))

        result = []
        for (angle_index, _), intervals in groups.items():
            if not intervals:
                continue
            angle = angle_index * orientation_step
            direction = np.array([math.cos(angle), math.sin(angle)])
            normal = np.array([-direction[1], direction[0]])
            offset = float(np.median([item[2] for item in intervals]))
            intervals.sort(key=lambda item: item[0])
            start, end = intervals[0][0], intervals[0][1]
            for next_start, next_end, _ in intervals[1:]:
                if next_start <= end + merge_gap:
                    end = max(end, next_end)
                    continue
                self._append_segment(result, start, end, direction, normal, offset, min_length)
                start, end = next_start, next_end
            self._append_segment(result, start, end, direction, normal, offset, min_length)

        return result

    @staticmethod
    def _append_segment(
        result: list[tuple[tuple[float, float], tuple[float, float]]],
        start: float,
        end: float,
        direction: np.ndarray,
        normal: np.ndarray,
        offset: float,
        min_length: int,
    ) -> None:
        if end - start < min_length:
            return
        first = direction * start + normal * offset
        second = direction * end + normal * offset
        result.append((tuple(first), tuple(second)))

    @staticmethod
    def _snap_angle(angle: float) -> float:
        candidates = (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4)
        nearest = min(
            candidates,
            key=lambda candidate: min(abs(angle - candidate), math.pi - abs(angle - candidate)),
        )
        if min(abs(angle - nearest), math.pi - abs(angle - nearest)) <= math.radians(12):
            return nearest
        snapped = round(angle / math.radians(5)) * math.radians(5)
        return snapped % math.pi

    def _draw_simplified_paths(
        self,
        output: np.ndarray,
        skeleton: np.ndarray,
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> None:
        contours, _ = cv2.findContours(skeleton, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        epsilon = max(1.0, self.settings.contour_epsilon_m / resolution)
        min_length = self.settings.min_line_length_m / resolution
        for contour in contours:
            if cv2.arcLength(contour, False) < min_length:
                continue
            approximation = cv2.approxPolyDP(contour, epsilon, False)
            if len(approximation) < 2:
                continue
            scaled = np.round(approximation.reshape(-1, 2) * scale).astype(np.int32)
            cv2.polylines(output, [scaled], False, 255, wall_width, cv2.LINE_AA)

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
