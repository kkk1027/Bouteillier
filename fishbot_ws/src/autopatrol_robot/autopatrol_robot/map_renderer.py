"""Clean display-only rendering for static occupancy maps."""
from __future__ import annotations

import base64
import math
from dataclasses import asdict, dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class MapRenderSettings:
    """Metric thresholds for conservative cleanup with object preservation."""

    occupied_threshold: int = 65
    min_island_area_m2: float = 0.012
    min_island_length_m: float = 0.20
    max_gap_m: float = 0.22
    min_straight_wall_length_m: float = 0.60
    line_merge_gap_m: float = 0.35
    vector_join_gap_m: float = 0.85
    corner_snap_m: float = 2.10
    duplicate_wall_separation_m: float = 0.45
    duplicate_wall_overlap_m: float = 0.70
    max_unclosed_gap_m: float = 3.50
    closure_axis_alignment: float = 0.90
    near_round_circularity: float = 0.76
    near_round_aspect: float = 0.84
    near_round_disk_ratio: float = 0.68
    main_body_anchor_ratio: float = 0.35
    wall_width_m: float = 0.08
    min_round_radius_m: float = 0.10
    max_round_radius_m: float = 1.25
    endpoint_direction_probe_px: int = 5
    render_scale: int = 2


@dataclass
class MapRenderStats:
    removed_components: int = 0
    filled_gap_pixels: int = 0
    bridged_gaps: int = 0
    vector_segments: int = 0
    removed_duplicate_segments: int = 0
    closed_wall_gaps: int = 0
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
        body, removed_outside_body = self._retain_main_map_body(closed, resolution)

        stats = MapRenderStats(
            removed_components=removed + removed_after_close + removed_outside_body,
            filled_gap_pixels=max(
                0, int(cv2.countNonZero(closed) - cv2.countNonZero(filtered))
            ),
        )
        return body, stats

    def _retain_main_map_body(
        self, mask: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, int]:
        """Keep the dominant map outline and obstacles that lie inside it."""
        count, labels, components, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        if count <= 2:
            return mask, 0

        areas = components[1:, cv2.CC_STAT_AREA]
        largest_area = int(np.max(areas))
        minimum_anchor_area = max(
            int(round(largest_area * self.settings.main_body_anchor_ratio)),
            int(round(0.08 / resolution)),
        )
        anchor_labels = [
            label
            for label in range(1, count)
            if int(components[label, cv2.CC_STAT_AREA]) >= minimum_anchor_area
        ]
        if not anchor_labels:
            anchor_labels = [int(np.argmax(areas)) + 1]

        body_mask = np.zeros_like(mask)
        for label in anchor_labels:
            body_mask[labels == label] = 255
        points = np.column_stack(np.where(body_mask > 0))[:, ::-1].astype(np.int32)
        if len(points) < 3:
            return mask, 0
        body_hull = cv2.convexHull(points.reshape(-1, 1, 2))

        bridge_distance = max(4.0, 0.75 / resolution)
        anchor_distances = []
        for anchor_label in anchor_labels:
            anchor_mask = np.where(labels == anchor_label, 0, 255).astype(
                np.uint8
            )
            anchor_distances.append(
                cv2.distanceTransform(anchor_mask, cv2.DIST_L2, 3)
            )

        retained = np.zeros_like(mask)
        removed = 0
        for label in range(1, count):
            component = labels == label
            center_x, center_y = centroids[label]
            is_anchor = label in anchor_labels
            is_inside_body = cv2.pointPolygonTest(
                body_hull, (float(center_x), float(center_y)), False
            ) >= 0
            nearby_anchors = sum(
                np.min(distance[component]) <= bridge_distance
                for distance in anchor_distances
            )
            completes_body_edge = nearby_anchors >= 2
            if is_anchor or is_inside_body or completes_body_edge:
                retained[component] = 255
            else:
                removed += 1

        return retained, removed

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
        """Render joined, denoised walls at a uniform width."""
        smoothed = self._smooth_wall_mask(obstacles, resolution)
        skeleton = self._skeletonize(smoothed)
        skeleton, bridges, bridge_pixels = self._bridge_skeleton_gaps(
            skeleton, resolution
        )
        stats.bridged_gaps = bridges
        stats.filled_gap_pixels += bridge_pixels

        wall_width = max(
            2, int(round(self.settings.wall_width_m / resolution * scale))
        )
        (
            output,
            segments,
            joins,
            join_pixels,
            removed_duplicates,
            closed_gaps,
        ) = self._draw_standard_wall_paths(
            skeleton, resolution, scale, wall_width
        )
        stats.vector_segments = segments
        stats.removed_duplicate_segments = removed_duplicates
        stats.closed_wall_gaps = closed_gaps
        stats.bridged_gaps += joins
        stats.filled_gap_pixels += join_pixels
        stats.regularized_shapes = self._regularize_round_obstacles(
            obstacles, skeleton, output, resolution, scale, wall_width
        )
        return output

    def _draw_standard_wall_paths(
        self,
        skeleton: np.ndarray,
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> tuple[np.ndarray, int, int, int]:
        """Redraw dominant wall strokes and bridge their geometric endpoints."""
        height, width = skeleton.shape
        canvas = np.zeros((height * scale, width * scale), dtype=np.uint8)
        minimum_length = max(
            8, int(round(self.settings.min_straight_wall_length_m / resolution))
        )
        max_gap = max(3, int(round(self.settings.line_merge_gap_m / resolution)))
        threshold = max(8, int(round(minimum_length * 0.70)))
        lines = cv2.HoughLinesP(
            skeleton,
            rho=1,
            theta=math.pi / 360.0,
            threshold=threshold,
            minLineLength=minimum_length,
            maxLineGap=max_gap,
        )
        if lines is None:
            return canvas, 0, 0, 0, 0, 0

        accepted: list[tuple[int, int, int, int]] = []
        for x1, y1, x2, y2 in lines[:, 0]:
            candidate = (int(x1), int(y1), int(x2), int(y2))
            if self._is_redundant_line(candidate, accepted):
                continue
            accepted.append(candidate)

        snapped, corner_joins, corner_pixels = self._snap_line_corners(
            accepted, resolution
        )
        deduplicated, removed_duplicates = self._remove_parallel_duplicates(
            snapped, resolution
        )
        # A removed inner duplicate may have claimed a corner first; snap the
        # remaining principal walls once more so the exterior boundary closes.
        snapped, final_corner_joins, final_corner_pixels = self._snap_line_corners(
            deduplicated,
            resolution,
            maximum_extension_m=0.50,
        )
        for candidate in snapped:
            cv2.line(
                canvas,
                (candidate[0] * scale, candidate[1] * scale),
                (candidate[2] * scale, candidate[3] * scale),
                255,
                wall_width,
                cv2.LINE_AA,
            )

        closed_gaps, closed_pixels = self._close_open_wall_gaps(
            canvas, snapped, resolution, scale, wall_width
        )
        joins, join_pixels = self._bridge_vector_line_endpoints(
            canvas, snapped, resolution, scale, wall_width
        )
        return (
            canvas,
            len(snapped),
            corner_joins + final_corner_joins + closed_gaps + joins,
            corner_pixels + final_corner_pixels + closed_pixels + join_pixels,
            removed_duplicates,
            closed_gaps,
        )

    def _close_open_wall_gaps(
        self,
        canvas: np.ndarray,
        lines: list[tuple[int, int, int, int]],
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> tuple[int, int]:
        """Close compatible open wall endpoints after all topology filtering."""
        segments = [
            np.array([[x1, y1], [x2, y2]], dtype=float)
            for x1, y1, x2, y2 in lines
        ]
        if len(segments) < 2:
            return 0, 0
        endpoints = []
        tolerance = max(2.5, wall_width / max(scale, 1) + 1.0)
        for line_index, segment in enumerate(segments):
            direction = segment[1] - segment[0]
            length = float(np.linalg.norm(direction))
            if length < 1.0:
                continue
            direction /= length
            for endpoint_index, point in enumerate(segment):
                nearest_wall = min(
                    self._point_to_segment_distance(point, other[0], other[1])
                    for other_index, other in enumerate(segments)
                    if other_index != line_index
                )
                if nearest_wall <= tolerance:
                    continue
                outward = -direction if endpoint_index == 0 else direction
                endpoints.append((line_index, endpoint_index, point, outward))

        maximum_gap = max(4.0, self.settings.max_unclosed_gap_m / resolution)
        candidates = []
        for first in range(len(endpoints) - 1):
            first_line, _, first_point, first_outward = endpoints[first]
            for second in range(first + 1, len(endpoints)):
                second_line, _, second_point, second_outward = endpoints[second]
                if first_line == second_line:
                    continue
                delta = second_point - first_point
                distance = float(np.linalg.norm(delta))
                if distance < tolerance or distance > maximum_gap:
                    continue
                axis_alignment = max(abs(delta[0]), abs(delta[1])) / distance
                if axis_alignment < self.settings.closure_axis_alignment:
                    continue
                # The final completion pass only draws straight horizontal or
                # vertical boundary closures. Diagonal joins are handled by the
                # earlier geometry snap and must not cut across a room.
                candidates.append((distance, distance, first, second))

        used = set()
        closed = 0
        pixels = 0
        for _, distance, first, second in sorted(candidates):
            if first in used or second in used:
                continue
            _, _, first_point, _ = endpoints[first]
            _, _, second_point, _ = endpoints[second]
            cv2.line(
                canvas,
                (int(round(first_point[0] * scale)), int(round(first_point[1] * scale))),
                (int(round(second_point[0] * scale)), int(round(second_point[1] * scale))),
                255,
                wall_width,
                cv2.LINE_AA,
            )
            used.update((first, second))
            closed += 1
            pixels += int(round(distance))
        return closed, pixels

    @staticmethod
    def _point_to_segment_distance(
        point: np.ndarray, start: np.ndarray, end: np.ndarray
    ) -> float:
        direction = end - start
        length_squared = float(np.dot(direction, direction))
        if length_squared < 1e-6:
            return float(np.linalg.norm(point - start))
        projection = float(
            np.clip(np.dot(point - start, direction) / length_squared, 0.0, 1.0)
        )
        return float(np.linalg.norm(point - (start + projection * direction)))

    def _remove_parallel_duplicates(
        self,
        lines: list[tuple[int, int, int, int]],
        resolution: float,
    ) -> tuple[list[tuple[int, int, int, int]], int]:
        """Discard a shorter wall stroke shadowing a longer parallel stroke."""
        if len(lines) < 2:
            return lines, 0
        maximum_separation = max(
            2.0, self.settings.duplicate_wall_separation_m / resolution
        )
        minimum_overlap = max(
            4.0, self.settings.duplicate_wall_overlap_m / resolution
        )
        geometry = [
            np.array([[x1, y1], [x2, y2]], dtype=float)
            for x1, y1, x2, y2 in lines
        ]
        candidates = []
        for first in range(len(geometry) - 1):
            first_direction = geometry[first][1] - geometry[first][0]
            first_length = float(np.linalg.norm(first_direction))
            if first_length < 1.0:
                continue
            first_unit = first_direction / first_length
            for second in range(first + 1, len(geometry)):
                second_direction = geometry[second][1] - geometry[second][0]
                second_length = float(np.linalg.norm(second_direction))
                if second_length < 1.0:
                    continue
                second_unit = second_direction / second_length
                if abs(float(np.dot(first_unit, second_unit))) < 0.985:
                    continue
                projections = np.dot(geometry[second] - geometry[first][0], first_unit)
                overlap = max(
                    0.0,
                    min(first_length, float(np.max(projections)))
                    - max(0.0, float(np.min(projections))),
                )
                if overlap < minimum_overlap:
                    continue
                midpoint = np.mean(geometry[second], axis=0) - geometry[first][0]
                separation = abs(
                    first_unit[0] * midpoint[1] - first_unit[1] * midpoint[0]
                )
                if separation > maximum_separation:
                    continue
                # Preserve the stronger stroke; it is less likely to be a
                # noisy echo of an exterior wall.
                remove = second if first_length >= second_length else first
                keep = first if remove == second else second
                candidates.append((separation, -overlap, remove, keep))

        removed: set[int] = set()
        for _, _, remove, keep in sorted(candidates):
            if remove in removed or keep in removed:
                continue
            removed.add(remove)
        return [line for index, line in enumerate(lines) if index not in removed], len(removed)

    def _snap_line_corners(
        self,
        lines: list[tuple[int, int, int, int]],
        resolution: float,
        maximum_extension_m: float | None = None,
    ) -> tuple[list[tuple[int, int, int, int]], int, int]:
        """Extend compatible walls to a shared geometric corner.

        Hough returns independent strokes.  Joining their nearest pixels creates
        diagonal patches; extending them to their line intersection preserves a
        clean horizontal/vertical (or arbitrary-angle) corner instead.
        """
        if len(lines) < 2:
            return lines, 0, 0

        geometry = [
            np.array([[x1, y1], [x2, y2]], dtype=float)
            for x1, y1, x2, y2 in lines
        ]
        extension_limit_m = (
            self.settings.corner_snap_m
            if maximum_extension_m is None
            else maximum_extension_m
        )
        maximum_extension = max(4.0, extension_limit_m / resolution)
        candidates = []
        for first in range(len(geometry) - 1):
            first_line = geometry[first]
            first_direction = first_line[1] - first_line[0]
            first_length = float(np.linalg.norm(first_direction))
            if first_length < 1.0:
                continue
            for second in range(first + 1, len(geometry)):
                second_line = geometry[second]
                second_direction = second_line[1] - second_line[0]
                second_length = float(np.linalg.norm(second_direction))
                if second_length < 1.0:
                    continue
                cross = float(
                    first_direction[0] * second_direction[1]
                    - first_direction[1] * second_direction[0]
                )
                # Near-parallel fragments are handled by the endpoint merger.
                if abs(cross) / (first_length * second_length) < 0.35:
                    continue
                intersection = self._line_intersection(
                    first_line[0], first_direction, second_line[0], second_direction
                )
                if intersection is None:
                    continue
                if not (
                    -2.0 <= intersection[0] <= max(line[:, 0].max() for line in geometry) + 2.0
                    and -2.0 <= intersection[1] <= max(line[:, 1].max() for line in geometry) + 2.0
                ):
                    continue
                first_endpoint, first_extension = self._corner_extension(
                    first_line, intersection
                )
                second_endpoint, second_extension = self._corner_extension(
                    second_line, intersection
                )
                if first_endpoint is None and second_endpoint is None:
                    continue
                if (
                    first_extension > maximum_extension
                    or second_extension > maximum_extension
                ):
                    continue
                candidates.append(
                    (
                        first_extension + second_extension,
                        first,
                        first_endpoint,
                        second,
                        second_endpoint,
                        intersection,
                    )
                )

        used_endpoints: set[tuple[int, int]] = set()
        joins = 0
        pixels = 0
        for _, first, first_endpoint, second, second_endpoint, intersection in sorted(candidates):
            requested = [
                (first, first_endpoint),
                (second, second_endpoint),
            ]
            if any(
                endpoint is not None and (line, endpoint) in used_endpoints
                for line, endpoint in requested
            ):
                continue
            changed = False
            for line, endpoint in requested:
                if endpoint is None:
                    continue
                distance = float(np.linalg.norm(geometry[line][endpoint] - intersection))
                if distance < 1.0:
                    continue
                geometry[line][endpoint] = intersection
                used_endpoints.add((line, endpoint))
                pixels += int(round(distance))
                changed = True
            if changed:
                joins += 1

        snapped = [
            (
                int(round(line[0, 0])),
                int(round(line[0, 1])),
                int(round(line[1, 0])),
                int(round(line[1, 1])),
            )
            for line in geometry
        ]
        return snapped, joins, pixels

    @staticmethod
    def _line_intersection(
        first_point: np.ndarray,
        first_direction: np.ndarray,
        second_point: np.ndarray,
        second_direction: np.ndarray,
    ) -> np.ndarray | None:
        denominator = float(
            first_direction[0] * second_direction[1]
            - first_direction[1] * second_direction[0]
        )
        if abs(denominator) < 1e-6:
            return None
        delta = second_point - first_point
        multiplier = float(
            (delta[0] * second_direction[1] - delta[1] * second_direction[0])
            / denominator
        )
        return first_point + first_direction * multiplier

    @staticmethod
    def _corner_extension(
        line: np.ndarray, intersection: np.ndarray
    ) -> tuple[int | None, float]:
        """Return a line endpoint to extend, or no endpoint when it already touches."""
        direction = line[1] - line[0]
        length = float(np.linalg.norm(direction))
        if length < 1e-6:
            return None, float("inf")
        unit = direction / length
        projection = float(np.dot(intersection - line[0], unit))
        if 0.0 <= projection <= length:
            return None, 0.0
        endpoint = 0 if projection < 0.0 else 1
        other = 1 - endpoint
        outward = intersection - line[endpoint]
        inward = line[other] - line[endpoint]
        if float(np.dot(outward, inward)) > 0.1:
            return None, float("inf")
        return endpoint, float(np.linalg.norm(outward))

    def _bridge_vector_line_endpoints(
        self,
        canvas: np.ndarray,
        lines: list[tuple[int, int, int, int]],
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> tuple[int, int]:
        """Join compatible Hough endpoints to restore a continuous perimeter."""
        endpoints = []
        for line_index, (x1, y1, x2, y2) in enumerate(lines):
            direction = np.array([x2 - x1, y2 - y1], dtype=float)
            length = float(np.linalg.norm(direction))
            if length == 0:
                continue
            direction /= length
            endpoints.extend(
                [
                    ((x1, y1), -direction, line_index),
                    ((x2, y2), direction, line_index),
                ]
            )

        max_gap = max(3.0, self.settings.vector_join_gap_m / resolution)
        candidates = []
        for first in range(len(endpoints) - 1):
            first_point, first_direction, first_line = endpoints[first]
            for second in range(first + 1, len(endpoints)):
                second_point, second_direction, second_line = endpoints[second]
                if first_line == second_line:
                    continue
                delta = np.array(second_point, dtype=float) - np.array(
                    first_point, dtype=float
                )
                distance = float(np.linalg.norm(delta))
                if distance < 1.5 or distance > max_gap:
                    continue
                direction = delta / distance
                first_alignment = abs(float(np.dot(first_direction, direction)))
                second_alignment = abs(
                    float(np.dot(second_direction, -direction))
                )
                aligned = first_alignment >= 0.35 and second_alignment >= 0.35
                right_angle = (
                    max(first_alignment, second_alignment) >= 0.85
                    and min(first_alignment, second_alignment) <= 0.25
                )
                if aligned or right_angle:
                    candidates.append((distance, first, second))

        used = set()
        joins = 0
        join_pixels = 0
        for distance, first, second in sorted(candidates):
            if first in used or second in used:
                continue
            first_point, _, _ = endpoints[first]
            second_point, _, _ = endpoints[second]
            cv2.line(
                canvas,
                (first_point[0] * scale, first_point[1] * scale),
                (second_point[0] * scale, second_point[1] * scale),
                255,
                wall_width,
                cv2.LINE_AA,
            )
            used.update((first, second))
            joins += 1
            join_pixels += int(round(distance))

        return joins, join_pixels

    @staticmethod
    def _is_redundant_line(
        candidate: tuple[int, int, int, int],
        accepted: list[tuple[int, int, int, int]],
    ) -> bool:
        """Avoid redrawing overlapping Hough detections of the same stroke."""
        x1, y1, x2, y2 = candidate
        direction = np.array([x2 - x1, y2 - y1], dtype=float)
        length = float(np.linalg.norm(direction))
        if length == 0:
            return True
        direction /= length
        midpoint = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0])

        for saved in accepted:
            sx1, sy1, sx2, sy2 = saved
            saved_direction = np.array([sx2 - sx1, sy2 - sy1], dtype=float)
            saved_length = float(np.linalg.norm(saved_direction))
            if saved_length == 0:
                continue
            saved_direction /= saved_length
            if abs(float(np.dot(direction, saved_direction))) < 0.985:
                continue
            saved_start = np.array([sx1, sy1], dtype=float)
            offset = midpoint - saved_start
            perpendicular = abs(
                saved_direction[0] * offset[1] - saved_direction[1] * offset[0]
            )
            projection = float(np.dot(offset, saved_direction))
            if perpendicular <= 4.0 and -4.0 <= projection <= saved_length + 4.0:
                return True
        return False

    def _smooth_wall_mask(
        self, obstacles: np.ndarray, resolution: float
    ) -> np.ndarray:
        """Join sub-cell breaks and suppress stair-step noise before thinning."""
        radius = max(1, int(round(0.06 / resolution)))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1)
        )
        joined = cv2.morphologyEx(obstacles, cv2.MORPH_CLOSE, kernel)
        thickened = cv2.dilate(
            joined,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        softened = cv2.GaussianBlur(thickened, (5, 5), 0)
        _, smoothed = cv2.threshold(softened, 100, 255, cv2.THRESH_BINARY)
        return cv2.morphologyEx(smoothed, cv2.MORPH_CLOSE, kernel)

    def _regularize_round_obstacles(
        self,
        obstacles: np.ndarray,
        skeleton: np.ndarray,
        output: np.ndarray,
        resolution: float,
        scale: int,
        wall_width: int,
    ) -> int:
        """Replace isolated circular obstacles and supported arcs with exact circles."""
        min_radius = max(2, int(round(self.settings.min_round_radius_m / resolution)))
        max_radius = min(
            int(round(self.settings.max_round_radius_m / resolution)),
            min(obstacles.shape) // 2,
        )
        accepted: list[tuple[float, float, float]] = []
        regularized = 0
        count, labels, components, _ = cv2.connectedComponentsWithStats(
            obstacles, connectivity=8
        )
        for label in range(1, count):
            width = int(components[label, cv2.CC_STAT_WIDTH])
            height = int(components[label, cv2.CC_STAT_HEIGHT])
            if not width or not height:
                continue
            aspect = min(width, height) / max(width, height)
            if aspect < 0.72:
                continue
            component = np.zeros_like(obstacles)
            component[labels == label] = 255
            contours, _ = cv2.findContours(
                component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
            )
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)
            circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
            (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
            disk_ratio = area / max(math.pi * radius * radius, 1.0)
            strong_round = circularity >= 0.82
            lightly_damaged_round = (
                circularity >= self.settings.near_round_circularity
                and aspect >= self.settings.near_round_aspect
                and disk_ratio >= self.settings.near_round_disk_ratio
            )
            if (
                not (strong_round or lightly_damaged_round)
                or radius < min_radius
                or radius > max_radius
            ):
                continue
            if self._replace_with_standard_circle(
                output,
                center_x,
                center_y,
                radius,
                scale,
                wall_width,
                accepted,
            ):
                regularized += 1

        circles = cv2.HoughCircles(
            cv2.GaussianBlur(skeleton, (3, 3), 0),
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(12, min_radius * 2),
            param1=50,
            param2=12,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is None:
            return regularized

        distance = cv2.distanceTransform(
            cv2.bitwise_not(skeleton), cv2.DIST_L2, 3
        )
        support_distance = max(2.0, 0.08 / resolution)
        for center_x, center_y, radius in np.round(circles[0]).astype(int):
            angles = np.linspace(
                0, 2 * math.pi, max(48, int(round(2 * math.pi * radius))), endpoint=False
            )
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
            if self._replace_with_standard_circle(
                output,
                float(center_x),
                float(center_y),
                float(radius),
                scale,
                wall_width,
                accepted,
            ):
                regularized += 1

        return regularized

    @staticmethod
    def _replace_with_standard_circle(
        output: np.ndarray,
        center_x: float,
        center_y: float,
        radius: float,
        scale: int,
        wall_width: int,
        accepted: list[tuple[float, float, float]],
    ) -> bool:
        if any(
            math.hypot(center_x - saved_x, center_y - saved_y)
            < max(4.0, radius * 0.25)
            and abs(radius - saved_radius) < max(3.0, radius * 0.25)
            for saved_x, saved_y, saved_radius in accepted
        ):
            return False
        center = (int(round(center_x * scale)), int(round(center_y * scale)))
        scaled_radius = int(round(radius * scale))
        cv2.circle(output, center, scaled_radius + wall_width * 2, 0, -1)
        cv2.circle(output, center, scaled_radius, 255, wall_width, cv2.LINE_AA)
        accepted.append((center_x, center_y, radius))
        return True

    def _bridge_skeleton_gaps(
        self, skeleton: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, int, int]:
        neighbors = cv2.filter2D(
            (skeleton > 0).astype(np.uint8), -1, np.ones((3, 3), dtype=np.uint8)
        )
        endpoints = np.argwhere((skeleton > 0) & (neighbors == 2))
        if len(endpoints) < 2:
            return skeleton, 0, 0
        _, component_labels = cv2.connectedComponents(
            (skeleton > 0).astype(np.uint8), connectivity=8
        )

        max_distance = max(2.0, self.settings.max_gap_m / resolution)
        candidates = []
        for first in range(len(endpoints) - 1):
            y1, x1 = endpoints[first]
            inward_first = self._endpoint_inward_vector(
                skeleton, x1, y1, component_labels
            )
            if inward_first is None:
                continue
            for second in range(first + 1, len(endpoints)):
                y2, x2 = endpoints[second]
                delta = np.array([x2 - x1, y2 - y1], dtype=float)
                distance = float(np.linalg.norm(delta))
                if distance < 2.0 or distance > max_distance:
                    continue
                inward_second = self._endpoint_inward_vector(
                    skeleton, x2, y2, component_labels
                )
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
        self,
        skeleton: np.ndarray,
        x: int,
        y: int,
        component_labels: np.ndarray,
    ) -> np.ndarray | None:
        radius = self.settings.endpoint_direction_probe_px
        height, width = skeleton.shape
        best_vector = None
        best_distance = 0.0
        component_label = component_labels[y, x]
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx == 0 and dy == 0:
                    continue
                target_x = x + dx
                target_y = y + dy
                if not (0 <= target_x < width and 0 <= target_y < height):
                    continue
                if component_labels[target_y, target_x] != component_label:
                    continue
                distance = math.hypot(dx, dy)
                if distance > best_distance:
                    best_distance = distance
                    best_vector = np.array([dx, dy], dtype=float) / distance
        return best_vector

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
