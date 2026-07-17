"""Display-only reconstruction of obstacles inside a validated map body."""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class InteriorObstacleSettings:
    """Metric limits that prevent interior cleanup from changing outer walls."""

    boundary_clearance_m: float = 0.18
    local_close_gap_m: float = 0.10
    min_component_area_m2: float = 0.010
    min_component_length_m: float = 0.25
    min_round_radius_m: float = 0.10
    max_round_radius_m: float = 1.25
    min_round_circularity: float = 0.80
    min_round_aspect: float = 0.90
    min_round_disk_ratio: float = 0.70
    min_ellipse_aspect: float = 0.58
    max_ellipse_aspect: float = 0.92
    min_ellipse_support: float = 0.70
    min_rectangle_extent: float = 0.68
    line_aspect_ratio: float = 3.0
    min_line_length_m: float = 0.30
    irregular_min_area_m2: float = 0.025
    irregular_simplify_m: float = 0.08


@dataclass
class InteriorObstacleStats:
    candidates: int = 0
    removed_components: int = 0
    filled_gap_pixels: int = 0
    circles: int = 0
    ellipses: int = 0
    rectangles: int = 0
    lines: int = 0
    irregulars: int = 0
    filled_closed_shapes: int = 0

    def to_dict(self) -> dict:
        return asdict(self)

    @property
    def regularized_shapes(self) -> int:
        return self.circles + self.ellipses + self.rectangles


class InteriorObstacleRenderer:
    """Render conservative, regularized obstacles without touching the exterior."""

    def __init__(self, settings: InteriorObstacleSettings | None = None):
        self.settings = settings or InteriorObstacleSettings()

    def draw(
        self,
        obstacles: np.ndarray,
        primary_floor: np.ndarray,
        resolution: float,
        scale: int,
        canvas: np.ndarray,
        color: tuple[int, int, int],
        line_width: int,
    ) -> InteriorObstacleStats:
        """Draw cleaned obstacle edges onto a BGR canvas and return quality data."""
        if obstacles.shape != primary_floor.shape:
            raise ValueError("障碍物和主体区域尺寸不一致")
        if resolution <= 0:
            raise ValueError("地图分辨率必须大于零")

        stats = InteriorObstacleStats()
        interior = self._interior_region(primary_floor, resolution)
        candidates = cv2.bitwise_and(obstacles, interior)
        if cv2.countNonZero(candidates) == 0:
            return stats

        candidates, filled = self._close_local_gaps(candidates, interior, resolution)
        stats.filled_gap_pixels = filled
        count, labels, components, _ = cv2.connectedComponentsWithStats(
            candidates, connectivity=8
        )
        accepted_circles: list[tuple[float, float, float]] = []
        unresolved = candidates.copy()

        for label in range(1, count):
            component = np.where(labels == label, 255, 0).astype(np.uint8)
            if not self._is_substantial(component, components[label], resolution):
                stats.removed_components += 1
                continue
            stats.candidates += 1
            shape = self._classify(component, resolution)
            if shape is None:
                stats.removed_components += 1
                continue
            kind, geometry = shape
            unresolved[labels == label] = 0
            self._draw_shape(
                canvas, kind, geometry, scale, color, line_width
            )
            if kind == "circle":
                center_x, center_y, radius = geometry
                accepted_circles.append((center_x, center_y, radius))
                stats.circles += 1
                stats.filled_closed_shapes += 1
            elif kind == "ellipse":
                stats.ellipses += 1
                stats.filled_closed_shapes += 1
            elif kind == "rectangle":
                stats.rectangles += 1
                stats.filled_closed_shapes += 1
            elif kind == "line":
                stats.lines += 1
            else:
                stats.irregulars += 1
                stats.filled_closed_shapes += 1

        # Hough only supplements damaged arcs that were not already rebuilt from
        # a connected component. The sampled support blocks false detections.
        for center_x, center_y, radius in self._supported_hough_circles(
            unresolved, interior, resolution
        ):
            if self._is_duplicate_circle(
                center_x, center_y, radius, accepted_circles
            ):
                continue
            self._draw_shape(
                canvas,
                "circle",
                (center_x, center_y, radius),
                scale,
                color,
                line_width,
            )
            accepted_circles.append((center_x, center_y, radius))
            stats.circles += 1
            stats.filled_closed_shapes += 1

        return stats

    def _interior_region(
        self, primary_floor: np.ndarray, resolution: float
    ) -> np.ndarray:
        clearance = max(
            2, int(math.ceil(self.settings.boundary_clearance_m / resolution))
        )
        distance = cv2.distanceTransform(primary_floor, cv2.DIST_L2, 5)
        return np.where(distance >= clearance, 255, 0).astype(np.uint8)

    def _close_local_gaps(
        self,
        candidates: np.ndarray,
        interior: np.ndarray,
        resolution: float,
    ) -> tuple[np.ndarray, int]:
        radius = max(
            1, int(round(self.settings.local_close_gap_m / (2.0 * resolution)))
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1)
        )
        closed = cv2.morphologyEx(candidates, cv2.MORPH_CLOSE, kernel)
        closed = cv2.bitwise_and(closed, interior)
        return closed, max(0, cv2.countNonZero(closed) - cv2.countNonZero(candidates))

    def _is_substantial(
        self,
        component: np.ndarray,
        component_stats: np.ndarray,
        resolution: float,
    ) -> bool:
        area = int(component_stats[cv2.CC_STAT_AREA])
        width = int(component_stats[cv2.CC_STAT_WIDTH])
        height = int(component_stats[cv2.CC_STAT_HEIGHT])
        min_area = max(
            4,
            int(round(self.settings.min_component_area_m2 / (resolution * resolution))),
        )
        min_length = max(
            4, int(round(self.settings.min_component_length_m / resolution))
        )
        return area >= min_area or math.hypot(width, height) >= min_length

    def _classify(
        self, component: np.ndarray, resolution: float
    ) -> tuple[str, object] | None:
        contours, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 1.0:
            return None
        area = cv2.contourArea(contour)
        x, y, width, height = cv2.boundingRect(contour)
        aspect = min(width, height) / max(width, height)
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
        disk_ratio = area / max(math.pi * radius * radius, 1.0)
        min_radius = max(
            2.0, self.settings.min_round_radius_m / resolution
        )
        max_radius = min(
            self.settings.max_round_radius_m / resolution,
            min(component.shape) / 2.0,
        )
        if (
            min_radius <= radius <= max_radius
            and aspect >= self.settings.min_round_aspect
            and circularity >= self.settings.min_round_circularity
            and disk_ratio >= self.settings.min_round_disk_ratio
        ):
            return "circle", (center_x, center_y, radius)

        ellipse = self._fit_supported_ellipse(contour, resolution)
        if ellipse is not None:
            return "ellipse", ellipse

        rectangle = self._fit_rectangle(contour, perimeter, area)
        if rectangle is not None:
            return "rectangle", rectangle

        line = self._fit_line(component, resolution)
        if line is not None:
            return "line", line

        minimum_irregular_area = max(
            6,
            int(round(self.settings.irregular_min_area_m2 / (resolution * resolution))),
        )
        if area < minimum_irregular_area:
            return None
        epsilon = max(1.0, self.settings.irregular_simplify_m / resolution)
        simplified = cv2.approxPolyDP(contour, epsilon, True)
        if len(simplified) < 3:
            return None
        return "irregular", simplified.reshape(-1, 2)

    def _fit_supported_ellipse(
        self, contour: np.ndarray, resolution: float
    ) -> tuple[tuple[float, float], tuple[float, float], float] | None:
        if len(contour) < 5:
            return None
        (center_x, center_y), axes, angle = cv2.fitEllipse(contour)
        major, minor = max(axes), min(axes)
        if minor <= 0:
            return None
        aspect = minor / major
        minimum_axis = max(4.0, 2.0 * self.settings.min_round_radius_m / resolution)
        if (
            not self.settings.min_ellipse_aspect <= aspect < self.settings.max_ellipse_aspect
            or minor < minimum_axis
        ):
            return None
        radians = math.radians(angle)
        cosine, sine = math.cos(radians), math.sin(radians)
        points = contour.reshape(-1, 2).astype(float)
        delta_x = points[:, 0] - center_x
        delta_y = points[:, 1] - center_y
        rotated_x = cosine * delta_x + sine * delta_y
        rotated_y = -sine * delta_x + cosine * delta_y
        normalized = (rotated_x / (axes[0] / 2.0)) ** 2 + (
            rotated_y / (axes[1] / 2.0)
        ) ** 2
        support = float(np.mean(np.abs(normalized - 1.0) <= 0.34))
        if support < self.settings.min_ellipse_support:
            return None
        return (center_x, center_y), axes, angle

    def _fit_rectangle(
        self,
        contour: np.ndarray,
        perimeter: float,
        area: float,
    ) -> np.ndarray | None:
        approximation = cv2.approxPolyDP(
            contour, max(1.0, perimeter * 0.025), True
        )
        if len(approximation) != 4 or not cv2.isContourConvex(approximation):
            return None
        rectangle = cv2.minAreaRect(contour)
        width, height = rectangle[1]
        if min(width, height) < 2.0:
            return None
        extent = area / max(width * height, 1.0)
        aspect = max(width, height) / min(width, height)
        if extent < self.settings.min_rectangle_extent:
            return None
        if aspect >= self.settings.line_aspect_ratio:
            return None
        return cv2.boxPoints(rectangle)

    def _fit_line(
        self,
        component: np.ndarray,
        resolution: float,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        points = np.column_stack(np.where(component > 0))[:, ::-1].astype(
            np.float32
        )
        if len(points) < 4:
            return None
        center = np.mean(points, axis=0)
        covariance = np.cov(points.T)
        values, vectors = np.linalg.eigh(covariance)
        direction = vectors[:, int(np.argmax(values))]
        minor = max(float(np.min(values)), 1e-6)
        aspect = math.sqrt(float(np.max(values)) / minor)
        projections = (points - center) @ direction
        length = float(np.max(projections) - np.min(projections))
        if (
            aspect < self.settings.line_aspect_ratio
            or length < self.settings.min_line_length_m / resolution
        ):
            return None
        start = center + direction * float(np.min(projections))
        end = center + direction * float(np.max(projections))
        return start, end

    def _supported_hough_circles(
        self,
        candidates: np.ndarray,
        interior: np.ndarray,
        resolution: float,
    ) -> list[tuple[float, float, float]]:
        minimum = max(2, int(round(self.settings.min_round_radius_m / resolution)))
        maximum = min(
            int(round(self.settings.max_round_radius_m / resolution)),
            min(candidates.shape) // 2,
        )
        if maximum <= minimum:
            return []
        circles = cv2.HoughCircles(
            cv2.GaussianBlur(candidates, (3, 3), 0),
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(12, minimum * 2),
            param1=50,
            param2=13,
            minRadius=minimum,
            maxRadius=maximum,
        )
        if circles is None:
            return []
        distance = cv2.distanceTransform(
            cv2.bitwise_not(candidates), cv2.DIST_L2, 3
        )
        support_limit = max(2.0, 0.08 / resolution)
        accepted = []
        for center_x, center_y, radius in np.round(circles[0]).astype(int):
            angles = np.linspace(
                0, 2 * math.pi, max(48, int(round(2 * math.pi * radius))), endpoint=False
            )
            sample_x = np.clip(
                np.round(center_x + radius * np.cos(angles)).astype(int),
                0,
                candidates.shape[1] - 1,
            )
            sample_y = np.clip(
                np.round(center_y + radius * np.sin(angles)).astype(int),
                0,
                candidates.shape[0] - 1,
            )
            support = float(np.mean(distance[sample_y, sample_x] <= support_limit))
            safely_inside = bool(np.all(interior[sample_y, sample_x] > 0))
            if support >= 0.72 and safely_inside:
                accepted.append((float(center_x), float(center_y), float(radius)))
        return accepted

    @staticmethod
    def _is_duplicate_circle(
        center_x: float,
        center_y: float,
        radius: float,
        accepted: list[tuple[float, float, float]],
    ) -> bool:
        return any(
            math.hypot(center_x - saved_x, center_y - saved_y)
            < max(4.0, radius * 0.30)
            and abs(radius - saved_radius) < max(3.0, radius * 0.30)
            for saved_x, saved_y, saved_radius in accepted
        )

    @staticmethod
    def _draw_shape(
        canvas: np.ndarray,
        kind: str,
        geometry: object,
        scale: int,
        color: tuple[int, int, int],
        line_width: int,
    ) -> None:
        if kind == "circle":
            center_x, center_y, radius = geometry
            center = (int(round(center_x * scale)), int(round(center_y * scale)))
            scaled_radius = int(round(radius * scale))
            cv2.circle(canvas, center, scaled_radius, color, -1, cv2.LINE_AA)
            cv2.circle(
                canvas, center, scaled_radius, color, line_width, cv2.LINE_AA
            )
        elif kind == "ellipse":
            (center_x, center_y), axes, angle = geometry
            center = (int(round(center_x * scale)), int(round(center_y * scale)))
            scaled_axes = (
                int(round(axes[0] * scale / 2.0)),
                int(round(axes[1] * scale / 2.0)),
            )
            cv2.ellipse(
                canvas, center, scaled_axes, angle, 0, 360, color, -1, cv2.LINE_AA
            )
            cv2.ellipse(
                canvas,
                center,
                scaled_axes,
                angle,
                0,
                360,
                color,
                line_width,
                cv2.LINE_AA,
            )
        elif kind == "rectangle":
            points = np.rint(geometry * scale).astype(np.int32).reshape(-1, 1, 2)
            cv2.fillPoly(canvas, [points], color, lineType=cv2.LINE_AA)
            cv2.polylines(canvas, [points], True, color, line_width, cv2.LINE_AA)
        elif kind == "line":
            start, end = geometry
            cv2.line(
                canvas,
                tuple(np.rint(start * scale).astype(int)),
                tuple(np.rint(end * scale).astype(int)),
                color,
                line_width,
                cv2.LINE_AA,
            )
        else:
            points = np.rint(geometry * scale).astype(np.int32).reshape(-1, 1, 2)
            cv2.fillPoly(canvas, [points], color, lineType=cv2.LINE_AA)
            cv2.polylines(canvas, [points], True, color, line_width, cv2.LINE_AA)
