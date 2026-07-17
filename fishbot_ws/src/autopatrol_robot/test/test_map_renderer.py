"""Tests for conservative display-only occupancy-map cleanup."""

import base64
import unittest

import cv2
import numpy as np

from autopatrol_robot.map_renderer import MapRenderStats, OccupancyMapRenderer


class TestOccupancyMapRenderer(unittest.TestCase):
    def setUp(self):
        self.renderer = OccupancyMapRenderer()

    def test_removes_short_island_and_closes_short_wall_gap(self):
        obstacles = np.zeros((80, 80), dtype=np.uint8)
        cv2.line(obstacles, (10, 20), (32, 20), 255, 1)
        cv2.line(obstacles, (36, 20), (70, 20), 255, 1)
        obstacles[50, 50] = 255

        cleaned, stats = self.renderer._clean_obstacles(obstacles, 0.05)

        self.assertGreaterEqual(stats.removed_components, 1)
        self.assertGreater(cv2.countNonZero(cleaned[19:22, 32:37]), 0)

    def test_renders_scaled_png_and_map_metadata(self):
        grid = np.full((60, 90), -1, dtype=np.int8)
        grid[5:55, 5:85] = 0
        cv2.rectangle(grid, (5, 5), (84, 54), 100, 1)

        encoded, info = self.renderer.render(grid.ravel(), 90, 60, 0.05)
        png = base64.b64decode(encoded)
        image = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)

        self.assertEqual(image.shape[:2], (120, 180))
        self.assertEqual(info["source_width"], 90)
        self.assertEqual(info["source_height"], 60)
        self.assertEqual(info["output_width"], 180)
        self.assertEqual(info["output_height"], 120)

    def test_primary_floor_mask_has_no_antialiased_edge_values(self):
        grid = np.full((80, 120), -1, dtype=np.int8)
        grid[10:70, 15:105] = 0
        free = (grid == 0).astype(np.uint8) * 255
        stats = MapRenderStats()

        floor, _ = self.renderer._build_primary_floor_mask(free, 0.05, stats)

        self.assertTrue(set(np.unique(floor)).issubset({0, 255}))

    def test_preserves_continuous_walls_and_joins_short_gap(self):
        obstacles = np.zeros((80, 100), dtype=np.uint8)
        cv2.line(obstacles, (10, 40), (34, 40), 255, 1)
        cv2.line(obstacles, (38, 40), (90, 40), 255, 1)
        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)
        stats = MapRenderStats()

        walls = self.renderer._draw_vector_walls(cleaned, 0.05, 2, stats)

        self.assertGreater(cv2.countNonZero(walls[76:85, 20:184]), 0)
        self.assertGreater(cv2.countNonZero(walls[76:85, 64:80]), 0)
        components, _ = cv2.connectedComponents(
            (walls[70:90, 10:190] > 0).astype(np.uint8)
        )
        self.assertEqual(components - 1, 1)

    def test_joins_detected_wall_lines_across_large_gap(self):
        obstacles = np.zeros((80, 120), dtype=np.uint8)
        cv2.line(obstacles, (10, 40), (45, 40), 255, 1)
        cv2.line(obstacles, (58, 40), (110, 40), 255, 1)
        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)
        stats = MapRenderStats()

        walls = self.renderer._draw_vector_walls(cleaned, 0.05, 2, stats)
        components, _ = cv2.connectedComponents((walls > 0).astype(np.uint8))

        self.assertGreaterEqual(stats.bridged_gaps, 1)
        self.assertEqual(components - 1, 1)

    def test_closes_axis_aligned_open_wall_gap(self):
        canvas = np.zeros((180, 180), dtype=np.uint8)
        lines = [(20, 20, 20, 80), (23, 150, 23, 90)]

        closed, _ = self.renderer._close_open_wall_gaps(
            canvas, lines, 0.05, 1, 2
        )

        self.assertEqual(closed, 1)
        self.assertGreater(cv2.countNonZero(canvas[78:93, 18:28]), 0)

    def test_does_not_close_diagonal_open_wall_gap(self):
        canvas = np.zeros((180, 180), dtype=np.uint8)
        lines = [(20, 20, 20, 80), (60, 100, 60, 160)]

        closed, _ = self.renderer._close_open_wall_gaps(
            canvas, lines, 0.05, 1, 2)

        self.assertEqual(closed, 0)

    def test_rebuilds_damaged_round_internal_obstacle(self):
        grid = np.full((100, 100), -1, dtype=np.int8)
        grid[5:95, 5:95] = 0
        cv2.circle(grid, (50, 50), 5, 100, 1)
        grid[45, 49:52] = 0

        _, info = self.renderer.render(grid.ravel(), 100, 100, 0.05)

        self.assertGreaterEqual(info["interior_circles"], 1)

    def test_rebuilds_closed_principal_contour_and_fills_internal_holes(self):
        grid = np.full((120, 160), -1, dtype=np.int8)
        grid[15:105, 20:145] = 0
        # This black notch reaches the exterior, so it must remain a concavity.
        grid[15:58, 92:114] = -1
        # This obstacle is internal and is intentionally filled in this pass.
        cv2.circle(grid, (62, 74), 7, 100, -1)

        encoded, info = self.renderer.render(grid.ravel(), 160, 120, 0.05)
        image = cv2.imdecode(
            np.frombuffer(base64.b64decode(encoded), dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )

        self.assertTrue(info["primary_contour_closed"])
        self.assertEqual(info["closed_wall_gaps"], 1)
        self.assertGreaterEqual(info["ignored_interior_obstacles"], 1)
        # Closed internal obstacles are now filled with the wall color.
        self.assertLess(int(image[74 * 2, 62 * 2].mean()), 100)
        # The exterior-connected notch remains black.
        self.assertLess(int(image[32 * 2, 102 * 2].mean()), 100)

    def test_inferrs_body_from_walls_when_full_grid_is_free(self):
        grid = np.zeros((120, 160), dtype=np.int8)
        cv2.rectangle(grid, (30, 20), (130, 100), 100, 1)
        # Simulate typical mapping breaks in an otherwise free background.
        grid[20, 74:78] = 0
        grid[57:62, 30] = 0
        cv2.circle(grid, (78, 62), 6, 100, 1)

        encoded, info = self.renderer.render(grid.ravel(), 160, 120, 0.05)
        image = cv2.imdecode(
            np.frombuffer(base64.b64decode(encoded), dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )

        self.assertEqual(info["boundary_mode"], "wall-inference")
        self.assertTrue(info["primary_contour_closed"])
        self.assertGreater(info["wall_inference_radius_m"], 0)
        # The enclosing rectangle is recovered and its closed internal
        # obstacle is filled by the independent interior layer.
        self.assertLess(int(image[62 * 2, 78 * 2].mean()), 100)
        self.assertLess(int(image[8 * 2, 8 * 2].mean()), 100)

    def test_removes_nearby_parallel_duplicate_wall_stroke(self):
        lines = [
            (80, 20, 80, 130),
            (87, 42, 87, 126),
            (20, 142, 64, 142),
        ]

        kept, removed = self.renderer._remove_parallel_duplicates(lines, 0.05)

        self.assertEqual(removed, 1)
        self.assertIn((80, 20, 80, 130), kept)
        self.assertNotIn((87, 42, 87, 126), kept)
        self.assertIn((20, 142, 64, 142), kept)

    def test_snaps_offset_perpendicular_walls_to_a_clean_corner(self):
        obstacles = np.zeros((140, 160), dtype=np.uint8)
        cv2.line(obstacles, (30, 86), (30, 34), 255, 1)
        cv2.line(obstacles, (46, 20), (120, 20), 255, 1)
        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)
        stats = MapRenderStats()

        walls = self.renderer._draw_vector_walls(cleaned, 0.05, 2, stats)
        components, _ = cv2.connectedComponents((walls > 0).astype(np.uint8))

        # The two detected strokes must meet at the geometric corner (30, 20),
        # not through a diagonal patch between their nearest noisy endpoints.
        self.assertEqual(components - 1, 1)
        self.assertGreater(cv2.countNonZero(walls[36:45, 56:68]), 0)
        self.assertGreaterEqual(stats.bridged_gaps, 1)

    def test_redraws_noisy_wall_as_single_straight_stroke(self):
        obstacles = np.zeros((120, 160), dtype=np.uint8)
        points = np.array(
            [
                (x, 60 + (3 if (x // 4) % 2 else -3))
                for x in range(20, 141, 4)
            ],
            dtype=np.int32,
        )
        cv2.polylines(obstacles, [points], False, 255, 1)
        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)
        stats = MapRenderStats()

        walls = self.renderer._draw_vector_walls(cleaned, 0.05, 2, stats)
        y_coordinates, _ = np.where(walls > 0)

        self.assertEqual(stats.vector_segments, 1)
        self.assertLessEqual(
            int(y_coordinates.max() - y_coordinates.min() + 1), 10
        )

    def test_removes_isolated_lines_outside_main_map_body(self):
        obstacles = np.zeros((150, 170), dtype=np.uint8)
        cv2.rectangle(obstacles, (40, 25), (145, 125), 255, 1)
        cv2.line(obstacles, (5, 8), (26, 8), 255, 1)
        cv2.line(obstacles, (9, 135), (9, 148), 255, 1)

        cleaned, stats = self.renderer._clean_obstacles(obstacles, 0.05)

        self.assertGreaterEqual(stats.removed_components, 2)
        self.assertEqual(cv2.countNonZero(cleaned[0:20, 0:32]), 0)
        self.assertEqual(cv2.countNonZero(cleaned[128:150, 0:20]), 0)
        self.assertGreater(cv2.countNonZero(cleaned[24:127, 39:147]), 0)

    def test_removes_fragment_outside_hull_but_keeps_inner_obstacle(self):
        obstacles = np.zeros((150, 150), dtype=np.uint8)
        cv2.line(obstacles, (30, 20), (30, 125), 255, 1)
        cv2.line(obstacles, (30, 125), (125, 125), 255, 1)
        cv2.line(obstacles, (108, 28), (108, 44), 255, 1)
        cv2.circle(obstacles, (58, 92), 3, 255, 1)

        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)

        self.assertEqual(cv2.countNonZero(cleaned[24:48, 104:112]), 0)
        self.assertGreater(cv2.countNonZero(cleaned[88:97, 54:63]), 0)

    def test_retains_short_segment_that_joins_two_body_edges(self):
        obstacles = np.zeros((100, 160), dtype=np.uint8)
        cv2.line(obstacles, (20, 50), (70, 50), 255, 1)
        cv2.line(obstacles, (90, 50), (140, 50), 255, 1)
        cv2.line(obstacles, (74, 50), (86, 50), 255, 1)

        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)

        self.assertGreater(cv2.countNonZero(cleaned[49:52, 73:88]), 0)

    def test_rebuilds_square_as_rectangle_not_circle(self):
        grid = np.full((100, 100), -1, dtype=np.int8)
        grid[5:95, 5:95] = 0
        cv2.rectangle(grid, (38, 38), (62, 62), 100, -1)

        _, info = self.renderer.render(grid.ravel(), 100, 100, 0.05)

        self.assertEqual(info["interior_circles"], 0)
        self.assertGreaterEqual(info["interior_rectangles"], 1)

    def test_rebuilds_ellipse_internal_obstacle(self):
        grid = np.full((100, 100), -1, dtype=np.int8)
        grid[5:95, 5:95] = 0
        cv2.ellipse(grid, (50, 50), (12, 10), 0, 0, 360, 100, -1)

        _, info = self.renderer.render(grid.ravel(), 100, 100, 0.05)

        self.assertEqual(info["interior_circles"], 0)
        self.assertGreaterEqual(info["interior_ellipses"], 1)

    def test_rebuilds_round_internal_obstacle(self):
        grid = np.full((100, 100), -1, dtype=np.int8)
        grid[5:95, 5:95] = 0
        cv2.circle(grid, (50, 50), 12, 100, -1)

        _, info = self.renderer.render(grid.ravel(), 100, 100, 0.05)

        self.assertGreaterEqual(info["interior_circles"], 1)


if __name__ == "__main__":
    unittest.main()
