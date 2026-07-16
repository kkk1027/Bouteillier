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

    def test_preserves_continuous_walls_and_joins_short_gap(self):
        obstacles = np.zeros((80, 100), dtype=np.uint8)
        cv2.line(obstacles, (10, 40), (34, 40), 255, 1)
        cv2.line(obstacles, (38, 40), (90, 40), 255, 1)
        cleaned, _ = self.renderer._clean_obstacles(obstacles, 0.05)
        stats = MapRenderStats()

        walls = self.renderer._draw_vector_walls(cleaned, 0.05, 2, stats)

        self.assertGreater(cv2.countNonZero(walls[76:85, 20:184]), 0)
        self.assertGreater(cv2.countNonZero(walls[76:85, 64:80]), 0)

    def test_rebuilds_round_obstacle_as_standard_circle(self):
        grid = np.full((100, 100), -1, dtype=np.int8)
        grid[5:95, 5:95] = 0
        cv2.circle(grid, (50, 50), 12, 100, -1)

        _, info = self.renderer.render(grid.ravel(), 100, 100, 0.05)

        self.assertEqual(info["regularized_shapes"], 1)


if __name__ == "__main__":
    unittest.main()
