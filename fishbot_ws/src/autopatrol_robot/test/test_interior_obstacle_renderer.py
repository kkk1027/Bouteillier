"""Regression tests for the isolated interior-obstacle rendering layer."""
import unittest

import cv2
import numpy as np

from autopatrol_robot.interior_obstacle_renderer import InteriorObstacleRenderer


class TestInteriorObstacleRenderer(unittest.TestCase):
    def setUp(self):
        self.renderer = InteriorObstacleRenderer()
        self.floor = np.zeros((140, 180), dtype=np.uint8)
        self.floor[10:130, 10:170] = 255
        self.canvas = np.zeros((280, 360, 3), dtype=np.uint8)

    def test_rebuilds_circle_rectangle_and_joined_line_inside_body(self):
        obstacles = np.zeros_like(self.floor)
        cv2.circle(obstacles, (48, 48), 9, 255, 1)
        cv2.rectangle(obstacles, (88, 37), (112, 61), 255, 1)
        cv2.line(obstacles, (45, 94), (74, 94), 255, 1)
        cv2.line(obstacles, (77, 94), (126, 94), 255, 1)

        stats = self.renderer.draw(
            obstacles,
            self.floor,
            0.05,
            2,
            self.canvas,
            (49, 76, 76),
            3,
        )

        self.assertGreaterEqual(stats.circles, 1)
        self.assertGreaterEqual(stats.rectangles, 1)
        self.assertGreaterEqual(stats.lines, 1)
        self.assertGreaterEqual(stats.filled_closed_shapes, 2)
        self.assertTrue(np.array_equal(self.canvas[96, 96], (49, 76, 76)))
        self.assertTrue(np.array_equal(self.canvas[98, 200], (49, 76, 76)))
        self.assertGreater(int(np.count_nonzero(self.canvas[76:120, 76:120])), 0)
        self.assertGreater(int(np.count_nonzero(self.canvas[170:208, 80:260])), 0)

    def test_drops_noise_and_keeps_clear_of_the_exterior_boundary(self):
        obstacles = np.zeros_like(self.floor)
        cv2.circle(obstacles, (15, 24), 5, 255, 1)
        obstacles[72, 90] = 255
        cv2.circle(obstacles, (90, 72), 8, 255, 1)

        stats = self.renderer.draw(
            obstacles,
            self.floor,
            0.05,
            2,
            self.canvas,
            (49, 76, 76),
            3,
        )

        self.assertGreaterEqual(stats.removed_components, 1)
        self.assertEqual(int(np.count_nonzero(self.canvas[0:60, 0:60])), 0)
        self.assertGreater(int(np.count_nonzero(self.canvas[120:170, 150:220])), 0)


if __name__ == "__main__":
    unittest.main()
