"""Tests for portable occupancy-map artifacts."""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from autopatrol_robot.map_artifact import (
    OccupancyMapArtifactProcessor,
    export_nav2_map,
    load_nav2_map,
)


class TestMapArtifactProcessor(unittest.TestCase):
    def setUp(self):
        self.processor = OccupancyMapArtifactProcessor()

    @staticmethod
    def _sample_grid():
        grid = np.full((100, 140), -1, dtype=np.int8)
        grid[6:94, 6:134] = 0
        cv2.rectangle(grid, (12, 12), (127, 88), 100, 1)
        cv2.circle(grid, (68, 48), 8, 100, 1)
        return grid

    def test_creates_png_svg_and_canvas_manifest(self):
        grid = self._sample_grid()

        artifact = self.processor.process(
            grid.ravel(), 140, 100, 0.05, -2.0, -1.0
        )

        self.assertTrue(artifact.png_bytes.startswith(b"\x89PNG"))
        self.assertIn("<svg", artifact.svg)
        self.assertEqual(
            artifact.manifest["schema"], "autopatrol.map-artifact/v1"
        )
        self.assertEqual(artifact.manifest["map"]["origin"]["x"], -2.0)
        self.assertEqual(artifact.manifest["render"]["background"], "#0a0e1a")
        image = cv2.imdecode(
            np.frombuffer(artifact.png_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        self.assertEqual(tuple(int(value) for value in image[0, 0]), (26, 14, 10))
        self.assertGreater(
            len(artifact.manifest["layers"]["walls"]["polygons"]), 0
        )

    def test_exports_all_cross_platform_assets(self):
        grid = self._sample_grid()
        artifact = self.processor.process(grid.ravel(), 140, 100, 0.05)

        with tempfile.TemporaryDirectory() as temporary_dir:
            paths = self.processor.export(
                artifact, temporary_dir, "my map / version 1"
            )
            self.assertEqual(set(paths), {"png", "svg", "json"})
            self.assertTrue(all(path.is_file() for path in paths.values()))
            self.assertEqual(paths["json"].name, "my_map_version_1.json")
            manifest = json.loads(paths["json"].read_text(encoding="utf-8"))
            self.assertEqual(manifest["assets"]["png"], "my_map_version_1.png")

    def test_loads_standard_nav2_yaml_and_uses_same_export_pipeline(self):
        image = np.full((60, 80), 205, dtype=np.uint8)
        image[5:55, 5:75] = 254
        cv2.rectangle(image, (5, 5), (74, 54), 0, 1)

        with tempfile.TemporaryDirectory() as temporary_dir:
            directory = Path(temporary_dir)
            image_path = directory / "sample.pgm"
            yaml_path = directory / "sample.yaml"
            cv2.imwrite(str(image_path), image)
            yaml_path.write_text(
                "\n".join(
                    [
                        "image: sample.pgm",
                        "mode: trinary",
                        "resolution: 0.05",
                        "origin: [-2.0, -1.5, 0.0]",
                        "negate: 0",
                        "occupied_thresh: 0.65",
                        "free_thresh: 0.25",
                    ]
                ),
                encoding="utf-8",
            )

            source = load_nav2_map(yaml_path)
            self.assertEqual((source.width, source.height), (80, 60))
            self.assertEqual(source.origin_y, -1.5)
            self.assertEqual(int(source.data.reshape(60, 80)[0, 0]), -1)

            paths = export_nav2_map(yaml_path, directory / "output")
            self.assertTrue(paths["png"].is_file())
            self.assertTrue(paths["svg"].is_file())
            self.assertTrue(paths["json"].is_file())


if __name__ == "__main__":
    unittest.main()
