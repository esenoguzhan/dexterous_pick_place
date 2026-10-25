"""Geometry-only tests: no ROS nodes, MuJoCo imports or physics execution."""

import math
from pathlib import Path
import tempfile
import unittest

from lara5_mujoco_sim.obstacles import load_obstacles, obstacle_geometry


class StaticObstacleTests(unittest.TestCase):
    def test_full_lengths_convert_to_mujoco_sizes(self):
        box = obstacle_geometry({"name": "box", "size": [0.2, 0.4, 0.6], "rpy": [0, 0, math.pi/2]})
        self.assertEqual(list(map(float, box["size"].split())), [0.1, 0.2, 0.3])
        quaternion = list(map(float, box["quat"].split()))
        self.assertAlmostEqual(quaternion[0], math.sqrt(0.5))
        self.assertAlmostEqual(quaternion[3], math.sqrt(0.5))
        cylinder = obstacle_geometry({"name": "post", "shape": "cylinder", "size": [0.05, 0.7]})
        self.assertEqual(list(map(float, cylinder["size"].split())), [0.05, 0.35])
        sphere = obstacle_geometry({"name": "sphere", "shape": "sphere", "size": [0.08]})
        self.assertEqual(float(sphere["size"]), 0.08)

    def test_invalid_geometry_and_duplicate_names_rejected(self):
        for obstacle in ({"name": "bad", "size": [-1, 1, 1]},
                         {"name": "bad", "size": [1, 1, 1], "position": [math.nan, 0, 0]},
                         {"name": "bad", "size": [1, 1, 1], "mujoco": "false"}):
            with self.assertRaises(ValueError):
                obstacle_geometry(obstacle)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.yaml"
            path.write_text("obstacles:\n  - {name: a, size: [1, 1, 1]}\n  - {name: a, size: [1, 1, 1]}\n")
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                load_obstacles(path)

    def test_demo_dividers_removed_and_existing_floor_retained(self):
        path = Path(__file__).resolve().parents[2] / "lara5_sim_approach/config/obstacle_demo.yaml"
        obstacles = load_obstacles(path)
        self.assertEqual({item["name"] for item in obstacles},
                         {"floor"})
        self.assertIsNone(obstacle_geometry(obstacles[0]))


if __name__ == "__main__":
    unittest.main()
