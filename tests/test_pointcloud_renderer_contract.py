from pathlib import Path
import re
import unittest


REPO_ROOT = Path(__file__).parents[1]
SWIFT_RENDERER = (
    REPO_ROOT / "Tracking Streamer" / "PointCloudRenderer.swift"
).read_text(encoding="utf-8")
METAL_SHADER = (
    REPO_ROOT / "Tracking Streamer" / "PointCloudShaders.metal"
).read_text(encoding="utf-8")
FRANKA_CONFIG = (
    REPO_ROOT
    / "ros2_ws"
    / "src"
    / "teleoperation"
    / "config"
    / "franka_teleoperation.yaml"
).read_text(encoding="utf-8")


def swift_integer_constant(name: str) -> int:
    match = re.search(rf"\b{name}\s*=\s*([\d_]+)", SWIFT_RENDERER)
    if match is None:
        raise AssertionError(f"Missing Swift constant: {name}")
    return int(match.group(1).replace("_", ""))


class PointCloudRendererContractTests(unittest.TestCase):
    def test_octagonal_mesh_layout_matches_metal_shader(self):
        self.assertEqual(swift_integer_constant("verticesPerPoint"), 9)
        self.assertEqual(swift_integer_constant("indicesPerPoint"), 24)
        self.assertIn("for segment in 0..<8", SWIFT_RENDERER)
        self.assertIn("pointIndex * 9u", METAL_SHADER)
        self.assertIn("perimeterIndex < 8u", METAL_SHADER)

    def test_adaptive_density_levels_are_full_half_and_quarter(self):
        self.assertIn("private(set) var sourceStride = 1", SWIFT_RENDERER)
        self.assertIn("sourceStride < 4", SWIFT_RENDERER)
        self.assertIn("sourceStride *= 2", SWIFT_RENDERER)
        self.assertIn("sourceStride /= 2", SWIFT_RENDERER)
        self.assertIn("measured > 0.012", SWIFT_RENDERER)
        self.assertIn("measured < 0.006", SWIFT_RENDERER)

    def test_swift_and_metal_buffer_bindings_match(self):
        for index in (5, 6, 7):
            self.assertIn(f"index: {index}", SWIFT_RENDERER)
            self.assertIn(f"[[buffer({index})]]", METAL_SHADER)
        self.assertIn("sourcePointIndex = pointIndex * sourceStride", METAL_SHADER)

    def test_franka_uses_full_decimated_cloud_at_ten_hertz(self):
        self.assertRegex(FRANKA_CONFIG, r"pointcloud_rate_hz:\s*10\.0")
        self.assertRegex(FRANKA_CONFIG, r"pointcloud_max_points:\s*20000")
        self.assertRegex(FRANKA_CONFIG, r"pointcloud_stride:\s*1(?:\s|$)")


if __name__ == "__main__":
    unittest.main()
