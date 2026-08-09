from __future__ import annotations

import importlib.util
import threading
from pathlib import Path
import tempfile
import unittest

from avp_stream.streamer import VisionProStreamer


ROOT = Path(__file__).parents[1]
GENERATOR_PATH = ROOT / "scripts" / "generate_fr3_usdz.py"
SPEC = importlib.util.spec_from_file_location("generate_fr3_usdz", GENERATOR_PATH)
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)


class PackagedUSDZTests(unittest.TestCase):
    def test_packaged_fr3_asset_is_valid_and_has_pose_targets(self):
        generator.validate_usdz(
            ROOT
            / "ros2_ws/src/robot_description/franka_mujoco/fr3_robotiq_2f85.usdz"
        )

    def test_staging_copies_only_referenced_assets(self):
        xml = ROOT / "ros2_ws/src/robot_description/franka_mujoco/fr3_robotiq_2f85.xml"
        description = Path(
            "/home/oda/franka_ws/install/franka_description/share/franka_description"
        )
        if not description.is_dir():
            self.skipTest("franka_description install is not available")

        with tempfile.TemporaryDirectory() as temp:
            stage = Path(temp)
            staged_xml = generator.stage_scene(xml, description, stage)
            staged_text = staged_xml.read_text(encoding="utf-8")
            self.assertNotIn("${franka_description}", staged_text)
            self.assertTrue(
                (stage / "assets/franka_description/meshes/robot_arms/fr3/collision/link0.stl").is_file()
            )

    def test_direct_asset_bypasses_conversion_and_is_idempotent(self):
        asset = ROOT / "ros2_ws/src/robot_description/franka_mujoco/fr3_robotiq_2f85.usdz"
        streamer = object.__new__(VisionProStreamer)
        streamer._sim_config = {
            "attach_to": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "grpc_port": 50051,
            "usdz_path": str(asset),
        }
        streamer._cross_network_mode = False
        streamer._usdz_sent = False
        streamer._usdz_send_lock = threading.Lock()
        streamer._log = lambda *args, **kwargs: None
        sends = []
        streamer._send_usdz_data = lambda path, attach, port: sends.append(path) or True
        streamer._load_and_send_mujoco_scene = lambda *args: self.fail(
            "runtime conversion must not be called"
        )

        self.assertTrue(streamer._load_and_send_scene())
        self.assertTrue(streamer._load_and_send_scene())
        self.assertEqual(sends, [str(asset)])


if __name__ == "__main__":
    unittest.main()
