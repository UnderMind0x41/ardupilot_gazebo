"""Static contract checks for the PX4 Swan K1 to ArduPilot port."""

from __future__ import annotations

import hashlib
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class QuadTailsitterPortTest(unittest.TestCase):
    def test_source_meshes_are_unchanged(self) -> None:
        expected = {
            "body.dae": "72934cd4119af69cabc22de79f992adac1e350bec39bd4c250e5d757529842ab",
            "iris_prop_ccw.dae": "5f8b01668ee24a5b663ca8c4dd56e294fd6480db7eec1e5a7c11e45ba9004e99",
            "iris_prop_cw.dae": "6cbc686772dccd46253fb65ece000e7ffa6a74b9c097bda349884bd1e78cd879",
        }
        mesh_dir = ROOT / "models/quadtailsitter/meshes"
        for name, digest in expected.items():
            actual = hashlib.sha256((mesh_dir / name).read_bytes()).hexdigest()
            self.assertEqual(digest, actual, name)

    def test_source_physics_and_motor_contract(self) -> None:
        root = ET.parse(ROOT / "models/quadtailsitter/model.sdf").getroot()
        model = root.find("model")
        self.assertIsNotNone(model)
        self.assertEqual("1.6", model.findtext("link/inertial/mass"))

        motors = [
            plugin
            for plugin in model.findall("plugin")
            if plugin.attrib.get("filename") == "gz-sim-multicopter-motor-model-system"
        ]
        self.assertEqual(4, len(motors))
        self.assertEqual(["0", "1", "2", "3"], [
            plugin.findtext("actuator_number") for plugin in motors
        ])
        self.assertEqual(
            ["ccw", "ccw", "cw", "cw"],
            [plugin.findtext("turningDirection") for plugin in motors],
        )
        self.assertTrue(all(
            plugin.findtext("commandSubTopic") == "command/motor_speed"
            for plugin in motors
        ))
        self.assertTrue(all(
            plugin.findtext("motorConstant") == "8.54858e-06" for plugin in motors
        ))

        lift_plugins = [
            plugin
            for plugin in model.findall("plugin")
            if plugin.attrib.get("filename") == "gz-sim-advanced-lift-drag-system"
        ]
        self.assertEqual(1, len(lift_plugins))
        self.assertEqual("base_link", lift_plugins[0].findtext("link_name"))

    def test_ardupilot_wrapper_maps_quad_x_to_one_actuator_vector(self) -> None:
        root = ET.parse(
            ROOT / "models/quadtailsitter_with_ardupilot/model.sdf"
        ).getroot()
        plugin = root.find("model/plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(
            "0 0 0 180 -90 0",
            plugin.findtext("modelXYZToAirplaneXForwardZDown"),
        )
        controls = plugin.findall("control")
        self.assertEqual(["4", "5", "6", "7"], [
            control.attrib["channel"] for control in controls
        ])
        self.assertEqual(["0", "1", "2", "3"], [
            control.findtext("actuator_index") for control in controls
        ])
        self.assertTrue(all(control.findtext("type") == "ACTUATOR" for control in controls))
        self.assertTrue(all(control.findtext("multiplier") == "1200" for control in controls))
        self.assertEqual(1, len({
            control.findtext("cmd_topic") for control in controls
        }))

    def test_parameter_profile_selects_motor_only_tailsitter(self) -> None:
        params: dict[str, str] = {}
        for line in (ROOT / "config/gazebo-quadtailsitter.parm").read_text().splitlines():
            content = line.split("#", 1)[0].strip()
            if content:
                key, value = content.split()
                params[key] = value
        self.assertEqual("1", params["Q_ENABLE"])
        self.assertEqual("2", params["Q_TAILSIT_ENABLE"])
        self.assertEqual("1", params["Q_ACRO_RY_SWAP"])
        self.assertEqual("1", params["Q_FRAME_CLASS"])
        self.assertEqual("1", params["Q_FRAME_TYPE"])
        self.assertEqual(
            ["33", "34", "35", "36"],
            [params[f"SERVO{channel}_FUNCTION"] for channel in range(5, 9)],
        )
        self.assertEqual("1", params["SIM_IMU_COUNT"])


if __name__ == "__main__":
    unittest.main()
