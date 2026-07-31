"""Contracts for the generated stock-Iris ArduPlane tailsitter fixture."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "scripts/generate_iris_tailsitter_fixture.py"
MODEL_NAME = "iris_tailsitter_with_ardupilot"


def load_params(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    for line in path.read_text().splitlines():
        content = line.split("#", 1)[0].strip()
        if content:
            key, value = content.split()
            params[key] = value
    return params


class IrisTailsitterFixtureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.output = Path(self.tempdir.name)
        subprocess.run(
            [
                sys.executable,
                str(GENERATOR),
                "--output-dir",
                str(self.output),
            ],
            check=True,
            capture_output=True,
            text=True,
        )

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def test_stock_iris_plugins_and_motor_contract_are_preserved(self) -> None:
        source_model = ET.parse(
            ROOT / "models/iris_with_ardupilot/model.sdf"
        ).getroot().find("model")
        generated_model = ET.parse(
            self.output / f"models/{MODEL_NAME}/model.sdf"
        ).getroot().find("model")
        self.assertIsNotNone(source_model)
        self.assertIsNotNone(generated_model)

        source_plugins = [
            plugin.get("filename")
            for plugin in source_model.findall("plugin")
            if plugin.get("name") != "ArduPilotPlugin"
        ]
        generated_plugins = [
            plugin.get("filename")
            for plugin in generated_model.findall("plugin")
            if plugin.get("name") != "ArduPilotPlugin"
        ]
        self.assertEqual(source_plugins, generated_plugins)

        source_controls = source_model.find(
            "plugin[@name='ArduPilotPlugin']"
        ).findall("control")
        generated_controls = generated_model.find(
            "plugin[@name='ArduPilotPlugin']"
        ).findall("control")
        self.assertEqual(
            [control.findtext("jointName") for control in source_controls],
            [control.findtext("jointName") for control in generated_controls],
        )
        self.assertEqual(
            [control.findtext("multiplier") for control in source_controls],
            [control.findtext("multiplier") for control in generated_controls],
        )
        self.assertEqual(
            ["4", "5", "6", "7"],
            [control.get("channel") for control in generated_controls],
        )

    def test_aircraft_frame_and_imu_are_rotated_together(self) -> None:
        model = ET.parse(
            self.output / f"models/{MODEL_NAME}/model.sdf"
        ).getroot().find("model")
        plugin = model.find("plugin[@name='ArduPilotPlugin']")
        self.assertEqual(
            "0 0 0 180 -90 0",
            plugin.findtext("modelXYZToAirplaneXForwardZDown"),
        )
        self.assertEqual(
            "tailsitter_imu_link::tailsitter_imu_sensor",
            plugin.findtext("imuName"),
        )
        self.assertEqual(
            "0 0 0 180 -90 0",
            model.findtext(
                "link[@name='tailsitter_imu_link']/"
                "sensor[@name='tailsitter_imu_sensor']/pose"
            ),
        )
        marker = model.find(
            "link[@name='tailsitter_imu_link']/"
            "visual[@name='aircraft_forward_marker']"
        )
        self.assertIsNotNone(marker)
        self.assertIsNone(
            model.find("link[@name='tailsitter_imu_link']/collision")
        )

    def test_world_uses_generated_vehicle_and_profile_defaults_swap_off(self) -> None:
        world = ET.parse(
            self.output / "worlds/iris_tailsitter_swap_test.sdf"
        ).getroot().find("world")
        vehicle_uri = next(
            include.findtext("uri")
            for include in world.findall("include")
            if (include.findtext("uri") or "").startswith("model://iris_")
        )
        self.assertEqual(f"model://{MODEL_NAME}", vehicle_uri)

        params = load_params(
            ROOT / "config/gazebo-iris-tailsitter-test.parm"
        )
        self.assertEqual("1", params["Q_FRAME_CLASS"])
        self.assertEqual("1", params["Q_FRAME_TYPE"])
        self.assertEqual("2", params["Q_TAILSIT_ENABLE"])
        self.assertEqual("0", params["Q_ACRO_RY_SWAP"])
        self.assertEqual(
            ["33", "34", "35", "36"],
            [params[f"SERVO{channel}_FUNCTION"] for channel in range(5, 9)],
        )


if __name__ == "__main__":
    unittest.main()
