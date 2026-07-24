#!/usr/bin/env python3
"""Regression tests for Gazebo sprayer model marking limits."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODELS_DIR = REPO_ROOT / "models"
MIN_GENERATED_FLEET_SPRAY_HEIGHT_MAX_M = 40.0
DOWNWARD_CAMERA_HORIZONTAL_FOV_RAD = math.pi / 2


def _load_world_generator():
    script_path = REPO_ROOT / "scripts" / "generate_agrodrone_fleet_world.py"
    spec = importlib.util.spec_from_file_location("generate_agrodrone_fleet_world", script_path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"Cannot load {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _spray_height_max(model_sdf: Path) -> float:
    root = ET.parse(model_sdf).getroot()
    for plugin in root.findall(".//plugin"):
        if plugin.get("name") != "SprayerPlugin":
            continue
        value = plugin.findtext("spray_height_max")
        if value is None:
            raise AssertionError(f"{model_sdf} SprayerPlugin has no spray_height_max")
        return float(value)
    raise AssertionError(f"{model_sdf} has no SprayerPlugin")


def _downward_camera_horizontal_fov(model_sdf: Path) -> float:
    root = ET.parse(model_sdf).getroot()
    value = root.findtext(".//sensor[@name='downward_camera']/camera/horizontal_fov")
    if value is None:
        raise AssertionError(f"{model_sdf} has no downward camera horizontal_fov")
    return float(value)


class SprayerModelConfigTests(unittest.TestCase):
    def test_precision_land_profile_uses_damped_stationary_target_limits(self) -> None:
        parameters: dict[str, float] = {}
        profile = REPO_ROOT / "config" / "agro-precision-landing.parm"
        for line in profile.read_text(encoding="utf-8").splitlines():
            content = line.split("#", 1)[0].strip()
            if not content:
                continue
            name, value = content.split()[:2]
            parameters[name] = float(value)
        self.assertEqual(parameters["PLND_ACC_P_NSE"], 0.5)
        self.assertEqual(parameters["PLND_XY_VEL_MAX"], 80.0)
        self.assertEqual(parameters["PLND_XY_ACC_MAX"], 30.0)
        self.assertEqual(parameters["PLND_OPTIONS"], 4.0)

    def test_landing_contacts_use_bounded_correction_and_friction(self) -> None:
        for model_name in ("iris_with_standoffs", "stationary_landing_base"):
            with self.subTest(model_name=model_name):
                root = ET.parse(SOURCE_MODELS_DIR / model_name / "model.sdf").getroot()
                collisions = root.findall(".//collision")
                self.assertTrue(collisions)
                for collision in collisions:
                    surface = collision.find("surface")
                    if surface is None:
                        continue
                    self.assertEqual(surface.findtext("contact/ode/max_vel"), "0.01")
                    self.assertEqual(surface.findtext("friction/ode/mu"), "10.0")
                    self.assertEqual(surface.findtext("friction/ode/mu2"), "10.0")
        standoffs = ET.parse(
            SOURCE_MODELS_DIR / "iris_with_standoffs" / "model.sdf"
        ).getroot()
        for name in (
            "front_left_leg_collision",
            "front_right_leg_collision",
            "rear_left_leg_collision",
            "rear_right_leg_collision",
        ):
            self.assertEqual(
                standoffs.findtext(f".//collision[@name='{name}']/geometry/cylinder/radius"),
                "0.015",
            )

    def test_source_sprayer_models_mark_generated_field_lanes(self) -> None:
        for model_name in ("iris_with_sprayer", "iris_with_sprayer_2"):
            with self.subTest(model_name=model_name):
                value = _spray_height_max(SOURCE_MODELS_DIR / model_name / "model.sdf")
                self.assertGreaterEqual(value, MIN_GENERATED_FLEET_SPRAY_HEIGHT_MAX_M)

    def test_source_sprayer_models_use_90_degree_downward_camera_fov(self) -> None:
        for model_name in ("iris_with_sprayer", "iris_with_sprayer_2"):
            with self.subTest(model_name=model_name):
                value = _downward_camera_horizontal_fov(
                    SOURCE_MODELS_DIR / model_name / "model.sdf"
                )
                self.assertTrue(
                    math.isclose(
                        value,
                        DOWNWARD_CAMERA_HORIZONTAL_FOV_RAD,
                        rel_tol=0,
                        abs_tol=1e-11,
                    )
                )

    def test_generated_drone_variants_keep_spray_mark_height_limit(self) -> None:
        generator = _load_world_generator()
        with tempfile.TemporaryDirectory() as tmp:
            generated_models_dir = Path(tmp)
            generator._generate_drone_variant(
                drone_number=4,
                source_models_dir=SOURCE_MODELS_DIR,
                generated_models_dir=generated_models_dir,
            )

            value = _spray_height_max(
                generated_models_dir / "iris_with_sprayer_4" / "model.sdf"
            )
            self.assertGreaterEqual(value, MIN_GENERATED_FLEET_SPRAY_HEIGHT_MAX_M)

            camera_fov = _downward_camera_horizontal_fov(
                generated_models_dir / "iris_with_sprayer_4" / "model.sdf"
            )
            self.assertTrue(
                math.isclose(
                    camera_fov,
                    DOWNWARD_CAMERA_HORIZONTAL_FOV_RAD,
                    rel_tol=0,
                    abs_tol=1e-11,
                )
            )

    def test_generated_stationary_bases_have_matching_gps_and_tag_ids(self) -> None:
        generator = _load_world_generator()
        with tempfile.TemporaryDirectory() as tmp:
            generated_models_dir = Path(tmp)
            for base_number in (1, 2):
                generator._generate_stationary_base_variant(
                    base_number=base_number,
                    source_models_dir=SOURCE_MODELS_DIR,
                    generated_models_dir=generated_models_dir,
                )
                root = ET.parse(
                    generated_models_dir
                    / f"stationary_landing_base_{base_number}"
                    / "model.sdf"
                ).getroot()
                model = root.find("model")
                self.assertEqual(
                    model.get("name"),
                    f"stationary_landing_base_{base_number}",
                )
                self.assertEqual(model.findtext("include/uri"), f"model://apriltag_36h11_{base_number}")
                self.assertEqual(len(model.findall("link")), 1)
                self.assertIsNotNone(model.find("link/sensor[@type='navsat']"))

    def test_minimal_world_contains_only_required_fleet_assets(self) -> None:
        generator = _load_world_generator()
        with tempfile.TemporaryDirectory() as tmp:
            output_dir = Path(tmp)
            output = generator._generate_world(
                SimpleNamespace(
                    template_world=REPO_ROOT / "worlds/iris_minimal_two_bases.sdf",
                    world_name="minimal_test",
                    latitude_deg=-35.363333333,
                    longitude_deg=149.165222222,
                    elevation_m=584.0,
                    heading_deg=0.0,
                    physics_max_step_size=None,
                    disable_camera_sensors=False,
                    disable_gpu_lidar_sensors=False,
                    minimal_scenery=False,
                    num_drones=2,
                    num_bases=2,
                    generated_worlds_dir=output_dir,
                )
            )
            text = output.read_text(encoding="utf-8")
            world = ET.parse(output).getroot().find("world")
            names = [include.findtext("name") for include in world.findall("include")]
            poses = {
                include.findtext("name"): include.findtext("pose")
                for include in world.findall("include")
            }

            self.assertEqual(
                names,
                [
                    "iris_with_sprayer",
                    "iris_with_sprayer_2",
                    "stationary_landing_base_1",
                    "stationary_landing_base_2",
                ],
            )
            self.assertNotIn("landing_truck", text)
            self.assertNotIn("landing_trailer", text)
            self.assertNotIn("fuel.gazebosim.org", text)
            self.assertNotIn("crop_field", text)
            self.assertEqual(world.findtext("physics/max_step_size"), "0.004")
            self.assertEqual(poses["iris_with_sprayer"], "-1 -30 0.281 0 0 90")
            self.assertEqual(poses["iris_with_sprayer_2"], "-8.8 -30 0.281 0 0 90")


if __name__ == "__main__":
    unittest.main()
