#!/usr/bin/env python3
"""Regression tests for Gazebo sprayer model marking limits."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
import importlib.util
import math
from pathlib import Path


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


if __name__ == "__main__":
    unittest.main()
