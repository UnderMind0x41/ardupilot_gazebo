#!/usr/bin/env python3
"""Regression tests for Gazebo sprayer model marking limits."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODELS_DIR = REPO_ROOT / "models"
MIN_GENERATED_FLEET_SPRAY_HEIGHT_MAX_M = 40.0


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


class SprayerModelConfigTests(unittest.TestCase):
    def test_source_sprayer_models_mark_generated_field_lanes(self) -> None:
        for model_name in ("iris_with_sprayer", "iris_with_sprayer_2"):
            with self.subTest(model_name=model_name):
                value = _spray_height_max(SOURCE_MODELS_DIR / model_name / "model.sdf")
                self.assertGreaterEqual(value, MIN_GENERATED_FLEET_SPRAY_HEIGHT_MAX_M)

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


if __name__ == "__main__":
    unittest.main()
