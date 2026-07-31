#!/usr/bin/env python3
"""Generate an isolated ArduPlane tailsitter fixture from the stock Iris.

The source Iris motor/aerodynamic model is intentionally left unchanged.  The
generated wrapper only:

* rotates the ArduPilot aircraft frame so the quad thrust axis is aircraft +X;
* provides an IMU with the matching aircraft-frame orientation;
* routes ArduPlane QuadPlane outputs SERVO5..SERVO8 to the four Iris motors.

Generated files belong under ardupilot_gazebo/build and are not source assets.
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


AIRCRAFT_FRAME_ROTATION_DEG = "0 0 0 180 -90 0"
MODEL_NAME = "iris_tailsitter_with_ardupilot"


def require(parent: ET.Element, path: str) -> ET.Element:
    element = parent.find(path)
    if element is None:
        raise RuntimeError(f"missing expected stock Iris element: {path}")
    return element


def add_tailsitter_imu_and_marker(model: ET.Element) -> None:
    link = ET.Element("link", {"name": "tailsitter_imu_link"})
    ET.SubElement(link, "pose").text = "0 0 0 0 0 0"

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = "0.001"
    inertia = ET.SubElement(inertial, "inertia")
    for name, value in (
        ("ixx", "1e-7"),
        ("ixy", "0"),
        ("ixz", "0"),
        ("iyy", "1e-7"),
        ("iyz", "0"),
        ("izz", "1e-7"),
    ):
        ET.SubElement(inertia, name).text = value

    # A red mast makes aircraft +X visible in the GUI.  It has no collision,
    # so the stock Iris physics remain the control fixture under test.
    visual = ET.SubElement(link, "visual", {"name": "aircraft_forward_marker"})
    ET.SubElement(visual, "pose").text = "0 0 0.35 0 0 0"
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = "0.012"
    ET.SubElement(cylinder, "length").text = "0.7"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "1 0.02 0.02 1"
    ET.SubElement(material, "diffuse").text = "1 0.02 0.02 1"
    ET.SubElement(material, "emissive").text = "0.4 0 0 1"

    sensor = ET.SubElement(
        link,
        "sensor",
        {"name": "tailsitter_imu_sensor", "type": "imu"},
    )
    ET.SubElement(sensor, "gz_frame_id").text = "tailsitter_imu_link"
    sensor_pose = ET.SubElement(sensor, "pose", {"degrees": "true"})
    sensor_pose.text = AIRCRAFT_FRAME_ROTATION_DEG
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = "1000.0"

    joint = ET.Element(
        "joint",
        {"name": "tailsitter_imu_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "parent").text = "iris_with_standoffs::base_link"
    ET.SubElement(joint, "child").text = "tailsitter_imu_link"

    plugin_index = next(
        (
            index
            for index, child in enumerate(model)
            if child.tag == "plugin" and child.get("name") == "ArduPilotPlugin"
        ),
        len(model),
    )
    model.insert(plugin_index, link)
    model.insert(plugin_index + 1, joint)


def generate_model(source_model: Path, destination: Path) -> None:
    tree = ET.parse(source_model)
    root = tree.getroot()
    model = require(root, "model")
    model.set("name", MODEL_NAME)

    plugin = next(
        (
            child
            for child in model.findall("plugin")
            if child.get("name") == "ArduPilotPlugin"
        ),
        None,
    )
    if plugin is None:
        raise RuntimeError("stock Iris wrapper has no ArduPilotPlugin")

    require(plugin, "modelXYZToAirplaneXForwardZDown").text = (
        AIRCRAFT_FRAME_ROTATION_DEG
    )
    require(plugin, "imuName").text = (
        "tailsitter_imu_link::tailsitter_imu_sensor"
    )

    controls = plugin.findall("control")
    if len(controls) != 4:
        raise RuntimeError(
            f"expected four stock Iris motor controls, found {len(controls)}"
        )
    for output_index, control in enumerate(controls, start=4):
        control.set("channel", str(output_index))

    add_tailsitter_imu_and_marker(model)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def generate_world(source_world: Path, destination: Path) -> None:
    tree = ET.parse(source_world)
    root = tree.getroot()
    world = require(root, "world")
    world.set("name", "iris_tailsitter_swap_test")

    vehicle_include = next(
        (
            include
            for include in world.findall("include")
            if (include.findtext("uri") or "").startswith("model://iris_")
        ),
        None,
    )
    if vehicle_include is None:
        raise RuntimeError("stock Iris world has no Iris model include")
    require(vehicle_include, "uri").text = f"model://{MODEL_NAME}"
    require(vehicle_include, "pose").text = "0 0 0.195 0 0 90"

    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def write_model_config(destination: Path) -> None:
    root = ET.Element("model")
    ET.SubElement(root, "name").text = "Iris Quad Tailsitter Control Fixture"
    ET.SubElement(root, "version").text = "1.0"
    sdf = ET.SubElement(root, "sdf", {"version": "1.9"})
    sdf.text = "model.sdf"
    author = ET.SubElement(root, "author")
    ET.SubElement(author, "name").text = "Agrodrone workspace"
    ET.SubElement(root, "description").text = (
        "Generated stock Iris with its thrust axis used as an ArduPlane "
        "tailsitter aircraft-forward axis."
    )
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    gazebo_dir = script_dir.parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=gazebo_dir / "build" / "iris-tailsitter-fixture",
    )
    args = parser.parse_args()

    output_dir = args.output_dir.resolve()
    model_dir = output_dir / "models" / MODEL_NAME
    world_dir = output_dir / "worlds"
    model_dir.mkdir(parents=True, exist_ok=True)
    world_dir.mkdir(parents=True, exist_ok=True)

    generate_model(
        gazebo_dir / "models" / "iris_with_ardupilot" / "model.sdf",
        model_dir / "model.sdf",
    )
    write_model_config(model_dir / "model.config")
    generate_world(
        gazebo_dir / "worlds" / "iris_runway.sdf",
        world_dir / "iris_tailsitter_swap_test.sdf",
    )
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
