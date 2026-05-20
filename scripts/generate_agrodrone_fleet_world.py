#!/usr/bin/env python3
"""Generate a Gazebo field world with one truck rig and one pad per drone."""

from __future__ import annotations

import argparse
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


BASE_DRONE_MODEL = "iris_with_sprayer"
BASE_GIMBAL_MODEL = "gimbal_small_3d"
WORLD_TEMPLATE_DRONE_URIS = {
    "model://iris_with_sprayer",
    "model://iris_with_sprayer_2",
}
TRAILER_SPACING_M = 7.8
TRUCK_PAD_X_M = -1.0
FIRST_TRAILER_PAD_X_M = -8.8
BASE_Y_M = -30.0
DRONE_SPAWN_Z_M = 1.601


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an Agrodrone Gazebo fleet world for one to four drones."
    )
    parser.add_argument("--num-drones", type=int, choices=range(1, 5), required=True)
    parser.add_argument("--world-name", default="iris_dynamic_field")
    parser.add_argument("--template-world", type=Path, required=True)
    parser.add_argument("--source-models-dir", type=Path, required=True)
    parser.add_argument("--generated-models-dir", type=Path, required=True)
    parser.add_argument("--generated-worlds-dir", type=Path, required=True)
    parser.add_argument("--latitude-deg", type=float, default=None)
    parser.add_argument("--longitude-deg", type=float, default=None)
    parser.add_argument("--elevation-m", type=float, default=None)
    parser.add_argument("--heading-deg", type=float, default=None)
    parser.add_argument(
        "--physics-max-step-size",
        type=float,
        default=None,
        help="Override the generated world's physics <max_step_size> in seconds.",
    )
    parser.add_argument(
        "--disable-camera-sensors",
        action="store_true",
        help=(
            "Generate model overrides with camera sensors removed. This keeps GPS, "
            "SITL, and sprayer behavior while avoiding Ogre camera rendering load "
            "during long full-field validation runs."
        ),
    )
    parser.add_argument(
        "--disable-gpu-lidar-sensors",
        action="store_true",
        help=(
            "Generate model overrides with Gazebo GPU lidar rangefinder sensors "
            "removed. Intended only for throughput validation runs that do not "
            "depend on rangefinder data."
        ),
    )
    parser.add_argument(
        "--minimal-scenery",
        action="store_true",
        help=(
            "Remove decorative crop rows, axes, and Fuel tree includes from the "
            "generated world. This keeps drones, ground, bases, GPS, and sprayer "
            "plugins for long headless validation runs."
        ),
    )
    return parser.parse_args()


def _drone_model_name(drone_number: int) -> str:
    if drone_number == 1:
        return BASE_DRONE_MODEL
    return f"{BASE_DRONE_MODEL}_{drone_number}"


def _gimbal_model_name(drone_number: int) -> str:
    if drone_number == 1:
        return BASE_GIMBAL_MODEL
    return f"{BASE_GIMBAL_MODEL}_udp_{5600 + drone_number - 1}"


def _pad_x(drone_number: int) -> float:
    if drone_number == 1:
        return TRUCK_PAD_X_M
    return FIRST_TRAILER_PAD_X_M - (drone_number - 2) * TRAILER_SPACING_M


def _fmt(value: float) -> str:
    text = f"{value:.3f}"
    return text.rstrip("0").rstrip(".")


def _fmt_geo(value: float) -> str:
    text = f"{value:.9f}"
    return text.rstrip("0").rstrip(".")


def _write_if_changed(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding="utf-8") == content:
        return
    path.write_text(content, encoding="utf-8")


def _remove_generated_model_override(generated_models_dir: Path, name: str) -> None:
    path = generated_models_dir / name
    if path.exists():
        shutil.rmtree(path)


def _generated_model_config(name: str, description: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


def _generate_gimbal_variant(
    *,
    drone_number: int,
    source_models_dir: Path,
    generated_models_dir: Path,
    force: bool = False,
    disable_camera_sensors: bool = False,
) -> None:
    if drone_number <= 2 and not force:
        return

    model_name = _gimbal_model_name(drone_number)
    source = source_models_dir / BASE_GIMBAL_MODEL / "model.sdf"
    text = source.read_text(encoding="utf-8")
    text = text.replace(f'<model name="{BASE_GIMBAL_MODEL}">', f'<model name="{model_name}">')
    text = text.replace("<udp_port>5600</udp_port>", f"<udp_port>{5600 + drone_number - 1}</udp_port>")
    if disable_camera_sensors:
        text = _strip_sensor_elements(text, sensor_types={"camera"})

    target_dir = generated_models_dir / model_name
    _write_if_changed(target_dir / "model.sdf", text)
    _write_if_changed(
        target_dir / "model.config",
        _generated_model_config(
            model_name,
            f"Generated gimbal camera variant for drone {drone_number}.",
        ),
    )


def _generate_drone_variant(
    *,
    drone_number: int,
    source_models_dir: Path,
    generated_models_dir: Path,
    force: bool = False,
    disable_camera_sensors: bool = False,
    disable_gpu_lidar_sensors: bool = False,
) -> None:
    if drone_number <= 2 and not force:
        return

    model_name = _drone_model_name(drone_number)
    gimbal_name = _gimbal_model_name(drone_number)
    fdm_port = 9002 + (drone_number - 1) * 10
    downward_camera_port = 5610 + drone_number - 1

    source = source_models_dir / BASE_DRONE_MODEL / "model.sdf"
    text = source.read_text(encoding="utf-8")
    text = text.replace(f'<model name="{BASE_DRONE_MODEL}">', f'<model name="{model_name}">')
    text = text.replace(f"<uri>model://{BASE_GIMBAL_MODEL}</uri>", f"<uri>model://{gimbal_name}</uri>")
    text = re.sub(
        r"<fdm_port_in>\d+</fdm_port_in>",
        f"<fdm_port_in>{fdm_port}</fdm_port_in>",
        text,
        count=1,
    )
    text = text.replace(f"/model/{BASE_DRONE_MODEL}/", f"/model/{model_name}/")
    text = re.sub(
        r"<udp_port>5610</udp_port>",
        f"<udp_port>{downward_camera_port}</udp_port>",
        text,
        count=1,
    )
    stripped_sensor_types: set[str] = set()
    if disable_camera_sensors:
        stripped_sensor_types.add("camera")
    if disable_gpu_lidar_sensors:
        stripped_sensor_types.add("gpu_lidar")
    if stripped_sensor_types:
        text = _strip_sensor_elements(
            text,
            sensor_types=stripped_sensor_types,
            remove_ardupilot_lidar=disable_gpu_lidar_sensors,
        )

    target_dir = generated_models_dir / model_name
    _write_if_changed(target_dir / "model.sdf", text)
    _write_if_changed(
        target_dir / "model.config",
        _generated_model_config(
            model_name,
            f"Generated Iris sprayer variant for SITL drone {drone_number}.",
        ),
    )


def _include(uri: str, pose: str, *, name: str | None = None, degrees: bool = False) -> ET.Element:
    include = ET.Element("include")
    uri_element = ET.SubElement(include, "uri")
    uri_element.text = uri
    if name:
        name_element = ET.SubElement(include, "name")
        name_element.text = name
    pose_element = ET.SubElement(include, "pose")
    if degrees:
        pose_element.set("degrees", "true")
    pose_element.text = pose
    return include


def _remove_static_fleet_includes(world: ET.Element) -> None:
    for include in list(world.findall("include")):
        uri = (include.findtext("uri") or "").strip()
        name = (include.findtext("name") or "").strip()
        if uri in WORLD_TEMPLATE_DRONE_URIS or uri == "model://landing_truck" or name == "landing_truck":
            world.remove(include)


def _remove_minimal_scenery(world: ET.Element) -> None:
    decorative_model_names = {"crop_field", "axes"}
    for model in list(world.findall("model")):
        if (model.get("name") or "").strip() in decorative_model_names:
            world.remove(model)

    decorative_fuel_model_suffixes = ("/Oak tree", "/Pine Tree")
    for include in list(world.findall("include")):
        uri = (include.findtext("uri") or "").strip()
        if uri.endswith(decorative_fuel_model_suffixes):
            world.remove(include)


def _set_spherical_coordinates(world: ET.Element, args: argparse.Namespace) -> None:
    values = {
        "latitude_deg": args.latitude_deg,
        "longitude_deg": args.longitude_deg,
        "elevation": args.elevation_m,
        "heading_deg": args.heading_deg,
    }
    if all(value is None for value in values.values()):
        return

    spherical = world.find("spherical_coordinates")
    if spherical is None:
        spherical = ET.SubElement(world, "spherical_coordinates")

    for tag, value in values.items():
        if value is None:
            continue
        element = spherical.find(tag)
        if element is None:
            element = ET.SubElement(spherical, tag)
        element.text = _fmt_geo(value)

    if spherical.find("surface_model") is None:
        surface_model = ET.SubElement(spherical, "surface_model")
        surface_model.text = "EARTH_WGS84"


def _set_physics_max_step_size(world: ET.Element, value: float | None) -> None:
    if value is None:
        return
    if value <= 0:
        raise ValueError("--physics-max-step-size must be greater than zero")

    physics = world.find("physics")
    if physics is None:
        physics = ET.SubElement(world, "physics")
        physics.set("name", "generated")
        physics.set("type", "ignore")
    max_step = physics.find("max_step_size")
    if max_step is None:
        max_step = ET.SubElement(physics, "max_step_size")
    max_step.text = _fmt_geo(value)


def _strip_sensor_elements(
    text: str,
    *,
    sensor_types: set[str],
    remove_ardupilot_lidar: bool = False,
) -> str:
    root = ET.fromstring(text)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "sensor" and child.get("type") in sensor_types:
                parent.remove(child)
            elif (
                remove_ardupilot_lidar
                and child.tag == "sensor"
                and (child.findtext("type") or "").strip() == "lidar"
            ):
                parent.remove(child)
    if hasattr(ET, "indent"):
        ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def _remove_render_sensors_system(world: ET.Element) -> None:
    for plugin in list(world.findall("plugin")):
        if (
            plugin.get("filename") == "gz-sim-sensors-system"
            and plugin.get("name") == "gz::sim::systems::Sensors"
        ):
            world.remove(plugin)


def _generate_world(args: argparse.Namespace) -> Path:
    tree = ET.parse(args.template_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"Template world has no <world>: {args.template_world}")

    world.set("name", args.world_name)
    _set_spherical_coordinates(world, args)
    _set_physics_max_step_size(world, args.physics_max_step_size)
    if args.disable_camera_sensors and args.disable_gpu_lidar_sensors:
        _remove_render_sensors_system(world)
    _remove_static_fleet_includes(world)
    if args.minimal_scenery:
        _remove_minimal_scenery(world)

    world.append(ET.Comment(" Generated fleet drone spawns "))
    for drone_number in range(1, args.num_drones + 1):
        model_name = _drone_model_name(drone_number)
        pose = (
            f"{_fmt(_pad_x(drone_number))} {_fmt(BASE_Y_M)} "
            f"{_fmt(DRONE_SPAWN_Z_M)} 0 0 90"
        )
        world.append(_include(f"model://{model_name}", pose, name=model_name, degrees=True))

    world.append(ET.Comment(" Generated truck rig and additional trailer pads "))
    world.append(
        _include(
            "model://landing_truck",
            f"0 {_fmt(BASE_Y_M)} 0 0 0 0",
            name="landing_truck",
        )
    )
    for drone_number in range(3, args.num_drones + 1):
        trailer_number = drone_number - 1
        trailer_name = f"landing_trailer_{trailer_number}"
        trailer_pose = f"{_fmt(_pad_x(drone_number))} {_fmt(BASE_Y_M)} 0 0 0 0"
        world.append(_include("model://landing_trailer", trailer_pose, name=trailer_name))

    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")

    output_world = args.generated_worlds_dir / f"{args.world_name}.sdf"
    args.generated_worlds_dir.mkdir(parents=True, exist_ok=True)
    tree.write(output_world, encoding="utf-8", xml_declaration=True)
    return output_world


def main() -> None:
    args = _parse_args()
    args.generated_models_dir.mkdir(parents=True, exist_ok=True)
    args.generated_worlds_dir.mkdir(parents=True, exist_ok=True)

    for drone_number in range(args.num_drones + 1, 5):
        _remove_generated_model_override(args.generated_models_dir, _drone_model_name(drone_number))
        _remove_generated_model_override(args.generated_models_dir, _gimbal_model_name(drone_number))

    generated_model_overrides = args.disable_camera_sensors or args.disable_gpu_lidar_sensors
    if not generated_model_overrides:
        for drone_number in (1, 2):
            _remove_generated_model_override(args.generated_models_dir, _drone_model_name(drone_number))
            _remove_generated_model_override(args.generated_models_dir, _gimbal_model_name(drone_number))

    first_generated_drone = 1 if generated_model_overrides else 3
    for drone_number in range(first_generated_drone, args.num_drones + 1):
        _generate_gimbal_variant(
            drone_number=drone_number,
            source_models_dir=args.source_models_dir,
            generated_models_dir=args.generated_models_dir,
            force=generated_model_overrides,
            disable_camera_sensors=args.disable_camera_sensors,
        )
        _generate_drone_variant(
            drone_number=drone_number,
            source_models_dir=args.source_models_dir,
            generated_models_dir=args.generated_models_dir,
            force=generated_model_overrides,
            disable_camera_sensors=args.disable_camera_sensors,
            disable_gpu_lidar_sensors=args.disable_gpu_lidar_sensors,
        )

    print(_generate_world(args))


if __name__ == "__main__":
    main()
