#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


TRACK_BLUE_RGBA = "0.04 0.20 0.55 1"
TRACK_WHITE_RGBA = "0.97 0.97 0.97 1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a camera-equipped G1 model and a 100m race scene."
    )
    parser.add_argument(
        "--unitree-mujoco",
        type=Path,
        required=True,
        help="Path to the official unitree_mujoco checkout.",
    )
    parser.add_argument("--dofs", choices=(23, 29), type=int, default=29)
    parser.add_argument("--camera-name", default="d435i_rgb")
    # Approximate a head-mounted D435i: high enough to avoid arm occlusion,
    # wide enough to keep both boundaries of the two-metre lane in frame.
    parser.add_argument("--camera-forward", type=float, default=0.16)
    parser.add_argument("--camera-height", type=float, default=0.48)
    parser.add_argument("--camera-pitch-deg", type=float, default=10.0)
    parser.add_argument("--fovy", type=float, default=72.0)
    parser.add_argument(
        "--lane-width",
        type=float,
        default=2.1,
        help="Distance in metres between the two white-line centerlines.",
    )
    parser.add_argument("--lane-count", type=int, default=4)
    parser.add_argument(
        "--target-lane-index",
        type=int,
        default=1,
        help="Zero-based lane containing the robot at y=0.",
    )
    parser.add_argument(
        "--line-width",
        type=float,
        default=0.10,
        help="Physical width in metres of each white boundary line.",
    )
    parser.add_argument("--track-length", type=float, default=100.0)
    parser.add_argument(
        "--finish-distance",
        type=float,
        default=None,
        help=(
            "Finish stripe X position in metres. Defaults to --track-length; "
            "use a longer track length to provide post-finish runoff."
        ),
    )
    return parser.parse_args()


def camera_xyaxes(pitch_degrees: float) -> str:
    pitch = math.radians(pitch_degrees)
    # Image-right points toward robot -Y. Image-up is tilted so that the
    # camera viewing direction (-Z) points robot-forward (+X) and downward.
    values = (0.0, -1.0, 0.0, math.sin(pitch), 0.0, math.cos(pitch))
    return " ".join(f"{value:.9g}" for value in values)


def add_camera(
    source_model: Path,
    output_model: Path,
    name: str,
    forward: float,
    height: float,
    pitch_degrees: float,
    fovy: float,
) -> None:
    tree = ET.parse(source_model)
    root = tree.getroot()
    torso = root.find(".//body[@name='torso_link']")
    if torso is None:
        raise RuntimeError(f"torso_link not found in {source_model}")
    if root.find(f".//camera[@name='{name}']") is not None:
        raise RuntimeError(f"camera {name!r} already exists in {source_model}")

    ET.SubElement(
        torso,
        "camera",
        {
            "name": name,
            "mode": "fixed",
            "pos": f"{forward:.6g} 0 {height:.6g}",
            "xyaxes": camera_xyaxes(pitch_degrees),
            "fovy": f"{fovy:.6g}",
        },
    )
    ET.indent(tree, space="  ")
    tree.write(output_model, encoding="utf-8", xml_declaration=False)


def prepare_scene(
    source_scene: Path,
    output_scene: Path,
    patched_model_name: str,
    lane_width: float,
    line_width: float,
    track_length: float,
    lane_count: int = 4,
    target_lane_index: int = 1,
    finish_distance: float | None = None,
) -> None:
    if lane_width <= 0.0:
        raise ValueError("lane_width must be positive")
    if line_width <= 0.0:
        raise ValueError("line_width must be positive")
    if track_length <= 0.0:
        raise ValueError("track_length must be positive")
    if finish_distance is None:
        finish_distance = track_length
    if finish_distance <= 0.0 or finish_distance > track_length:
        raise ValueError(
            "finish_distance must be positive and no greater than track_length"
        )
    if lane_count <= 0:
        raise ValueError("lane_count must be positive")
    if not 0 <= target_lane_index < lane_count:
        raise ValueError("target_lane_index must identify an existing lane")

    tree = ET.parse(source_scene)
    root = tree.getroot()
    include = root.find("include")
    if include is None:
        raise RuntimeError(f"include element not found in {source_scene}")
    include.set("file", patched_model_name)

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    track_material = root.find(".//material[@name='race_track']")
    if track_material is None:
        track_material = ET.SubElement(
            asset,
            "material",
            {
                "name": "race_track",
            },
        )
    track_material.set("rgba", TRACK_BLUE_RGBA)
    track_material.set("reflectance", "0")

    white_material = root.find(".//material[@name='race_white']")
    if white_material is None:
        white_material = ET.SubElement(
            asset,
            "material",
            {
                "name": "race_white",
            },
        )
    white_material.set("rgba", TRACK_WHITE_RGBA)
    white_material.set("reflectance", "0")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"worldbody element not found in {source_scene}")
    floor = worldbody.find("geom[@name='floor']")
    if floor is not None:
        floor.set("material", "race_track")

    half_line = 0.5 * line_width
    center_x = 0.5 * track_length
    half_length = 0.5 * track_length + 2.5
    common = {
        "type": "box",
        "material": "race_white",
        "contype": "0",
        "conaffinity": "0",
        "group": "2",
    }
    # Place the selected lane around y=0, but render every neighbouring lane
    # so perception can be tested against accidental lane switching.
    boundary_positions = [
        (index - target_lane_index - 0.5) * lane_width
        for index in range(lane_count + 1)
    ]
    for index, boundary_y in enumerate(boundary_positions):
        ET.SubElement(
            worldbody,
            "geom",
            {
                **common,
                "name": f"race_lane_boundary_{index}",
                "pos": f"{center_x:.6g} {boundary_y:.6g} 0.002",
                "size": f"{half_length:.6g} {half_line:.6g} 0.002",
            },
        )

    track_center_y = 0.5 * (
        boundary_positions[0] + boundary_positions[-1]
    )
    half_track_width = 0.5 * lane_count * lane_width
    ET.SubElement(
        worldbody,
        "geom",
        {
            **common,
            "name": "race_finish_line",
            "pos": f"{finish_distance:.6g} {track_center_y:.6g} 0.002",
            "size": f"{half_line:.6g} {half_track_width:.6g} 0.002",
        },
    )

    ET.indent(tree, space="  ")
    tree.write(output_scene, encoding="utf-8", xml_declaration=False)


def main() -> None:
    args = parse_args()
    g1_dir = args.unitree_mujoco.expanduser().resolve() / "unitree_robots" / "g1"
    model_name = f"g1_{args.dofs}dof.xml"
    scene_name = f"scene_{args.dofs}dof.xml"
    source_model = g1_dir / model_name
    source_scene = g1_dir / scene_name
    if not source_model.is_file() or not source_scene.is_file():
        raise FileNotFoundError(
            f"Expected {source_model} and {source_scene}. "
            "Check --unitree-mujoco and --dofs."
        )

    output_model = g1_dir / f"g1_{args.dofs}dof_race.xml"
    output_scene = g1_dir / f"scene_race_{args.dofs}dof.xml"
    add_camera(
        source_model,
        output_model,
        args.camera_name,
        args.camera_forward,
        args.camera_height,
        args.camera_pitch_deg,
        args.fovy,
    )
    prepare_scene(
        source_scene,
        output_scene,
        output_model.name,
        args.lane_width,
        args.line_width,
        args.track_length,
        args.lane_count,
        args.target_lane_index,
        args.finish_distance,
    )
    print(f"Created camera model: {output_model}")
    print(f"Created race scene:  {output_scene}")
    print("The original Unitree files were not modified.")


if __name__ == "__main__":
    main()
