#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import sys
import time

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from g1_race_vision.controller import LaneFollowerConfig, LaneFollowerController
from g1_race_vision.line_detector import WhiteLaneDetector
from g1_race_vision.rendering import MujocoRgbdRenderer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a closed-loop RGB-D lane demo.")
    parser.add_argument(
        "--scene",
        type=Path,
        default=ROOT / "assets" / "standalone_lane_scene.xml",
    )
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--initial-y", type=float, default=0.35)
    parser.add_argument("--initial-yaw-deg", type=float, default=6.0)
    parser.add_argument("--speed", type=float, default=0.55)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "output" / "standalone"
    )
    return parser.parse_args()


def yaw_quaternion(yaw: float) -> np.ndarray:
    return np.array(
        [math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)],
        dtype=np.float64,
    )


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    # Loading from a string also works when the Windows workspace path contains
    # non-ASCII characters. This standalone scene has no external mesh assets.
    scene_xml = args.scene.resolve().read_text(encoding="utf-8")
    model = mujoco.MjModel.from_xml_string(scene_xml)
    data = mujoco.MjData(model)
    mocap_id = model.body("camera_rig").mocapid
    if mocap_id < 0:
        raise RuntimeError("camera_rig must be a mocap body")

    detector = WhiteLaneDetector()
    controller = LaneFollowerController(
        LaneFollowerConfig(cruise_speed_mps=args.speed)
    )
    renderer = MujocoRgbdRenderer(model)

    x = 0.0
    y = args.initial_y
    yaw = math.radians(args.initial_yaw_deg)
    dt = 1.0 / args.fps
    frames = max(1, int(round(args.seconds * args.fps)))
    rows: list[dict[str, float | str | bool]] = []
    video = None
    if args.record:
        video_path = args.output_dir / "debug.mp4"
        video = cv2.VideoWriter(
            str(video_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            args.fps,
            (640, 480),
        )

    last_debug = None
    try:
        for frame_index in range(frames):
            data.mocap_pos[mocap_id] = (x, y, 1.18)
            data.mocap_quat[mocap_id] = yaw_quaternion(yaw)
            mujoco.mj_forward(model, data)

            rgb, depth_m = renderer.render_rgbd(data)
            result = detector.detect(rgb, depth_m)
            command = controller.update(result, now=frame_index * dt)
            debug = detector.annotate(rgb, result)
            cv2.putText(
                debug,
                f"x={x:.2f}m y={y:+.3f}m yaw={math.degrees(yaw):+.2f}deg "
                f"vx={command.vx:.2f} wz={command.wz:+.3f}",
                (12, 92),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            last_debug = debug

            rows.append(
                {
                    "time_s": frame_index * dt,
                    "x_m": x,
                    "y_m": y,
                    "yaw_rad": yaw,
                    "valid": result.valid,
                    "confidence": result.confidence,
                    "lateral_error": result.lateral_error,
                    "heading_error_rad": result.heading_error_rad,
                    "vx_mps": command.vx,
                    "wz_rps": command.wz,
                    "state": command.state,
                }
            )

            if video is not None:
                video.write(cv2.cvtColor(debug, cv2.COLOR_RGB2BGR))
            if args.show:
                cv2.imshow("G1 race lane detector", cv2.cvtColor(debug, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(max(1, int(round(1000.0 * dt)))) & 0xFF in (27, ord("q")):
                    break

            x += command.vx * math.cos(yaw) * dt
            y += command.vx * math.sin(yaw) * dt
            yaw += command.wz * dt
    finally:
        renderer.close()
        if video is not None:
            video.release()
        if args.show:
            cv2.destroyAllWindows()

    if last_debug is not None:
        cv2.imwrite(
            str(args.output_dir / "last_debug.png"),
            cv2.cvtColor(last_debug, cv2.COLOR_RGB2BGR),
        )
    if rows:
        with (args.output_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(
            f"Finished: x={x:.2f} m, y={y:+.3f} m, "
            f"yaw={math.degrees(yaw):+.2f} deg"
        )
        print(f"Outputs: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
