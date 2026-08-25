#!/usr/bin/env python3
"""Show the live ROS2 RealSense lane-detection dashboard without commands.

This is the hardware counterpart of the MuJoCo camera dashboard.  It
subscribes directly to the camera topics, runs the same WhiteLaneDetector,
and never publishes velocity commands or talks to rl_sar.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from g1_race_vision.camera_geometry import CameraIntrinsics
from g1_race_vision.line_detector import LaneDetectorConfig, WhiteLaneDetector


WINDOW_NAME = "G1 robot camera - RGB-D lane detection"


def add_panel_title(
    image: np.ndarray,
    title: str,
    color: tuple[int, int, int] = (255, 255, 255),
) -> np.ndarray:
    panel = image.copy()
    cv2.rectangle(
        panel,
        (0, 0),
        (panel.shape[1] - 1, 38),
        (20, 20, 20),
        -1,
    )
    cv2.putText(
        panel,
        title,
        (12, 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.66,
        color,
        2,
        cv2.LINE_AA,
    )
    return panel


def build_dashboard(
    rgb: np.ndarray,
    annotated_rgb: np.ndarray,
    source: str,
    status_lines: tuple[str, ...] = (),
) -> np.ndarray:
    """Build the raw-left / detection-right camera view."""

    raw = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    detected = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
    raw = add_panel_title(raw, "RAW ROBOT RGB")
    detected_title = (
        "DETECTION: BOTH LINES"
        if source.startswith("two-lines")
        else "DETECTION: TWO LINES REQUIRED"
    )
    detected_color = (80, 255, 80) if source.startswith("two-lines") else (80, 80, 255)
    detected = add_panel_title(detected, detected_title, detected_color)

    dashboard = np.hstack((raw, detected))
    for index, line in enumerate(status_lines):
        cv2.putText(
            dashboard,
            line,
            (10, dashboard.shape[0] - 28 + index * 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
    return dashboard


class RealSenseDashboard(Node):
    def __init__(self) -> None:
        super().__init__("g1_realsense_dashboard_preview")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        self.declare_parameter("max_depth_age_s", 0.20)
        self.declare_parameter("debug_candidates", True)

        self.bridge = CvBridge()
        self.detector = WhiteLaneDetector(LaneDetectorConfig())
        self.debug_candidates = bool(
            self.get_parameter("debug_candidates").value
        )
        self.max_depth_age_s = float(
            self.get_parameter("max_depth_age_s").value
        )
        self.latest_depth_m: np.ndarray | None = None
        self.latest_depth_time = 0.0
        self.camera_intrinsics: CameraIntrinsics | None = None
        self.latest_dashboard: np.ndarray | None = None
        self.latest_dashboard_time = 0.0
        self.latest_dashboard_seq = 0
        self.latest_color_time = 0.0
        self.latest_color_seq = 0
        self.received_color_fps = 0.0
        self.processed_color_fps = 0.0

        # Keep ROS callbacks short.  The old implementation ran RGB-D
        # conversion, lane detection, annotation, and dashboard composition
        # inside on_color().  If one detection took longer than the camera
        # period, the executor stopped servicing new frames and the GUI
        # appeared frozen on its first image.  The callback now only keeps
        # the newest frame; this worker processes that newest frame and drops
        # stale frames by design.
        self._frame_condition = threading.Condition()
        self._latest_rgb: np.ndarray | None = None
        self._latest_rgb_time = 0.0
        self._latest_rgb_seq = 0
        self._worker_stop = False
        self._worker = threading.Thread(
            target=self._process_latest_frame,
            name="skill6-dashboard-detector",
            daemon=True,
        )
        self._worker.start()

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.create_subscription(
            Image, depth_topic, self.on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, color_topic, self.on_color, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"RGB: {color_topic}")
        self.get_logger().info(f"Depth: {depth_topic}")
        self.get_logger().info("Preview only: no velocity publisher and no UDP output")

    def on_camera_info(self, message: CameraInfo) -> None:
        if message.width <= 0 or message.height <= 0:
            return
        has_p = len(message.p) >= 8
        has_k = len(message.k) >= 9
        if not has_p and not has_k:
            return
        fx = float(message.p[0] if has_p else message.k[0])
        fy = float(message.p[5] if has_p else message.k[4])
        cx = float(message.p[2] if has_p else message.k[2])
        cy = float(message.p[6] if has_p else message.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return
        self.camera_intrinsics = CameraIntrinsics(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=int(message.width),
            height=int(message.height),
            model=message.distortion_model or "plumb_bob",
            distortion=tuple(float(value) for value in message.d),
        )

    def on_depth(self, message: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(
            message, desired_encoding="passthrough"
        )
        if message.encoding in ("16UC1", "mono16"):
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)
        with self._frame_condition:
            self.latest_depth_m = depth_m
            self.latest_depth_time = time.monotonic()

    def on_color(self, message: Image) -> None:
        rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
        received_at = time.monotonic()
        with self._frame_condition:
            self._latest_rgb = rgb
            self._latest_rgb_time = received_at
            self._latest_rgb_seq += 1
            self.latest_color_time = received_at
            self.latest_color_seq = self._latest_rgb_seq
            self._frame_condition.notify()

    def _process_latest_frame(self) -> None:
        last_processed_seq = 0
        fps_window_started = time.monotonic()
        received_at_window_start = 0
        processed_in_window = 0

        while True:
            with self._frame_condition:
                while (
                    not self._worker_stop
                    and self._latest_rgb is not None
                    and self._latest_rgb_seq == last_processed_seq
                ):
                    self._frame_condition.wait(timeout=0.20)
                if self._worker_stop:
                    return
                if self._latest_rgb is None:
                    self._frame_condition.wait(timeout=0.20)
                    continue

                # Copy the references while holding the lock, then release it
                # before doing any OpenCV work.  A new callback can replace
                # these references immediately, so the worker never blocks
                # reception of the next camera frame.
                rgb = self._latest_rgb
                color_time = self._latest_rgb_time
                color_seq = self._latest_rgb_seq
                depth = self.latest_depth_m
                depth_time = self.latest_depth_time
                intrinsics = self.camera_intrinsics

            depth_age = time.monotonic() - depth_time
            if depth is None or depth_age > self.max_depth_age_s:
                depth = None
                intrinsics = None
            elif depth.shape != rgb.shape[:2]:
                depth = None
                intrinsics = None

            result = self.detector.detect(rgb, depth, intrinsics)
            annotated = self.detector.annotate(
                rgb,
                result,
                debug_candidates=self.debug_candidates,
            )

            now = time.monotonic()
            processed_in_window += 1
            elapsed = now - fps_window_started
            if elapsed >= 1.0:
                self.processed_color_fps = processed_in_window / elapsed
                with self._frame_condition:
                    received_count = self._latest_rgb_seq - received_at_window_start
                self.received_color_fps = received_count / elapsed
                received_at_window_start = self._latest_rgb_seq
                processed_in_window = 0
                fps_window_started = now

            age_ms = max(0.0, (now - color_time) * 1000.0)
            status_lines = (
                f"RGB {self.received_color_fps:4.1f} fps | "
                f"PROC {self.processed_color_fps:4.1f} fps | "
                f"AGE {age_ms:5.0f} ms | FRAME {color_seq}",
                "LIVE ROBOT CAMERA | PREVIEW ONLY | COMMAND OUTPUT: OFF",
            )
            dashboard = build_dashboard(
                rgb,
                annotated,
                result.source,
                status_lines=status_lines,
            )
            with self._frame_condition:
                self.latest_dashboard = dashboard
                self.latest_dashboard_time = now
                self.latest_dashboard_seq = color_seq
            last_processed_seq = color_seq

    def close(self) -> None:
        with self._frame_condition:
            self._worker_stop = True
            self._frame_condition.notify_all()
        self._worker.join(timeout=2.0)


def main() -> int:
    if not os.environ.get("DISPLAY"):
        print(
            "DISPLAY is not set. Run this from a terminal opened inside "
            "the NoMachine desktop.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = RealSenseDashboard()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1400, 520)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.latest_dashboard is not None:
                cv2.imshow(WINDOW_NAME, node.latest_dashboard)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        node.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
