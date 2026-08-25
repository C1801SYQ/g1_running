#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import queue
import sys
import threading
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from g1_race_vision.camera_geometry import CameraIntrinsics
from g1_race_vision.command_output import CommandOutputGate
from g1_race_vision.controller import (
    ControllerCommand,
    LaneFollowerConfig,
    LaneFollowerController,
    Walk0p5mConfig,
    Walk0p5mGate,
)
from g1_race_vision.depth_safety import (
    DepthSafetyConfig,
    DepthSafetyGate,
    DepthSafetyResult,
)
from g1_race_vision.line_detector import LaneDetectorConfig, WhiteLaneDetector
from g1_race_vision.mission_lifecycle import apply_num7_mode_transition
from g1_race_vision.running_vision_mission import (
    RunningVisionMission,
    RunningVisionMissionConfig,
    RunningVisionRampConfig,
)
from g1_race_vision.sprint_ramp import (
    SprintRampConfig,
    clamp_command_vx,
    sprint_speed_cap,
)
from g1_race_vision.udp_command import (
    UdpVisionStatusReceiver,
    VisionMode,
)

try:
    import rclpy
    from cv_bridge import CvBridge
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
except ImportError as error:
    raise SystemExit(
        "ROS2 Python packages are missing. Source /opt/ros/humble/setup.bash "
        "and install ros-humble-cv-bridge."
    ) from error


class RealSenseLaneFollower(Node):
    def __init__(self) -> None:
        super().__init__("g1_realsense_lane_follower")
        self.declare_parameter(
            "color_topic", "/camera/camera/color/image_raw"
        )
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.declare_parameter("cruise_speed_mps", 0.20)
        self.declare_parameter("mission", "sprint100m")
        self.declare_parameter("running_vision_target_m", 110.0)
        self.declare_parameter("running_vision_max_duration_s", 260.0)
        self.declare_parameter("skill6_ramp_to_1_s", 0.60)
        self.declare_parameter("skill6_ramp_to_3_s", 0.80)
        self.declare_parameter("skill6_ramp_to_max_s", 1.00)
        self.declare_parameter("num7_target_m", 1.00)
        self.declare_parameter("num7_max_duration_s", 7.0)
        self.declare_parameter("num7_distance_scale", 0.60)
        self.declare_parameter("command_output_enabled", False)
        self.declare_parameter("status_host", "127.0.0.1")
        self.declare_parameter("status_port", 15002)
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 15001)
        self.declare_parameter("audit_udp_enabled", False)
        self.declare_parameter("audit_udp_host", "127.0.0.1")
        self.declare_parameter("audit_udp_port", 15003)
        self.declare_parameter("max_depth_age_s", 0.20)
        self.declare_parameter("depth_roi_left_ratio", 0.35)
        self.declare_parameter("depth_roi_right_ratio", 0.65)
        self.declare_parameter("depth_roi_top_ratio", 0.30)
        self.declare_parameter("depth_roi_bottom_ratio", 0.62)
        self.declare_parameter("depth_min_valid_fraction", 0.10)
        self.declare_parameter("depth_obstacle_percentile", 10.0)
        self.declare_parameter("depth_stop_distance_m", 0.80)
        self.declare_parameter("depth_slow_distance_m", 1.50)
        # Competition running mode has no obstacle course.  This flag only
        # bypasses the distance-based obstacle stop/slow decision for the
        # running-vision mission; sensor/communication fail-safe paths remain
        # active.
        self.declare_parameter("competition_no_obstacle_stop", False)
        # Competition running is IMU-primary: RGB/depth may add bounded yaw
        # correction, but camera health must never gate forward vx.
        self.declare_parameter("competition_imu_forward_only", False)
        self.declare_parameter("white_value_min", 175)
        self.declare_parameter("adaptive_value_floor", 55)
        self.declare_parameter("white_saturation_max", 100)
        self.declare_parameter("local_contrast_min", 14)
        self.declare_parameter("guided_search_margin_ratio", 0.075)
        self.declare_parameter("local_value_ratio", 0.65)
        self.declare_parameter("min_mean_thickness_ratio", 0.007)
        self.declare_parameter("initial_lock_confirm_frames", 10)
        self.declare_parameter("initial_min_center_margin_ratio", 0.03)
        self.declare_parameter("initial_min_abs_perspective_slope", 0.05)
        self.declare_parameter("expected_lane_width_m", 2.10)
        self.declare_parameter("min_lane_width_m", 1.75)
        self.declare_parameter("max_lane_width_m", 2.45)
        self.declare_parameter("debug_candidates", True)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.debug_candidates = bool(
            self.get_parameter("debug_candidates").value
        )
        speed = float(self.get_parameter("cruise_speed_mps").value)
        self.sprint_speed_mps = speed
        self.sprint_ramp = SprintRampConfig(
            ramp_to_1_s=float(
                self.get_parameter("skill6_ramp_to_1_s").value
            ),
            ramp_to_3_s=float(
                self.get_parameter("skill6_ramp_to_3_s").value
            ),
            ramp_to_max_s=float(
                self.get_parameter("skill6_ramp_to_max_s").value
            ),
        )
        self.mission = str(self.get_parameter("mission").value)
        if self.mission not in ("sprint100m", "walk0p5m", "runningvision110m"):
            raise ValueError(
                "mission must be sprint100m, walk0p5m, or runningvision110m"
            )
        running_vision_target_m = float(
            self.get_parameter("running_vision_target_m").value
        )
        running_vision_max_duration_s = float(
            self.get_parameter("running_vision_max_duration_s").value
        )
        self.num7_target_m = float(
            self.get_parameter("num7_target_m").value
        )
        self.num7_target_m = float(
            min(max(self.num7_target_m, 0.05), 200.00)
        )
        self.num7_max_duration_s = float(
            self.get_parameter("num7_max_duration_s").value
        )
        self.num7_max_duration_s = float(
            max(self.num7_max_duration_s, 0.1)
        )
        self.num7_distance_scale = float(
            min(
                max(
                    float(self.get_parameter("num7_distance_scale").value),
                    0.10,
                ),
                2.0,
            )
        )
        command_output_enabled = bool(
            self.get_parameter("command_output_enabled").value
        )
        status_host = self.get_parameter("status_host").value
        status_port = int(self.get_parameter("status_port").value)
        udp_host = self.get_parameter("udp_host").value
        udp_port = int(self.get_parameter("udp_port").value)
        audit_udp_enabled = bool(
            self.get_parameter("audit_udp_enabled").value
        )
        audit_udp_host = self.get_parameter("audit_udp_host").value
        audit_udp_port = int(self.get_parameter("audit_udp_port").value)
        max_depth_age_s = float(
            self.get_parameter("max_depth_age_s").value
        )
        self.competition_no_obstacle_stop = bool(
            self.get_parameter("competition_no_obstacle_stop").value
        )
        self.competition_imu_forward_only = bool(
            self.get_parameter("competition_imu_forward_only").value
        )

        detector_config = LaneDetectorConfig(
            correction_only_mode=(self.mission == "walk0p5m"),
            white_value_min=int(self.get_parameter("white_value_min").value),
            adaptive_value_floor=int(
                self.get_parameter("adaptive_value_floor").value
            ),
            white_saturation_max=int(
                self.get_parameter("white_saturation_max").value
            ),
            local_contrast_min=int(
                self.get_parameter("local_contrast_min").value
            ),
            guided_search_margin_ratio=float(
                self.get_parameter("guided_search_margin_ratio").value
            ),
            local_value_ratio=float(
                self.get_parameter("local_value_ratio").value
            ),
            min_mean_thickness_ratio=float(
                self.get_parameter("min_mean_thickness_ratio").value
            ),
            initial_lock_confirm_frames=int(
                self.get_parameter("initial_lock_confirm_frames").value
            ),
            initial_min_center_margin_ratio=float(
                self.get_parameter("initial_min_center_margin_ratio").value
            ),
            initial_min_abs_perspective_slope=float(
                self.get_parameter(
                    "initial_min_abs_perspective_slope"
                ).value
            ),
            expected_lane_width_m=float(
                self.get_parameter("expected_lane_width_m").value
            ),
            min_lane_width_m=float(
                self.get_parameter("min_lane_width_m").value
            ),
            max_lane_width_m=float(
                self.get_parameter("max_lane_width_m").value
            ),
        )
        self.bridge = CvBridge()
        self.detector = WhiteLaneDetector(detector_config)
        if self.mission == "walk0p5m":
            self.detector.configure_num7_lifecycle(True)
            # Use a command inside the locomotion policy's trained walking
            # range. The gate remains a secondary independent clamp.
            self.controller = LaneFollowerController(
                LaneFollowerConfig(
                    cruise_speed_mps=0.50,
                    minimum_tracking_speed_mps=0.50,
                    max_yaw_rate_rps=0.25,
                    max_forward_accel_mps2=0.20,
                    max_forward_decel_mps2=1.20,
                    max_yaw_accel_rps2=0.50,
                    lateral_kp=1.20,
                    heading_kp=0.20,
                    use_visual_heading_correction=True,
                    imu_heading_kp=1.20,
                    correction_enter_lateral_error=0.12,
                    correction_exit_lateral_error=0.06,
                    predictive_enter_lateral_error=0.08,
                    error_filter_alpha=0.32,
                )
            )
        else:
            self.controller = LaneFollowerController(
                LaneFollowerConfig(cruise_speed_mps=speed)
            )
        self.walk_gate = Walk0p5mGate(
            Walk0p5mConfig(
                target_distance_m=self.num7_target_m,
                max_duration_s=self.num7_max_duration_s,
                distance_scale=self.num7_distance_scale,
            )
        )
        self.depth_safety = DepthSafetyGate(
            DepthSafetyConfig(
                roi_left_ratio=float(
                    self.get_parameter("depth_roi_left_ratio").value
                ),
                roi_right_ratio=float(
                    self.get_parameter("depth_roi_right_ratio").value
                ),
                roi_top_ratio=float(
                    self.get_parameter("depth_roi_top_ratio").value
                ),
                roi_bottom_ratio=float(
                    self.get_parameter("depth_roi_bottom_ratio").value
                ),
                min_valid_fraction=float(
                    self.get_parameter("depth_min_valid_fraction").value
                ),
                obstacle_percentile=float(
                    self.get_parameter("depth_obstacle_percentile").value
                ),
                stop_distance_m=float(
                    self.get_parameter("depth_stop_distance_m").value
                ),
                slow_distance_m=float(
                    self.get_parameter("depth_slow_distance_m").value
                ),
                max_depth_age_s=max_depth_age_s,
            )
        )
        self.command_output = CommandOutputGate(
            enabled=command_output_enabled,
            host=udp_host,
            port=udp_port,
        )
        self.audit_output = CommandOutputGate(
            enabled=audit_udp_enabled,
            host=audit_udp_host,
            port=audit_udp_port,
        )
        self.latest_depth_m: np.ndarray | None = None
        self.latest_depth_time = 0.0
        self.camera_intrinsics: CameraIntrinsics | None = None
        self._last_detector_mode = "PREVIEW"
        self._num7_references_calibrated = False
        self._sprint_enabled = False
        self._sprint_started_at: float | None = None
        self._running_command_lock = threading.Lock()
        self._output_lock = threading.Lock()
        self._latest_running_command = ControllerCommand(
            0.0, 0.0, 0.0, "WAIT_FOR_RUNNING_VISION", False, False
        )
        self._last_color_time = 0.0
        self._running_vision_mission = RunningVisionMission(
            RunningVisionMissionConfig(
                target_distance_m=min(max(running_vision_target_m, 1.0), 200.0),
                cruise_speed_mps=min(max(speed, 0.2), 2.5),
                max_duration_s=min(
                    max(running_vision_max_duration_s, 1.0), 1200.0
                ),
                ramp=RunningVisionRampConfig(),
            )
        )
        self.latest_safety = self.depth_safety.evaluate(None, float("inf"))
        # The UDP listener runs on a background thread, while the detector,
        # controller and distance gate are owned by the ROS image callback.
        # Queue mode edges instead of mutating those state machines from both
        # threads. This also preserves a fast WALK -> NONE -> WALK sequence.
        self._mode_events: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._depth_callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            Image,
            depth_topic,
            self.on_depth,
            qos_profile_sensor_data,
            callback_group=self._depth_callback_group,
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
        self.cmd_publisher = self.create_publisher(
            Twist, "/g1/race/cmd_vel", 10
        )
        self.desired_cmd_publisher = self.create_publisher(
            Twist, "/g1/race/desired_cmd_vel", 10
        )
        self.safe_cmd_publisher = self.create_publisher(
            Twist, "/g1/race/safe_cmd_vel", 10
        )
        self.safety_publisher = self.create_publisher(
            DiagnosticArray, "/g1/race/safety_state", 10
        )
        self.debug_publisher = self.create_publisher(
            Image, "/g1/race/debug_image", 2
        )
        self.create_timer(0.10, self.on_safety_timer)
        # FSM status listener: for walk0p5m the mission only starts once the
        # C++ FSM reports G1_VISION_WALK_0P5M 1 on the status port. The gate
        # is explicitly activated/deactivated; before activation it never
        # accumulates time or distance and never emits a stale hard_stop.
        try:
            self.status_receiver = UdpVisionStatusReceiver(
                host=status_host,
                port=status_port,
            )
        except OSError as error:
            self.get_logger().fatal(
                f"FSM status port {status_host}:{status_port} is already "
                f"in use ({error}). Another vision/controller instance is "
                "running. Close it before starting Num7."
            )
            raise SystemExit(1) from error
        self._status_stop = threading.Event()
        self._status_thread = threading.Thread(
            target=self._status_loop, name="fsm-status", daemon=True
        )
        self._status_thread.start()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        if self.mission == "runningvision110m":
            self._heartbeat_thread = threading.Thread(
                target=self._running_vision_heartbeat_loop,
                name="running-vision-heartbeat",
                daemon=True,
            )
            self._heartbeat_thread.start()
        self.get_logger().info(
            f"FSM status listener on {status_host}:{status_port}"
        )
        self.get_logger().info(f"RGB:   {color_topic}")
        self.get_logger().info(f"Depth: {depth_topic}")
        if command_output_enabled:
            self.get_logger().warning(
                f"ROBOT COMMAND ENABLED: UDP {udp_host}:{udp_port}"
            )
        else:
            self.get_logger().warning(
                "DRY RUN - ROBOT COMMAND DISABLED; /g1/race/cmd_vel "
                "will remain zero"
            )
        if audit_udp_enabled:
            self.get_logger().warning(
                f"AUDIT UDP ENABLED: safe commands -> "
                f"{audit_udp_host}:{audit_udp_port}; this is separate from "
                "robot command output"
            )
        if self.mission == "runningvision110m" and self.competition_no_obstacle_stop:
            self.get_logger().warning(
                "COMPETITION MODE: obstacle distance stop/slow is disabled; "
                "communication watchdog, E-stop and mission gates "
                "remain active"
            )
        if self.mission == "runningvision110m" and self.competition_imu_forward_only:
            self.get_logger().warning(
                "IMU-PRIMARY MODE: RGB/depth are correction diagnostics only; "
                "camera health will not gate forward vx"
            )

    def _evaluate_running_vision_safety(
        self,
        depth_m: np.ndarray | None,
        depth_age_s: float,
    ) -> DepthSafetyResult:
        """Evaluate safety, with only course-obstacle handling bypassed.

        The competition course is obstacle-free and running is IMU-primary.
        In that explicit mode, depth is diagnostic only: missing, stale,
        malformed or obstacle-like depth cannot gate forward vx. The
        independent C++ communication watchdog and operator emergency-stop
        paths remain authoritative.
        """

        result = self.depth_safety.evaluate(depth_m, depth_age_s)
        if self.competition_imu_forward_only:
            return DepthSafetyResult(
                motion_allowed=True,
                speed_scale=1.0,
                obstacle_distance_m=result.obstacle_distance_m,
                valid_fraction=result.valid_fraction,
                depth_age_s=result.depth_age_s,
                reason=f"DEPTH_DIAGNOSTIC_ONLY|{result.reason}",
            )
        if (
            self.competition_no_obstacle_stop
            and result.reason in ("OBSTACLE_STOP", "OBSTACLE_SLOW")
        ):
            return DepthSafetyResult(
                motion_allowed=True,
                speed_scale=1.0,
                obstacle_distance_m=result.obstacle_distance_m,
                valid_fraction=result.valid_fraction,
                depth_age_s=result.depth_age_s,
                reason="OBSTACLE_IGNORED",
            )
        return result

    def _status_loop(self) -> None:
        """Poll the FSM status port and queue lifecycle edges for ROS."""
        while not self._status_stop.is_set():
            try:
                transitions = self.status_receiver.poll_events()
            except OSError as error:
                self.get_logger().error(
                    f"status receiver failed: {error}"
                )
                break
            for current_mode in transitions:
                if self.mission == "walk0p5m" or (
                    self.mission in ("sprint100m", "runningvision110m")
                    and current_mode in (
                        VisionMode.SPRINT100M,
                        VisionMode.WALK0P5M,
                        VisionMode.NONE,
                    )
                ):
                    self._mode_events.put(current_mode)
            # Wake up ~10 Hz to keep up with the C++ heartbeat.
            self._status_stop.wait(0.10)

    def _apply_pending_num7_transitions(self, now: float) -> None:
        """Apply every queued FSM edge on the image-callback thread.

        A genuine Num7 rising edge starts a brand-new lane identity. Repeated
        500 ms enable heartbeats are filtered by ``poll_events`` and therefore
        cannot reset the immutable lane anchor during a mission.
        """

        while True:
            try:
                current_mode = self._mode_events.get_nowait()
            except queue.Empty:
                return

            transition = apply_num7_mode_transition(
                current_mode,
                self.detector,
                self.controller,
                self.walk_gate,
                now=now,
            )
            if transition == "enabled":
                self.get_logger().warning(
                    "Num7 FSM enable received; detector identity reset and "
                    "mission timer started"
                )
            elif transition == "disabled":
                self.get_logger().warning(
                    "Num7 FSM disable received; gate inactive"
                )

    def _apply_pending_sprint_transitions(self, now: float) -> None:
        while True:
            try:
                current_mode = self._mode_events.get_nowait()
            except queue.Empty:
                return

            running_vision_walk_edge = (
                self.mission == "runningvision110m"
                and current_mode == VisionMode.WALK0P5M
            )
            if current_mode == VisionMode.SPRINT100M or running_vision_walk_edge:
                self._sprint_enabled = True
                self._sprint_started_at = now
                self.controller.reset()
                if self.mission == "runningvision110m":
                    self._running_vision_mission.enable(now)
                    self.get_logger().warning(
                        "Running-vision Num7 enable received; 110 m gate "
                        "and speed ramp start now"
                    )
                else:
                    self.get_logger().warning(
                        "Sprint FSM enable received; speed ramp starts now"
                    )
            elif current_mode == VisionMode.NONE and self._sprint_enabled:
                self._sprint_enabled = False
                self._sprint_started_at = None
                self.controller.reset()
                self._running_vision_mission.disable()
                self.get_logger().warning(
                    "Visual sprint disable received; robot command gated to zero"
                )

    def on_camera_info(self, message: CameraInfo) -> None:
        if message.width <= 0 or message.height <= 0:
            self.get_logger().warning(
                "ignoring invalid CameraInfo", once=True
            )
            return
        has_p = len(message.p) >= 8
        has_k = len(message.k) >= 9
        if not has_p and not has_k:
            self.get_logger().warning(
                "CameraInfo contains neither p nor k", once=True
            )
            return
        fx = float(message.p[0] if has_p else message.k[0])
        fy = float(message.p[5] if has_p else message.k[4])
        cx = float(message.p[2] if has_p else message.k[2])
        cy = float(message.p[6] if has_p else message.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(
                "CameraInfo has non-positive focal length", once=True
            )
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
        self.get_logger().info(
            "CameraInfo received: "
            f"fx={self.camera_intrinsics.fx:.1f} "
            f"fy={self.camera_intrinsics.fy:.1f} "
            f"cx={self.camera_intrinsics.cx:.1f} "
            f"cy={self.camera_intrinsics.cy:.1f}",
            once=True,
        )

    def on_depth(self, message: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(
            message, desired_encoding="passthrough"
        )
        if message.encoding in ("16UC1", "mono16"):
            self.latest_depth_m = depth.astype(np.float32) * 0.001
        else:
            self.latest_depth_m = depth.astype(np.float32)
        self.latest_depth_time = time.monotonic()

    def on_color(self, message: Image) -> None:
        now = time.monotonic()
        if self.mission == "walk0p5m":
            self._apply_pending_num7_transitions(now)
        elif self.mission in ("sprint100m", "runningvision110m"):
            self._apply_pending_sprint_transitions(now)

        rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
        max_age = float(self.get_parameter("max_depth_age_s").value)
        depth_age = time.monotonic() - self.latest_depth_time
        depth = (
            self.latest_depth_m
            if depth_age <= max_age
            else None
        )
        if depth is not None and depth.shape != rgb.shape[:2]:
            depth = None
        intrinsics = (
            self.camera_intrinsics
            if depth is not None
            else None
        )

        result = self.detector.detect(rgb, depth, intrinsics)
        if self.mission == "walk0p5m":
            if (
                result.mode == "LOCKED"
                and self._last_detector_mode != "LOCKED"
            ):
                self.detector.set_lateral_reference(result.lateral_error)
                self.controller.set_visual_heading_reference(
                    result.heading_error_rad
                )
                self._num7_references_calibrated = True
                self.get_logger().warning(
                    "Num7 initial lock completed; lane/heading references "
                    "calibrated"
                )
            self._last_detector_mode = result.mode
            desired_command = self.controller.update(result)
            self.latest_safety = self.depth_safety.evaluate(
                self.latest_depth_m, depth_age
            )
            if self.latest_safety.motion_allowed:
                safe_command = self.walk_gate.update(
                    desired_command, now=now
                )
            else:
                safe_command = ControllerCommand(
                    0.0, 0.0, 0.0, self.latest_safety.reason, False, True
                )
        elif self.mission == "runningvision110m":
            desired_command = self.controller.update(result)
            self.latest_safety = self._evaluate_running_vision_safety(
                self.latest_depth_m, depth_age
            )
            mission_command = self._running_vision_mission.update(
                now,
                desired_command.wz,
                perception_valid=result.valid,
                motion_allowed=self.latest_safety.motion_allowed,
                safety_reason=self.latest_safety.reason,
            )
            safe_command = self.depth_safety.apply(
                mission_command, self.latest_safety
            )
            if not self._sprint_enabled:
                safe_command = ControllerCommand(
                    0.0,
                    0.0,
                    0.0,
                    "WAIT_FOR_RUNNING_VISION",
                    result.valid,
                    False,
                )
            with self._running_command_lock:
                self._latest_running_command = safe_command
                self._last_color_time = now
        else:
            desired_command = self.controller.update(result)
            self.latest_safety = self.depth_safety.evaluate(
                self.latest_depth_m, depth_age
            )
            safe_command = self.depth_safety.apply(
                desired_command, self.latest_safety
            )
            if not self._sprint_enabled:
                if self.command_output.enabled:
                    safe_command = ControllerCommand(
                        0.0, 0.0, 0.0, "WAIT_FOR_SKILL6", result.valid, False
                    )
            else:
                ramp_elapsed = (
                    now - self._sprint_started_at
                    if self._sprint_started_at is not None
                    else 0.0
                )
                safe_command = clamp_command_vx(
                    safe_command,
                    sprint_speed_cap(
                        ramp_elapsed,
                        self.sprint_speed_mps,
                        self.sprint_ramp,
                    ),
                )
        with self._output_lock:
            self.audit_output.emit(safe_command)
            actual_command = self.command_output.emit(safe_command)

        self.desired_cmd_publisher.publish(self._to_twist(desired_command))
        self.safe_cmd_publisher.publish(self._to_twist(safe_command))
        self.cmd_publisher.publish(self._to_twist(actual_command))
        self.publish_safety_status(self.latest_safety)

        debug = self.detector.annotate(
            rgb, result, debug_candidates=self.debug_candidates
        )
        cv2.putText(
            debug,
            (
                f"{safe_command.state} safe "
                f"vx={safe_command.vx:.2f} wz={safe_command.wz:+.3f} "
                f"depth={self.latest_safety.reason} "
                f"output={'ON' if self.command_output.enabled else 'DRY-RUN'}"
            ),
            (12, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="rgb8")
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

    def _running_vision_heartbeat_loop(self) -> None:
        """Keep the C++ watchdog fed independently of RGB processing.

        USB2 D435i delivers RGB at about 6 Hz, while a 640x480 detection and
        debug publish can occasionally occupy the ROS callback for more than
        the C++ 300 ms watchdog window. The heartbeat never invents a first
        command. In IMU-primary competition mode it repeats forward vx and
        removes stale visual yaw; camera/depth status remains diagnostic.
        """

        while not self._heartbeat_stop.wait(0.05):
            if not self._sprint_enabled:
                continue

            with self._running_command_lock:
                command = self._latest_running_command
                last_color_time = self._last_color_time

            # Do not turn the pre-first-frame zero into a received command;
            # the existing C++ start watchdog must remain authoritative.
            if command.state == "WAIT_FOR_RUNNING_VISION":
                continue

            now = time.monotonic()
            depth_age = now - self.latest_depth_time
            safety = self._evaluate_running_vision_safety(
                self.latest_depth_m, depth_age
            )
            if not safety.motion_allowed:
                heartbeat = ControllerCommand(
                    0.0, 0.0, 0.0, safety.reason, False, True
                )
            elif command.hard_stop:
                heartbeat = command
            elif now - last_color_time > 0.25:
                # No fresh RGB result: continue forward, but do not hold a
                # stale visual yaw correction.  This is the intended
                # straight-running/no-line behavior.
                heartbeat = ControllerCommand(
                    max(0.0, command.vx),
                    0.0,
                    0.0,
                    "RUNNING_VISION_FORWARD_NO_LINE",
                    False,
                    False,
                )
            else:
                heartbeat = command

            with self._output_lock:
                self.audit_output.emit(heartbeat)
                self.command_output.emit(heartbeat)

    def on_safety_timer(self) -> None:
        depth_age = time.monotonic() - self.latest_depth_time
        if self.mission == "runningvision110m":
            self.latest_safety = self._evaluate_running_vision_safety(
                self.latest_depth_m, depth_age
            )
        else:
            self.latest_safety = self.depth_safety.evaluate(
                self.latest_depth_m, depth_age
            )
        self.publish_safety_status(self.latest_safety)

    def publish_safety_status(self, result: DepthSafetyResult) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        if not result.motion_allowed:
            status.level = DiagnosticStatus.ERROR
        elif result.speed_scale < 1.0:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.name = "g1_race/depth_safety"
        status.hardware_id = "d435i"
        status.message = result.reason
        status.values = [
            KeyValue(key="motion_allowed", value=str(result.motion_allowed)),
            KeyValue(key="speed_scale", value=f"{result.speed_scale:.3f}"),
            KeyValue(
                key="obstacle_distance_m",
                value=f"{result.obstacle_distance_m:.3f}",
            ),
            KeyValue(
                key="valid_fraction",
                value=f"{result.valid_fraction:.3f}",
            ),
            KeyValue(key="depth_age_s", value=f"{result.depth_age_s:.3f}"),
        ]
        message.status = [status]
        self.safety_publisher.publish(message)

    @staticmethod
    def _to_twist(command: ControllerCommand) -> Twist:
        twist = Twist()
        twist.linear.x = command.vx
        twist.linear.y = command.vy
        twist.angular.z = command.wz
        return twist

    def destroy_node(self):
        self._heartbeat_stop.set()
        if (
            self._heartbeat_thread is not None
            and self._heartbeat_thread.is_alive()
        ):
            self._heartbeat_thread.join(timeout=1.0)
        self._status_stop.set()
        if self._status_thread.is_alive():
            self._status_thread.join(timeout=1.0)
        self.status_receiver.close()
        self.audit_output.close()
        self.command_output.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RealSenseLaneFollower()
    executor: MultiThreadedExecutor | None = None
    try:
        if node.mission == "runningvision110m":
            # Keep Depth callbacks alive while RGB detection/annotation is
            # busy. Other missions retain the original single-thread path.
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
