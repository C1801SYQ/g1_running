#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from g1_race_vision.controller import LaneFollowerConfig, LaneFollowerController
from g1_race_vision.line_detector import LaneDetectorConfig, WhiteLaneDetector
from g1_race_vision.udp_command import UdpCommandSender

try:
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
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
        self.declare_parameter("cruise_speed_mps", 0.20)
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 15001)
        self.declare_parameter("max_depth_age_s", 0.15)
        self.declare_parameter("white_value_min", 175)
        self.declare_parameter("adaptive_value_floor", 55)
        self.declare_parameter("white_saturation_max", 100)
        self.declare_parameter("local_contrast_min", 14)
        self.declare_parameter("guided_search_margin_ratio", 0.075)

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        speed = float(self.get_parameter("cruise_speed_mps").value)
        udp_host = self.get_parameter("udp_host").value
        udp_port = int(self.get_parameter("udp_port").value)

        detector_config = LaneDetectorConfig(
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
        )
        self.bridge = CvBridge()
        self.detector = WhiteLaneDetector(detector_config)
        self.controller = LaneFollowerController(
            LaneFollowerConfig(cruise_speed_mps=speed)
        )
        self.sender = UdpCommandSender(udp_host, udp_port)
        self.latest_depth_m: np.ndarray | None = None
        self.latest_depth_time = 0.0

        self.create_subscription(
            Image, depth_topic, self.on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, color_topic, self.on_color, qos_profile_sensor_data
        )
        self.cmd_publisher = self.create_publisher(Twist, "/g1/race/cmd_vel", 10)
        self.debug_publisher = self.create_publisher(
            Image, "/g1/race/debug_image", 2
        )
        self.get_logger().info(f"RGB:   {color_topic}")
        self.get_logger().info(f"Depth: {depth_topic}")
        self.get_logger().info(f"UDP:   {udp_host}:{udp_port}")

    def on_depth(self, message: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        if message.encoding in ("16UC1", "mono16"):
            self.latest_depth_m = depth.astype(np.float32) * 0.001
        else:
            self.latest_depth_m = depth.astype(np.float32)
        self.latest_depth_time = time.monotonic()

    def on_color(self, message: Image) -> None:
        rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
        max_age = float(self.get_parameter("max_depth_age_s").value)
        depth = (
            self.latest_depth_m
            if time.monotonic() - self.latest_depth_time <= max_age
            else None
        )
        if depth is not None and depth.shape != rgb.shape[:2]:
            depth = None

        result = self.detector.detect(rgb, depth)
        command = self.controller.update(result)
        self.sender.send(command)

        twist = Twist()
        twist.linear.x = command.vx
        twist.linear.y = command.vy
        twist.angular.z = command.wz
        self.cmd_publisher.publish(twist)

        debug = self.detector.annotate(rgb, result)
        cv2.putText(
            debug,
            f"{command.state} vx={command.vx:.2f} wz={command.wz:+.3f}",
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

    def destroy_node(self):
        self.sender.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RealSenseLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
