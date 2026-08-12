#!/usr/bin/env python3
"""Display the live Num7 annotated camera topic without publishing commands."""

from __future__ import annotations

import os
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Num7CameraViewer(Node):
    def __init__(self) -> None:
        super().__init__("num7_camera_viewer")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.create_subscription(
            Image,
            "/g1/race/debug_image",
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message: Image) -> None:
        self.latest_frame = self.bridge.imgmsg_to_cv2(
            message, desired_encoding="bgr8"
        )


def main() -> int:
    if not os.environ.get("DISPLAY"):
        print(
            "DISPLAY is not set. Run this from a terminal opened inside the "
            "NoMachine desktop.",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = Num7CameraViewer()
    window = "Num7 live camera - Q/Esc to close"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.latest_frame is not None:
                cv2.imshow(window, node.latest_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
