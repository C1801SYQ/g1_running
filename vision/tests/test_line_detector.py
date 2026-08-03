from __future__ import annotations

import unittest

import cv2
import numpy as np

from g1_race_vision.line_detector import LaneDetectorConfig, WhiteLaneDetector


def synthetic_lane(shift_px: int = 0, width: int = 640, height: int = 480) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (85, 28, 28)
    left_bottom = (95 + shift_px, height - 1)
    left_top = (255 + shift_px, 175)
    right_bottom = (545 + shift_px, height - 1)
    right_top = (385 + shift_px, 175)
    cv2.line(image, left_bottom, left_top, (245, 245, 245), 14)
    cv2.line(image, right_bottom, right_top, (245, 245, 245), 14)
    return image


class WhiteLaneDetectorTest(unittest.TestCase):
    def test_centered_lane(self):
        result = WhiteLaneDetector().detect(synthetic_lane())
        self.assertTrue(result.valid)
        self.assertLess(abs(result.lateral_error), 0.04)
        self.assertLess(abs(result.heading_error_rad), 0.04)

    def test_lane_center_to_right_is_positive(self):
        result = WhiteLaneDetector().detect(synthetic_lane(shift_px=70))
        self.assertTrue(result.valid)
        self.assertGreater(result.lateral_error, 0.15)

    def test_lane_center_to_left_is_negative(self):
        result = WhiteLaneDetector().detect(synthetic_lane(shift_px=-70))
        self.assertTrue(result.valid)
        self.assertLess(result.lateral_error, -0.15)

    def test_one_remaining_line_is_invalid_in_strict_two_line_mode(self):
        image = synthetic_lane()
        depth = np.full(image.shape[:2], 3.0, dtype=np.float32)
        depth[:, : image.shape[1] // 2] = 0.10
        detector = WhiteLaneDetector()
        detector.detect(synthetic_lane())
        result = detector.detect(image, depth)
        self.assertFalse(result.valid)
        self.assertEqual(result.source, "none")
        self.assertIsNone(result.left_line)
        self.assertIsNone(result.right_line)

    def test_empty_image_is_invalid(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = WhiteLaneDetector().detect(image)
        self.assertFalse(result.valid)

    def test_impossible_heading_pair_is_rejected(self):
        detector = WhiteLaneDetector(
            LaneDetectorConfig(max_abs_heading_error_rad=0.05)
        )
        result = detector.detect(synthetic_lane(shift_px=70))
        self.assertFalse(result.valid)

    def test_locked_lane_does_not_jump_to_adjacent_pair(self):
        detector = WhiteLaneDetector(
            LaneDetectorConfig(max_boundary_step_ratio=0.18)
        )
        self.assertTrue(detector.detect(synthetic_lane()).valid)

        adjacent_lane = synthetic_lane(shift_px=190)
        result = detector.detect(adjacent_lane)

        self.assertFalse(result.valid)
        self.assertEqual(result.source, "none")

    def test_reset_allows_selecting_a_different_lane(self):
        detector = WhiteLaneDetector(
            LaneDetectorConfig(max_boundary_step_ratio=0.18)
        )
        self.assertTrue(detector.detect(synthetic_lane()).valid)
        self.assertFalse(detector.detect(synthetic_lane(shift_px=190)).valid)

        detector.reset()

        self.assertTrue(detector.detect(synthetic_lane(shift_px=190)).valid)


if __name__ == "__main__":
    unittest.main()
