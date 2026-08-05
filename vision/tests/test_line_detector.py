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


def horizontal_motion_blur(image: np.ndarray, width: int = 25) -> np.ndarray:
    kernel = np.full((1, width), 1.0 / width, dtype=np.float32)
    return cv2.filter2D(image, -1, kernel)


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

    def test_low_light_and_color_cast_remain_detectable(self):
        image = synthetic_lane().astype(np.float32)
        dim_warm = np.clip(
            image * np.asarray([0.42, 0.34, 0.30], dtype=np.float32),
            0,
            255,
        ).astype(np.uint8)

        result = WhiteLaneDetector().detect(dim_warm)

        self.assertTrue(result.valid)
        self.assertEqual(result.source, "two-lines")
        self.assertLess(result.value_threshold, 100.0)

    def test_abrupt_shadow_after_bright_frame_does_not_drop_pair(self):
        detector = WhiteLaneDetector()
        self.assertTrue(detector.detect(synthetic_lane()).valid)
        shadow = np.clip(
            synthetic_lane().astype(np.float32) * 0.28, 0, 255
        ).astype(np.uint8)

        result = detector.detect(shadow)

        self.assertTrue(result.valid)
        self.assertLess(abs(result.lateral_error), 0.04)

    def test_horizontal_motion_blur_remains_detectable(self):
        result = WhiteLaneDetector().detect(
            horizontal_motion_blur(synthetic_lane())
        )

        self.assertTrue(result.valid)
        self.assertLess(abs(result.lateral_error), 0.05)

    def test_locked_pair_survives_large_common_camera_shake(self):
        detector = WhiteLaneDetector()
        self.assertTrue(detector.detect(synthetic_lane()).valid)

        right = detector.detect(synthetic_lane(shift_px=110))
        left = detector.detect(synthetic_lane(shift_px=-110))
        recovered = detector.detect(synthetic_lane())

        self.assertTrue(right.valid)
        self.assertTrue(left.valid)
        self.assertTrue(recovered.valid)
        self.assertGreater(right.lateral_error, 0.25)
        self.assertLess(left.lateral_error, -0.25)

    def test_fragmented_locked_boundaries_are_refit_from_real_pixels(self):
        detector = WhiteLaneDetector()
        image = synthetic_lane()
        self.assertTrue(detector.detect(image).valid)
        fragmented = image.copy()
        fragmented[225:260, :] = (85, 28, 28)
        fragmented[300:335, :] = (85, 28, 28)
        fragmented[375:410, :] = (85, 28, 28)
        fragmented[450:, :] = (85, 28, 28)

        result = detector.detect(fragmented)

        self.assertTrue(result.valid)
        self.assertEqual(result.source, "two-lines-guided")
        self.assertIsNotNone(result.left_line)
        self.assertIsNotNone(result.right_line)

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
