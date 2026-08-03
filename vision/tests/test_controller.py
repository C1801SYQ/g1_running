from __future__ import annotations

import unittest

from g1_race_vision.controller import LaneFollowerController
from g1_race_vision.line_detector import DetectionResult


class LaneFollowerControllerTest(unittest.TestCase):
    def test_lane_to_right_commands_right_turn(self):
        controller = LaneFollowerController()
        result = DetectionResult(
            valid=True,
            lateral_error=0.35,
            heading_error_rad=0.05,
            confidence=0.9,
        )
        command = controller.update(result, now=0.0)
        self.assertGreater(command.vx, 0.0)
        self.assertLess(command.wz, 0.0)

    def test_lane_to_left_commands_left_turn(self):
        controller = LaneFollowerController()
        result = DetectionResult(
            valid=True,
            lateral_error=-0.35,
            heading_error_rad=-0.05,
            confidence=0.9,
        )
        command = controller.update(result, now=0.0)
        self.assertGreater(command.wz, 0.0)

    def test_long_line_loss_stops(self):
        controller = LaneFollowerController()
        valid = DetectionResult(valid=True, confidence=1.0)
        controller.update(valid, now=0.0)
        stopped = controller.update(DetectionResult(valid=False), now=2.0)
        self.assertEqual(stopped.state, "FAILSAFE_STOP")
        self.assertEqual(stopped.vx, 0.0)

    def test_forward_speed_ramps_up(self):
        controller = LaneFollowerController()
        valid = DetectionResult(valid=True, confidence=1.0)
        first = controller.update(valid, now=0.0)
        second = controller.update(valid, now=0.1)
        self.assertGreater(first.vx, 0.0)
        self.assertLess(first.vx, second.vx)
        self.assertLess(second.vx, controller.config.cruise_speed_mps)

    def test_brief_dropout_holds_speed_and_does_not_stop(self):
        controller = LaneFollowerController()
        valid = DetectionResult(valid=True, confidence=1.0)
        for index in range(20):
            command = controller.update(valid, now=0.1 * index)

        lost = controller.update(DetectionResult(valid=False), now=1.95)

        self.assertEqual(lost.state, "VISION_DROPOUT_HEADING_HOLD")
        self.assertAlmostEqual(lost.vx, command.vx)
        self.assertGreater(lost.vx, controller.config.minimum_tracking_speed_mps)

    def test_sustained_dropout_decelerates_smoothly_then_stops(self):
        controller = LaneFollowerController()
        valid = DetectionResult(valid=True, confidence=1.0)
        for index in range(20):
            command = controller.update(valid, now=0.1 * index)

        speeds = []
        for index in range(1, 31):
            lost = controller.update(
                DetectionResult(valid=False), now=1.9 + 0.1 * index
            )
            speeds.append(lost.vx)

        per_step_limit = controller.config.max_forward_decel_mps2 * 0.1
        self.assertTrue(
            all(
                before - after <= per_step_limit + 1e-9
                for before, after in zip([command.vx] + speeds, speeds)
            )
        )
        self.assertEqual(lost.state, "FAILSAFE_STOP")
        self.assertEqual(lost.vx, 0.0)

    def test_finish_approach_keeps_speed_and_removes_yaw(self):
        controller = LaneFollowerController()
        turning = DetectionResult(
            valid=True,
            lateral_error=0.4,
            heading_error_rad=0.2,
            confidence=1.0,
        )
        for index in range(20):
            command = controller.update(turning, now=0.1 * index)

        finish = controller.hold_straight_for_finish(now=2.0)

        self.assertEqual(finish.state, "FINISH_APPROACH_HEADING_HOLD")
        self.assertGreater(finish.vx, 0.0)
        self.assertLess(abs(finish.wz), abs(command.wz))

    def test_post_finish_braking_starts_after_line_and_reaches_zero(self):
        controller = LaneFollowerController()
        valid = DetectionResult(valid=True, confidence=1.0)
        for index in range(20):
            running = controller.update(valid, now=0.1 * index)

        speeds = []
        for index in range(1, 20):
            braking = controller.brake_after_finish(now=1.9 + 0.1 * index)
            speeds.append(braking.vx)
            if braking.state == "POST_FINISH_STOP":
                break

        self.assertLess(speeds[0], running.vx)
        self.assertTrue(
            all(after <= before for before, after in zip(speeds, speeds[1:]))
        )
        self.assertEqual(braking.state, "POST_FINISH_STOP")
        self.assertEqual(braking.vx, 0.0)

    def test_post_finish_braking_keeps_visual_lane_correction(self):
        controller = LaneFollowerController()
        centered = DetectionResult(valid=True, confidence=1.0)
        controller.update(centered, now=0.0)
        lane_center_to_right = DetectionResult(
            valid=True,
            lateral_error=0.35,
            heading_error_rad=0.05,
            confidence=1.0,
        )

        braking = controller.brake_after_finish(
            result=lane_center_to_right,
            now=0.1,
            heading_hold_error_rad=0.0,
        )

        self.assertTrue(braking.perception_valid)
        self.assertLess(braking.wz, 0.0)

    def test_dropout_uses_imu_heading_hold_without_visual_inference(self):
        controller = LaneFollowerController()
        valid = DetectionResult(valid=True, confidence=1.0)
        controller.update(valid, now=0.0, heading_hold_error_rad=0.0)

        lost = controller.update(
            DetectionResult(valid=False),
            now=0.1,
            heading_hold_error_rad=0.2,
        )

        self.assertFalse(lost.perception_valid)
        self.assertLess(lost.wz, 0.0)
        self.assertGreater(lost.vx, 0.0)


if __name__ == "__main__":
    unittest.main()
