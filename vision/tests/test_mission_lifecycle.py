from __future__ import annotations

import unittest

from g1_race_vision.controller import LaneFollowerController, Walk0p5mGate
from g1_race_vision.line_detector import WhiteLaneDetector
from g1_race_vision.mission_lifecycle import (
    apply_num7_mode_transition,
    mujoco_state_was_reset,
)


class FakeResettable:
    def __init__(self) -> None:
        self.reset_count = 0

    def reset(self) -> None:
        self.reset_count += 1


class FakeGate:
    def __init__(self) -> None:
        self.active = False
        self.activations: list[float | None] = []
        self.deactivation_count = 0

    def activate(self, now: float | None = None) -> None:
        self.active = True
        self.activations.append(now)

    def deactivate(self) -> None:
        self.active = False
        self.deactivation_count += 1


class Num7ModeLifecycleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.detector = FakeResettable()
        self.controller = FakeResettable()
        self.gate = FakeGate()

    def apply(self, mode: str, now: float) -> str | None:
        return apply_num7_mode_transition(
            mode,
            self.detector,
            self.controller,
            self.gate,
            now=now,
        )

    def test_rising_edge_resets_complete_visual_session(self) -> None:
        self.assertEqual(self.apply("WALK0P5M", 10.0), "enabled")
        self.assertEqual(self.detector.reset_count, 1)
        self.assertEqual(self.controller.reset_count, 1)
        self.assertEqual(self.gate.activations, [10.0])

    def test_enable_heartbeat_does_not_reset_lane_or_timer(self) -> None:
        self.apply("WALK0P5M", 10.0)
        self.assertIsNone(self.apply("WALK0P5M", 10.5))
        self.assertEqual(self.detector.reset_count, 1)
        self.assertEqual(self.controller.reset_count, 1)
        self.assertEqual(self.gate.activations, [10.0])

    def test_disable_reenable_creates_fresh_second_mission(self) -> None:
        self.apply("WALK0P5M", 10.0)
        self.assertEqual(self.apply("NONE", 11.0), "disabled")
        self.assertEqual(self.apply("WALK0P5M", 12.0), "enabled")
        self.assertEqual(self.detector.reset_count, 2)
        self.assertEqual(self.controller.reset_count, 2)
        self.assertEqual(self.gate.activations, [10.0, 12.0])
        self.assertEqual(self.gate.deactivation_count, 1)

    def test_real_components_clear_latched_identity_on_second_mission(self) -> None:
        detector = WhiteLaneDetector()
        controller = LaneFollowerController()
        gate = Walk0p5mGate()

        detector.configure_num7_lifecycle(True)
        detector.set_num7_mission_active(True)
        detector._lane_identity_lost = True
        controller._previous_vx = 0.50
        gate.activate(now=1.0)
        gate._distance_m = 0.75

        apply_num7_mode_transition(
            "NONE", detector, controller, gate, now=2.0
        )
        apply_num7_mode_transition(
            "WALK0P5M", detector, controller, gate, now=3.0
        )

        self.assertFalse(detector._lane_identity_lost)
        self.assertEqual(controller._previous_vx, 0.0)
        self.assertEqual(gate._distance_m, 0.0)
        self.assertTrue(gate.active)
        self.assertTrue(detector._num7_mission_active)
        self.assertEqual(detector._mission_state, "INITIAL_LOCK")

    def test_rising_edge_activates_num7_initial_lock(self) -> None:
        detector = WhiteLaneDetector()
        detector.configure_num7_lifecycle(True)
        controller = LaneFollowerController()
        gate = Walk0p5mGate()

        apply_num7_mode_transition(
            "WALK0P5M", detector, controller, gate, now=10.0
        )

        self.assertTrue(detector._num7_mission_active)
        self.assertEqual(detector._mission_state, "INITIAL_LOCK")

    def test_disable_returns_detector_to_preview(self) -> None:
        detector = WhiteLaneDetector()
        detector.configure_num7_lifecycle(True)
        detector.set_num7_mission_active(True)
        controller = LaneFollowerController()
        gate = Walk0p5mGate()
        gate.activate(now=1.0)

        apply_num7_mode_transition(
            "NONE", detector, controller, gate, now=2.0
        )

        self.assertFalse(detector._num7_mission_active)
        self.assertEqual(detector._mission_state, "PREVIEW")


class MujocoResetDetectionTest(unittest.TestCase):
    def test_detects_simulation_time_rewind(self) -> None:
        self.assertTrue(mujoco_state_was_reset(31.0, 0.0, 41.0, 41.0))

    def test_detects_position_teleport_when_time_is_preserved(self) -> None:
        self.assertTrue(mujoco_state_was_reset(31.0, 31.1, 41.0, 0.0))

    def test_normal_forward_motion_is_not_a_reset(self) -> None:
        self.assertFalse(mujoco_state_was_reset(31.0, 31.1, 41.0, 41.5))

    def test_small_gait_oscillation_is_not_a_reset(self) -> None:
        self.assertFalse(mujoco_state_was_reset(31.0, 31.1, 4.0, 3.8))

    def test_first_sample_is_not_a_reset(self) -> None:
        self.assertFalse(mujoco_state_was_reset(None, 0.0, None, 0.0))


if __name__ == "__main__":
    unittest.main()
