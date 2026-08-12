from __future__ import annotations

import unittest

from g1_race_vision.mission_lifecycle import mujoco_state_was_reset


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
