from __future__ import annotations

import time
import unittest

from g1_race_vision.controller import (
    ControllerCommand,
    Walk0p5mConfig,
    Walk0p5mGate,
)
from g1_race_vision.depth_safety import DepthSafetyGate


class Walk0p5mGateTest(unittest.TestCase):
    def make_gate(self, **overrides) -> Walk0p5mGate:
        config = Walk0p5mConfig(**overrides)
        return Walk0p5mGate(config)

    def test_clamps_speed_and_yaw(self) -> None:
        gate = self.make_gate()
        gate.activate(now=1.0)
        cmd = ControllerCommand(0.80, 0.5, 0.40, "T", True, False)
        out = gate.update(cmd, now=1.0)
        self.assertAlmostEqual(out.vx, 0.50, places=6)
        self.assertEqual(out.vy, 0.0)
        self.assertAlmostEqual(out.wz, 0.25, places=6)

    def test_vy_always_zero(self) -> None:
        gate = self.make_gate()
        gate.activate(now=1.0)
        cmd = ControllerCommand(0.05, 0.3, 0.0, "T", True, False)
        out = gate.update(cmd, now=1.0)
        self.assertEqual(out.vy, 0.0)

    def test_zero_before_first_valid_perception(self) -> None:
        gate = self.make_gate()
        gate.activate(now=1.0)
        cmd = ControllerCommand(0.10, 0.0, 0.0, "T", False, False)
        out = gate.update(cmd, now=1.0)
        self.assertEqual(out.vx, 0.0)
        self.assertEqual(out.hard_stop, False)

    def test_line_loss_after_lock_stops_with_hard_stop(self) -> None:
        gate = self.make_gate()
        gate.activate(now=1.0)
        gate.update(ControllerCommand(0.05, 0.0, 0.0, "T", True, False), now=1.0)
        out = gate.update(ControllerCommand(0.05, 0.0, 0.0, "T", False, False), now=1.1)
        self.assertEqual(out.vx, 0.0)
        self.assertTrue(out.hard_stop)

    def test_distance_completion_keeps_zero_and_hard_stop(self) -> None:
        gate = self.make_gate(target_distance_m=0.10, distance_scale=1.0)
        gate.activate(now=1.0)
        # Feed 0.10 m/s for ~1.1 s to exceed 0.10 m.
        now = 1.0
        last = None
        for _ in range(40):
            out = gate.update(
                ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now
            )
            last = out
            now += 1.0 / 30.0
            if last.state == "NUM7_DISTANCE":
                break
        self.assertEqual(last.state, "NUM7_DISTANCE")
        self.assertTrue(last.hard_stop)
        self.assertEqual(last.vx, 0.0)
        # After stop, all subsequent commands stay zero + hard_stop.
        out2 = gate.update(
            ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now + 1.0
        )
        self.assertEqual(out2.vx, 0.0)
        self.assertTrue(out2.hard_stop)

    def test_timeout_forces_zero(self) -> None:
        gate = self.make_gate(max_duration_s=0.5)
        gate.activate(now=1.0)
        last = None
        now = 1.0
        for _ in range(60):
            last = gate.update(
                ControllerCommand(0.05, 0.0, 0.0, "T", True, False), now=now
            )
            now += 1.0 / 30.0
            if last.state == "NUM7_TIMEOUT":
                break
        self.assertEqual(last.state, "NUM7_TIMEOUT")
        self.assertTrue(last.hard_stop)

    def test_depth_stop_states_emit_zero_and_hard_stop(self) -> None:
        gate = self.make_gate()
        gate.activate(now=1.0)
        gate.update(ControllerCommand(0.05, 0.0, 0.0, "T", True, False), now=1.0)
        depth_gate = DepthSafetyGate()
        # Missing depth must hard stop.
        safety = depth_gate.evaluate(None, 0.0)
        stopped = depth_gate.apply(
            ControllerCommand(0.05, 0.0, 0.0, "T", True, False), safety
        )
        self.assertEqual(stopped.reason if hasattr(stopped, "reason") else stopped.state, "DEPTH_MISSING")
        self.assertEqual(stopped.vx, 0.0)
        self.assertTrue(stopped.hard_stop)

    def test_obstacle_stop_sets_hard_stop(self) -> None:
        import numpy as np

        depth_gate = DepthSafetyGate()
        depth = np.full((48, 64), 2.0, dtype=np.float32)
        depth[:, 20:44] = 0.50  # close obstacle in the corridor
        safety = depth_gate.evaluate(depth, 0.0)
        self.assertEqual(safety.reason, "OBSTACLE_STOP")
        stopped = depth_gate.apply(
            ControllerCommand(0.05, 0.0, 0.0, "T", True, False), safety
        )
        self.assertTrue(stopped.hard_stop)
        self.assertEqual(stopped.vx, 0.0)


class Walk0p5mConfigTest(unittest.TestCase):
    def test_defaults(self) -> None:
        config = Walk0p5mConfig()
        self.assertEqual(config.max_vx_mps, 0.50)
        self.assertEqual(config.max_wz_rps, 0.25)
        self.assertEqual(config.target_distance_m, 1.00)
        self.assertEqual(config.distance_scale, 0.60)
        self.assertEqual(config.max_duration_s, 7.0)


class Walk0p5mGateLifecycleTest(unittest.TestCase):
    def test_inactive_before_activate_no_timeout_no_distance(self) -> None:
        """Gate constructed but never activated must not accumulate time or
        distance and must not emit a stale hard_stop (blocking issue 3)."""
        gate = Walk0p5mGate(Walk0p5mConfig(max_duration_s=0.1))
        last = None
        now = 1.0
        # Simulate 20 s of frames with valid perception but no FSM enable.
        for _ in range(600):
            last = gate.update(
                ControllerCommand(0.10, 0.0, 0.0, "T", True, False),
                now=now,
            )
            now += 1.0 / 30.0
        self.assertFalse(gate.active)
        self.assertEqual(last.state, "NUM7_INACTIVE")
        self.assertEqual(last.vx, 0.0)
        self.assertFalse(last.hard_stop)

    def test_timing_starts_only_after_activate(self) -> None:
        gate = Walk0p5mGate(Walk0p5mConfig(max_duration_s=0.2))
        now = 1.0
        # Run 5 s before activation with a would-be timeout config.
        for _ in range(150):
            gate.update(
                ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now
            )
            now += 1.0 / 30.0
        self.assertFalse(gate.active)
        # Activate now; the 0.2 s timeout must run from *this* point.
        gate.activate(now=now)
        self.assertTrue(gate.active)
        last = None
        for _ in range(30):
            last = gate.update(
                ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now
            )
            now += 1.0 / 30.0
            if last.state == "NUM7_TIMEOUT":
                break
        self.assertEqual(last.state, "NUM7_TIMEOUT")

    def test_disable_outputs_zero_continuously(self) -> None:
        gate = Walk0p5mGate()
        gate.activate(now=1.0)
        gate.update(
            ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=1.1
        )
        gate.deactivate()
        self.assertFalse(gate.active)
        last = gate.update(
            ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=1.2
        )
        self.assertEqual(last.vx, 0.0)
        self.assertEqual(last.state, "NUM7_INACTIVE")
        self.assertFalse(last.hard_stop)

    def test_enable_complete_disable_reenable_runs_second_mission(self) -> None:
        """enable -> complete -> disable -> enable must run a fresh task with
        no inherited distance/timeout/hard_stop (blocking issue 4)."""
        gate = Walk0p5mGate(Walk0p5mConfig(target_distance_m=0.05))
        now = 1.0

        # Mission 1: reach distance.
        gate.activate(now=now)
        last = None
        for _ in range(60):
            last = gate.update(
                ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now
            )
            now += 1.0 / 30.0
            if last.state == "NUM7_DISTANCE":
                break
        self.assertEqual(last.state, "NUM7_DISTANCE")
        self.assertTrue(last.hard_stop)

        # Deactivate and re-activate: must be a fresh mission.
        gate.deactivate()
        self.assertFalse(gate.active)
        gate.activate(now=now)
        self.assertTrue(gate.active)
        self.assertEqual(gate._distance_m, 0.0)
        first = gate.update(
            ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now
        )
        # Fresh mission must not immediately stop with a stale distance.
        self.assertNotEqual(first.state, "NUM7_DISTANCE")
        self.assertNotEqual(first.state, "NUM7_STOPPED")
        # And a second completion must happen only after fresh integration.
        last2 = None
        for _ in range(60):
            last2 = gate.update(
                ControllerCommand(0.10, 0.0, 0.0, "T", True, False), now=now
            )
            now += 1.0 / 30.0
            if last2.state == "NUM7_DISTANCE":
                break
        self.assertEqual(last2.state, "NUM7_DISTANCE")
        self.assertTrue(last2.hard_stop)


if __name__ == "__main__":
    unittest.main()
