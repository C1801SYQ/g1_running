"""Command-distance gate for the running-backed Num7 visual mission."""

from __future__ import annotations

from dataclasses import dataclass

from .controller import ControllerCommand


@dataclass(frozen=True)
class RunningVisionRampConfig:
    """Smooth running-backed Num7 startup ramp.

    The ramp is intentionally separate from Skill 6's sprint ramp.  Each
    segment uses a smootherstep profile so velocity and acceleration return to
    zero at segment boundaries instead of producing a slope discontinuity.
    """

    # Running is deliberately brought in more slowly than the generic sprint
    # mission.  These are the cap-profile durations; the independent slew
    # limiter below remains authoritative if a camera callback is delayed.
    ramp_to_1_s: float = 2.00
    ramp_to_3_s: float = 3.00
    ramp_to_max_s: float = 5.00
    max_forward_accel_mps2: float = 0.80


def _smootherstep(alpha: float) -> float:
    alpha = min(max(float(alpha), 0.0), 1.0)
    return alpha**3 * (alpha * (alpha * 6.0 - 15.0) + 10.0)


def running_vision_speed_cap(
    elapsed_s: float,
    max_speed_mps: float,
    config: RunningVisionRampConfig | None = None,
) -> float:
    """Return the smooth running-backed Num7 speed cap."""

    cfg = config or RunningVisionRampConfig()
    elapsed = max(0.0, float(elapsed_s))
    max_speed = max(0.0, float(max_speed_mps))
    low_speed = min(1.0, max_speed)
    mid_speed = min(3.0, max_speed)

    if cfg.ramp_to_1_s > 0.0 and elapsed < cfg.ramp_to_1_s:
        return low_speed * _smootherstep(elapsed / cfg.ramp_to_1_s)
    elapsed -= max(0.0, cfg.ramp_to_1_s)

    if cfg.ramp_to_3_s > 0.0 and elapsed < cfg.ramp_to_3_s:
        alpha = _smootherstep(elapsed / cfg.ramp_to_3_s)
        return low_speed + (mid_speed - low_speed) * alpha
    elapsed -= max(0.0, cfg.ramp_to_3_s)

    if cfg.ramp_to_max_s > 0.0 and elapsed < cfg.ramp_to_max_s:
        alpha = _smootherstep(elapsed / cfg.ramp_to_max_s)
        return mid_speed + (max_speed - mid_speed) * alpha

    return max_speed


@dataclass(frozen=True)
class RunningVisionMissionConfig:
    target_distance_m: float = 110.0
    cruise_speed_mps: float = 2.5
    max_duration_s: float = 260.0
    ramp: RunningVisionRampConfig = RunningVisionRampConfig()
    max_step_s: float = 0.20


class RunningVisionMission:
    """Emit a bounded running command and a latched 110 m stop.

    The distance is intentionally a commanded-distance estimate.  The FSM
    owns the authoritative watchdog and returns to locomotion after the
    hard-stop packet, while this gate provides an independent Python-side
    target and timeout.
    """

    def __init__(
        self, config: RunningVisionMissionConfig | None = None
    ) -> None:
        self.config = config or RunningVisionMissionConfig()
        if self.config.target_distance_m <= 0.0:
            raise ValueError("target_distance_m must be positive")
        if self.config.cruise_speed_mps <= 0.0:
            raise ValueError("cruise_speed_mps must be positive")
        if self.config.max_duration_s <= 0.0:
            raise ValueError("max_duration_s must be positive")
        if self.config.ramp.max_forward_accel_mps2 <= 0.0:
            raise ValueError("max_forward_accel_mps2 must be positive")
        self.reset()

    def reset(self) -> None:
        self.enabled = False
        self.finished = False
        self.started_at: float | None = None
        self.last_update_at: float | None = None
        self.commanded_distance_m = 0.0
        self._previous_speed_mps = 0.0

    def enable(self, now: float) -> None:
        self.enabled = True
        self.finished = False
        self.started_at = float(now)
        self.last_update_at = float(now)
        self.commanded_distance_m = 0.0
        self._previous_speed_mps = 0.0

    def disable(self) -> None:
        self.reset()

    def update(
        self,
        now: float,
        visual_wz: float,
        *,
        perception_valid: bool,
        motion_allowed: bool,
        safety_reason: str,
    ) -> ControllerCommand:
        now = float(now)
        if not self.enabled:
            return ControllerCommand(
                0.0, 0.0, 0.0, "WAIT_FOR_RUNNING_VISION", perception_valid
            )
        if self.finished:
            return ControllerCommand(
                0.0,
                0.0,
                0.0,
                "RUNNING_VISION_110M_COMPLETE",
                perception_valid,
                True,
            )

        previous = self.last_update_at if self.last_update_at is not None else now
        dt = min(max(0.0, now - previous), self.config.max_step_s)
        self.last_update_at = now
        started = self.started_at if self.started_at is not None else now
        elapsed = max(0.0, now - started)
        if elapsed >= self.config.max_duration_s:
            self.finished = True
            return ControllerCommand(
                0.0,
                0.0,
                0.0,
                "RUNNING_VISION_TIMEOUT",
                perception_valid,
                True,
            )
        if not motion_allowed:
            self.finished = True
            return ControllerCommand(
                0.0, 0.0, 0.0, safety_reason, perception_valid, True
            )

        speed_cap = running_vision_speed_cap(
            elapsed, self.config.cruise_speed_mps, self.config.ramp
        )
        # The smootherstep cap is continuous under nominal camera timing, but
        # a delayed callback could otherwise evaluate far ahead and jump to
        # the full 5.1 m/s policy command in one packet.  Bound each command
        # increment independently so the policy always receives a gradual
        # acceleration profile.
        max_delta = (
            self.config.ramp.max_forward_accel_mps2 * dt
        )
        self._previous_speed_mps += min(
            max(speed_cap - self._previous_speed_mps, 0.0), max_delta
        )
        speed = self._previous_speed_mps
        self.commanded_distance_m += speed * dt
        if self.commanded_distance_m >= self.config.target_distance_m:
            self.finished = True
            return ControllerCommand(
                0.0,
                0.0,
                0.0,
                "RUNNING_VISION_110M_COMPLETE",
                perception_valid,
                True,
            )
        return ControllerCommand(
            speed,
            0.0,
            float(visual_wz),
            "RUNNING_VISION_FORWARD_LANE_CORRECTION"
            if perception_valid
            else "RUNNING_VISION_FORWARD_NO_LINE",
            perception_valid,
            False,
        )
