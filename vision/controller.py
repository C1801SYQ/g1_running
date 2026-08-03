from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .line_detector import DetectionResult


@dataclass(frozen=True)
class LaneFollowerConfig:
    cruise_speed_mps: float = 0.55
    minimum_tracking_speed_mps: float = 0.20
    lateral_kp: float = 1.25
    lateral_kd: float = 0.02
    heading_kp: float = 0.20
    imu_heading_kp: float = 1.10
    error_filter_alpha: float = 0.45
    max_forward_accel_mps2: float = 0.35
    max_forward_decel_mps2: float = 1.20
    max_yaw_rate_rps: float = 0.48
    max_yaw_accel_rps2: float = 3.00
    hold_last_command_s: float = 0.35
    slow_after_lost_s: float = 0.90
    stop_after_lost_s: float = 1.50


@dataclass(frozen=True)
class ControllerCommand:
    vx: float
    vy: float
    wz: float
    state: str
    perception_valid: bool


class LaneFollowerController:
    """Turns visual lane errors into conservative velocity commands."""

    def __init__(self, config: LaneFollowerConfig | None = None):
        self.config = config or LaneFollowerConfig()
        self._filtered_lateral = 0.0
        self._previous_lateral = 0.0
        self._previous_wz = 0.0
        self._previous_vx = 0.0
        self._last_time: float | None = None
        self._last_valid_time: float | None = None

    def reset(self) -> None:
        self._filtered_lateral = 0.0
        self._previous_lateral = 0.0
        self._previous_wz = 0.0
        self._previous_vx = 0.0
        self._last_time = None
        self._last_valid_time = None

    def update(
        self,
        result: DetectionResult,
        now: float | None = None,
        heading_hold_error_rad: float | None = None,
    ) -> ControllerCommand:
        now = time.monotonic() if now is None else float(now)
        dt = self._time_step(now)

        if result.valid:
            wz = self._visual_steering(
                result, dt, heading_hold_error_rad
            )
            self._last_valid_time = now

            confidence_scale = 0.78 + 0.22 * float(
                np.clip(result.confidence, 0.0, 1.0)
            )
            error_scale = float(
                np.clip(
                    1.0
                    - 0.18 * abs(self._filtered_lateral)
                    - 0.10 * abs(result.heading_error_rad),
                    0.75,
                    1.0,
                )
            )
            vx = max(
                self.config.minimum_tracking_speed_mps,
                self.config.cruise_speed_mps * confidence_scale * error_scale,
            )
            vx = self._rate_limit_vx(vx, dt)
            return ControllerCommand(vx, 0.0, wz, "LINE_FOLLOW", True)

        if self._last_valid_time is None:
            self._previous_wz = self._rate_limit(
                self._heading_hold_target(heading_hold_error_rad), dt
            )
            vx = self._rate_limit_vx(0.0, dt)
            return ControllerCommand(vx, 0.0, self._previous_wz, "WAIT_FOR_LINE", False)

        lost_for = now - self._last_valid_time
        if lost_for <= self.config.hold_last_command_s:
            # Zero yaw-rate means keep the current body heading.  Forward speed
            # is held briefly, preventing a one-frame vision dropout from
            # causing the locomotion policy to stumble.
            wz = self._rate_limit(
                self._heading_hold_target(heading_hold_error_rad), dt
            )
            return ControllerCommand(
                self._previous_vx,
                0.0,
                wz,
                "VISION_DROPOUT_HEADING_HOLD",
                False,
            )
        if lost_for <= self.config.slow_after_lost_s:
            wz = self._rate_limit(
                self._heading_hold_target(heading_hold_error_rad), dt
            )
            vx = self._rate_limit_vx(
                self.config.minimum_tracking_speed_mps, dt
            )
            return ControllerCommand(
                vx,
                0.0,
                wz,
                "VISION_DROPOUT_SLOW",
                False,
            )
        if lost_for <= self.config.stop_after_lost_s:
            wz = self._rate_limit(
                self._heading_hold_target(heading_hold_error_rad), dt
            )
            vx = self._rate_limit_vx(0.0, dt)
            return ControllerCommand(
                vx,
                0.0,
                wz,
                "VISION_DROPOUT_BRAKE",
                False,
            )

        wz = self._rate_limit(
            self._heading_hold_target(heading_hold_error_rad), dt
        )
        vx = self._rate_limit_vx(0.0, dt)
        state = "FAILSAFE_STOP" if vx <= 1e-3 else "FAILSAFE_BRAKE"
        return ControllerCommand(vx, 0.0, wz, state, False)

    def hold_straight_for_finish(
        self,
        now: float | None = None,
        heading_hold_error_rad: float | None = None,
    ) -> ControllerCommand:
        """Ignore finish-stripe pixels and hold speed with zero yaw-rate."""

        now = time.monotonic() if now is None else float(now)
        dt = self._time_step(now)
        vx = self._rate_limit_vx(self.config.cruise_speed_mps, dt)
        wz = self._rate_limit(
            self._heading_hold_target(heading_hold_error_rad), dt
        )
        return ControllerCommand(
            vx, 0.0, wz, "FINISH_APPROACH_HEADING_HOLD", False
        )

    def brake_after_finish(
        self,
        result: DetectionResult | None = None,
        now: float | None = None,
        heading_hold_error_rad: float | None = None,
    ) -> ControllerCommand:
        """Decelerate after 100 m while still following both lane lines."""

        now = time.monotonic() if now is None else float(now)
        dt = self._time_step(now)
        vx = self._rate_limit_vx(0.0, dt)
        perception_valid = result is not None and result.valid
        if perception_valid:
            wz = self._visual_steering(
                result, dt, heading_hold_error_rad
            )
            self._last_valid_time = now
        else:
            wz = self._rate_limit(
                self._heading_hold_target(heading_hold_error_rad), dt
            )
        state = "POST_FINISH_STOP" if vx <= 1e-3 else "POST_FINISH_DECEL"
        return ControllerCommand(vx, 0.0, wz, state, perception_valid)

    def _visual_steering(
        self,
        result: DetectionResult,
        dt: float,
        heading_hold_error_rad: float | None,
    ) -> float:
        alpha = self.config.error_filter_alpha
        self._filtered_lateral = (
            alpha * result.lateral_error
            + (1.0 - alpha) * self._filtered_lateral
        )
        derivative = (
            self._filtered_lateral - self._previous_lateral
        ) / dt
        self._previous_lateral = self._filtered_lateral
        raw_wz = -(
            self.config.lateral_kp * self._filtered_lateral
            + self.config.lateral_kd * derivative
            + self.config.heading_kp * result.heading_error_rad
            + self.config.imu_heading_kp
            * (heading_hold_error_rad or 0.0)
        )
        target_wz = float(
            np.clip(
                raw_wz,
                -self.config.max_yaw_rate_rps,
                self.config.max_yaw_rate_rps,
            )
        )
        return self._rate_limit(target_wz, dt)

    def _time_step(self, now: float) -> float:
        dt = (
            1.0 / 30.0
            if self._last_time is None
            else max(now - self._last_time, 1e-3)
        )
        self._last_time = now
        return dt

    def _heading_hold_target(
        self, heading_error_rad: float | None
    ) -> float:
        if heading_error_rad is None:
            return 0.0
        return float(
            np.clip(
                -self.config.imu_heading_kp * heading_error_rad,
                -self.config.max_yaw_rate_rps,
                self.config.max_yaw_rate_rps,
            )
        )

    def _rate_limit(self, target_wz: float, dt: float) -> float:
        max_delta = self.config.max_yaw_accel_rps2 * dt
        delta = float(np.clip(target_wz - self._previous_wz, -max_delta, max_delta))
        self._previous_wz += delta
        return self._previous_wz

    def _rate_limit_vx(self, target_vx: float, dt: float) -> float:
        rate = (
            self.config.max_forward_accel_mps2
            if target_vx >= self._previous_vx
            else self.config.max_forward_decel_mps2
        )
        max_delta = rate * dt
        self._previous_vx += float(
            np.clip(target_vx - self._previous_vx, -max_delta, max_delta)
        )
        return self._previous_vx
