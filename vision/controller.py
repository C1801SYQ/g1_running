from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .line_detector import DetectionResult


@dataclass(frozen=True)
class LaneFollowerConfig:
    cruise_speed_mps: float = 0.55
    minimum_tracking_speed_mps: float = 0.20
    # Strong correction authority: once a real departure is detected, pull back
    # toward the lane centre hard. The previous 0.65 gain + 0.10 rad heading
    # limit let a 0.4 m drift grow for ~16 m before the boundary stop kicked in.
    lateral_kp: float = 1.20
    # Keep outward derivative authority modest so gait/camera noise cannot
    # create steering spikes. Use stronger derivative damping only while
    # returning inward, where it releases steering before centre-line overrun.
    lateral_kd: float = 0.08
    lateral_inward_kd: float = 0.18
    heading_kp: float = 0.20
    use_visual_heading_correction: bool = False
    imu_heading_kp: float = 1.20
    error_filter_alpha: float = 0.32
    heading_filter_alpha: float = 0.16
    max_lateral_error_rate_per_s: float = 1.20
    max_visual_heading_rate_rps: float = 1.20
    max_visual_heading_error_rad: float = 0.35
    max_heading_imu_disagreement_rad: float = 0.25
    heading_confidence_floor: float = 0.45
    # The trained policy owns the straight run. Vision enters a bounded
    # correction mode only after leaving this corridor and stays quiet again
    # after returning to the smaller exit threshold (Schmitt hysteresis).
    #
    # Stability-first corridor. A steady 0.20--0.25 m offset is a real tracking
    # error on the physical robot, not harmless gait sway. Enter correction
    # early and keep it active until the robot is close to its locked path.
    correction_enter_lateral_error: float = 0.18
    correction_exit_lateral_error: float = 0.06
    # A real slow drift should not have to reach the wide static-offset gate.
    # Enter early only when an offset is already meaningful and keeps moving
    # outward in one direction for several frames. Gait sway reverses too often
    # to accumulate this confirmation time.
    predictive_enter_lateral_error: float = 0.08
    predictive_outward_rate_per_s: float = 0.04
    predictive_confirm_s: float = 0.12
    # Strong correction authority: once a real departure is detected, pull back
    # toward the lane centre hard. The previous 0.10 rad heading limit and 0.65
    # lateral gain let a 0.4 m drift grow for ~16 m before the boundary stop
    # kicked in; with a stronger correction the robot recentres much sooner and
    # the speed ramp stays mostly inactive.
    max_lane_correction_heading_rad: float = 0.18
    # Once correction is active, trade speed for a damped return. Keeping full
    # sprint speed while already moving inward caused overshoot on hardware.
    lateral_slow_start: float = 0.16
    lateral_full_slow: float = 0.50
    lateral_rate_slow_start_per_s: float = 0.12
    lateral_rate_full_slow_per_s: float = 0.45
    imu_heading_slow_start_rad: float = 0.05
    imu_heading_full_slow_rad: float = 0.20
    heading_slow_start_rad: float = 0.10
    heading_full_slow_rad: float = 0.35
    steering_slow_start_ratio: float = 0.65
    # Stability takes priority whenever the lane controller needs substantial
    # yaw authority. Full cruise returns only after the exit corridor is met.
    minimum_correction_speed_scale: float = 0.45
    minimum_saturated_speed_scale: float = 0.30
    minimum_speed_scale: float = 0.22
    yaw_saturation_ratio: float = 0.92
    yaw_saturation_slow_after_s: float = 0.18
    yaw_saturation_full_slow_s: float = 0.65
    max_forward_accel_mps2: float = 0.35
    max_forward_decel_mps2: float = 3.00
    max_yaw_rate_rps: float = 0.35
    max_yaw_accel_rps2: float = 1.50
    # A resumed/stalled camera callback must not spend the whole wall-clock gap
    # in one control update and jump directly to a large steering command.
    max_control_dt_s: float = 0.10
    # Retained for configuration compatibility. Visual availability no longer
    # gates forward motion: the running policy and IMU own the straight path,
    # while white lines only add bounded yaw correction when they are valid.
    hold_last_command_s: float = 0.35
    slow_after_lost_s: float = 0.90
    stop_after_lost_s: float = 1.50
    # If the camera drops out while a real lateral recovery is active, going
    # straight immediately preserves the *outward* body momentum and can put
    # the robot into the neighbouring lane before the paint is visible again.
    # Keep the last lane-safe target heading briefly, then blend it back to the
    # immutable IMU heading. Ordinary centred dropouts do not use this path.
    line_loss_recovery_trigger_m: float = 0.14
    line_loss_recovery_min_heading_rad: float = 0.14
    line_loss_recovery_hold_s: float = 1.20
    line_loss_recovery_decay_s: float = 0.80
    line_loss_recovery_speed_scale: float = 0.40
    # A confirmed identity loss means the original path is not currently
    # trustworthy. Continue moving so the gait stays settled, but never resume
    # full sprint until the detector has safely reacquired the original pair.
    identity_lost_speed_scale: float = 0.35


@dataclass(frozen=True)
class ControllerCommand:
    vx: float
    vy: float
    wz: float
    state: str
    perception_valid: bool
    hard_stop: bool = False


class LaneFollowerController:
    """Turns visual lane errors into conservative velocity commands."""

    def __init__(self, config: LaneFollowerConfig | None = None):
        self.config = config or LaneFollowerConfig()
        self._filtered_lateral = 0.0
        self._filtered_lateral_rate = 0.0
        self._filtered_heading = 0.0
        self._previous_lateral = 0.0
        self._previous_wz = 0.0
        self._previous_vx = 0.0
        self._yaw_saturation_duration_s = 0.0
        self._correction_direction = 0.0
        self._outward_drift_direction = 0.0
        self._outward_drift_duration_s = 0.0
        self._visual_heading_reference_rad = 0.0
        self._last_time: float | None = None
        self._last_valid_time: float | None = None
        self._line_loss_recovery_heading_rad: float | None = None

    def reset(self) -> None:
        self._filtered_lateral = 0.0
        self._filtered_lateral_rate = 0.0
        self._filtered_heading = 0.0
        self._previous_lateral = 0.0
        self._previous_wz = 0.0
        self._previous_vx = 0.0
        self._yaw_saturation_duration_s = 0.0
        self._correction_direction = 0.0
        self._outward_drift_direction = 0.0
        self._outward_drift_duration_s = 0.0
        self._visual_heading_reference_rad = 0.0
        self._last_time = None
        self._last_valid_time = None
        self._line_loss_recovery_heading_rad = None

    def reset_lane_guidance(self) -> None:
        """Forget the old lane after a legal 100 m obstacle pass.

        Forward-speed history is preserved so accepting the newly occupied
        lane cannot create a second, artificial stop after the bypass.
        """

        previous_vx = self._previous_vx
        last_time = self._last_time
        self.reset()
        self._previous_vx = previous_vx
        self._last_time = last_time

    def set_visual_heading_reference(self, heading_error_rad: float) -> None:
        """Zero a fixed camera/mounting vanishing-point bias at lane lock."""

        self._visual_heading_reference_rad = float(heading_error_rad)
        self._filtered_heading = 0.0

    def update(
        self,
        result: DetectionResult,
        now: float | None = None,
        heading_hold_error_rad: float | None = None,
    ) -> ControllerCommand:
        now = time.monotonic() if now is None else float(now)
        dt = self._time_step(now)
        body_heading_error = self._finite_heading_error(
            heading_hold_error_rad
        )

        if self._perception_is_usable(result):
            wz = self._visual_steering(
                result, dt, body_heading_error
            )
            self._last_valid_time = now
            self._line_loss_recovery_heading_rad = None

            correcting = self._correction_direction != 0.0
            direction = self._correction_direction
            outward_rate = max(
                0.0,
                direction * self._filtered_lateral_rate,
            )
            lateral_scale = (
                self._safety_scale(
                    abs(self._filtered_lateral),
                    self.config.lateral_slow_start,
                    self.config.lateral_full_slow,
                    self.config.minimum_correction_speed_scale,
                )
                if correcting
                else 1.0
            )
            lateral_rate_scale = (
                self._safety_scale(
                    outward_rate,
                    self.config.lateral_rate_slow_start_per_s,
                    self.config.lateral_rate_full_slow_per_s,
                    self.config.minimum_correction_speed_scale,
                )
                if correcting
                else 1.0
            )
            # Visual heading is diagnostic only during the straight corridor;
            # body yaw is held by the IMU reference captured at mission start.
            heading_scale = 1.0
            # During lane recovery a non-zero body heading is intentional. A
            # body angle between straight ahead and the requested recovery
            # angle is healthy progress, not an uncommanded heading error.
            desired_lane_heading = self._desired_lane_heading()
            unsafe_heading_error = (
                self._unsafe_heading_error(
                    body_heading_error,
                    desired_lane_heading,
                )
                if correcting
                else body_heading_error
            )
            imu_heading_scale = self._safety_scale(
                abs(unsafe_heading_error),
                self.config.imu_heading_slow_start_rad,
                self.config.imu_heading_full_slow_rad,
                self.config.minimum_speed_scale,
            )
            steering_ratio = abs(wz) / max(self.config.max_yaw_rate_rps, 1e-6)
            steering_scale = self._safety_scale(
                steering_ratio,
                self.config.steering_slow_start_ratio,
                1.0,
                self.config.minimum_correction_speed_scale,
            )
            saturation_scale = self._safety_scale(
                self._yaw_saturation_duration_s,
                self.config.yaw_saturation_slow_after_s,
                self.config.yaw_saturation_full_slow_s,
                self.config.minimum_saturated_speed_scale,
            )
            boundary_scale = (
                self.config.minimum_speed_scale
                if result.boundary_risk
                else 1.0
            )
            safety_scale = min(
                lateral_scale,
                lateral_rate_scale,
                heading_scale,
                imu_heading_scale,
                steering_scale,
                saturation_scale,
                boundary_scale,
            )
            vx = max(
                self.config.minimum_tracking_speed_mps,
                self.config.cruise_speed_mps * safety_scale,
            )
            vx = self._rate_limit_vx(vx, dt)
            state = (
                "LINE_FOLLOW_BOUNDARY_RECOVERY"
                if result.boundary_risk
                else "LINE_FOLLOW"
            )
            return ControllerCommand(vx, 0.0, wz, state, True)

        # White-line perception is correction-only. Before the first lock and
        # during any later dropout, keep following the immutable IMU heading
        # and ramp toward the policy's normal straight-line speed. Independent
        # depth, command-freshness, distance, timeout and E-stop gates remain
        # responsible for stopping the robot.
        lost_for_s = (
            0.0
            if self._last_valid_time is None
            else max(0.0, now - self._last_valid_time)
        )
        recovery_heading = self._line_loss_recovery_heading(lost_for_s)
        if recovery_heading is not None:
            wz = self._rate_limit(
                self._heading_target(
                    body_heading_error,
                    recovery_heading,
                ),
                dt,
            )
            target_vx = max(
                self.config.minimum_tracking_speed_mps,
                self.config.cruise_speed_mps
                * self.config.line_loss_recovery_speed_scale,
            )
            vx = self._rate_limit_vx(target_vx, dt)
            return ControllerCommand(
                vx,
                0.0,
                wz,
                "LINE_LOST_RECOVERY",
                False,
            )

        wz = self._rate_limit(
            self._heading_hold_target(body_heading_error), dt
        )
        target_vx = self.config.cruise_speed_mps
        if result.source == "lane-identity-lost":
            target_vx = max(
                self.config.minimum_tracking_speed_mps,
                self.config.cruise_speed_mps
                * self.config.identity_lost_speed_scale,
            )
        vx = self._rate_limit_vx(target_vx, dt)
        state = (
            "STRAIGHT_IMU_NO_LINE"
            if self._last_valid_time is None
            else (
                "STRAIGHT_IMU_IDENTITY_LOST"
                if result.source == "lane-identity-lost"
                else "STRAIGHT_IMU_LINE_LOST"
            )
        )
        return ControllerCommand(vx, 0.0, wz, state, False)

    def follow_lane_through_finish(
        self,
        result: DetectionResult,
        now: float | None = None,
        heading_hold_error_rad: float | None = None,
    ) -> ControllerCommand:
        """Keep normal two-line correction active until 100 m is crossed."""

        command = self.update(
            result,
            now=now,
            heading_hold_error_rad=heading_hold_error_rad,
        )
        return ControllerCommand(
            command.vx,
            command.vy,
            command.wz,
            "FINISH_APPROACH_LANE_FOLLOW",
            command.perception_valid,
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
        perception_valid = (
            result is not None and self._perception_is_usable(result)
        )
        body_heading_error = self._finite_heading_error(
            heading_hold_error_rad
        )
        if perception_valid:
            wz = self._visual_steering(
                result, dt, body_heading_error
            )
            self._last_valid_time = now
        else:
            wz = self._rate_limit(
                self._heading_hold_target(body_heading_error), dt
            )
        state = "POST_FINISH_STOP" if vx <= 1e-3 else "POST_FINISH_DECEL"
        return ControllerCommand(vx, 0.0, wz, state, perception_valid)

    def _visual_steering(
        self,
        result: DetectionResult,
        dt: float,
        heading_hold_error_rad: float | None,
    ) -> float:
        alpha = float(np.clip(self.config.error_filter_alpha, 0.0, 1.0))
        lateral_target = (
            alpha * result.lateral_error
            + (1.0 - alpha) * self._filtered_lateral
        )
        max_lateral_delta = self.config.max_lateral_error_rate_per_s * dt
        self._filtered_lateral += float(
            np.clip(
                lateral_target - self._filtered_lateral,
                -max_lateral_delta,
                max_lateral_delta,
            )
        )
        derivative_raw = (
            self._filtered_lateral - self._previous_lateral
        ) / dt
        self._filtered_lateral_rate = (
            0.18 * derivative_raw
            + 0.82 * self._filtered_lateral_rate
        )
        self._previous_lateral = self._filtered_lateral
        self._update_correction_mode(dt)
        self._update_filtered_heading(
            result,
            dt,
            heading_hold_error_rad,
        )
        # Treat the lane error as a bounded desired body heading.  Directly
        # mapping a large lateral error to yaw rate creates an under-damped
        # weave at sprint speed: the robot crosses the centre while still
        # carrying a large yaw angle.  The inner IMU loop damps that angle.
        desired_heading = self._desired_lane_heading()
        heading_error = (
            (heading_hold_error_rad or 0.0) - desired_heading
        )
        # Do not continuously steer from the camera's visual heading. The
        # trained running policy plus IMU heading hold produce the straight
        # trajectory; vision only supplies the bounded lateral recovery angle.
        visual_heading_wz = (
            -self.config.heading_kp * self._filtered_heading
            if self.config.use_visual_heading_correction
            else 0.0
        )
        raw_wz = (
            -(self.config.imu_heading_kp * heading_error)
            + visual_heading_wz
        )
        saturated = (
            abs(raw_wz)
            >= self.config.yaw_saturation_ratio
            * self.config.max_yaw_rate_rps
        )
        if saturated:
            self._yaw_saturation_duration_s += dt
        else:
            self._yaw_saturation_duration_s = max(
                0.0,
                self._yaw_saturation_duration_s - 2.0 * dt,
            )
        target_wz = float(
            np.clip(
                raw_wz,
                -self.config.max_yaw_rate_rps,
                self.config.max_yaw_rate_rps,
            )
        )
        return self._rate_limit(target_wz, dt)

    def _desired_lane_heading(self) -> float:
        if self._correction_direction == 0.0:
            return 0.0

        # Keep one correction direction until the robot is back inside the
        # exit corridor. This prevents left/right command chatter when gait
        # sway makes the measured centre cross zero on consecutive frames.
        signed_error = self._correction_direction * self._filtered_lateral
        lateral = max(
            0.0,
            signed_error - self.config.correction_exit_lateral_error,
        )
        # Keep the rate signed in the correction frame: outward motion adds
        # authority, while inward motion subtracts it and releases steering
        # before the robot crosses the centre corridor.
        lateral_rate = (
            self._correction_direction * self._filtered_lateral_rate
        )
        rate_gain = (
            self.config.lateral_kd
            if lateral_rate >= 0.0
            else self.config.lateral_inward_kd
        )
        lane_guidance = max(
            0.0,
            self.config.lateral_kp * lateral
            + rate_gain * lateral_rate,
        )
        return float(
            np.clip(
                -self._correction_direction
                * lane_guidance
                / max(self.config.imu_heading_kp, 1e-6),
                -self.config.max_lane_correction_heading_rad,
                self.config.max_lane_correction_heading_rad,
            )
        )

    def _update_correction_mode(self, dt: float) -> None:
        magnitude = abs(self._filtered_lateral)
        if self._correction_direction == 0.0:
            if magnitude >= self.config.correction_enter_lateral_error:
                self._correction_direction = float(
                    np.sign(self._filtered_lateral)
                )
                self._clear_outward_drift_confirmation()
                return

            direction = float(np.sign(self._filtered_lateral))
            outward_rate = direction * self._filtered_lateral_rate
            predictive = (
                direction != 0.0
                and magnitude >= self.config.predictive_enter_lateral_error
                and outward_rate
                >= self.config.predictive_outward_rate_per_s
            )
            if predictive:
                if direction != self._outward_drift_direction:
                    self._outward_drift_direction = direction
                    self._outward_drift_duration_s = 0.0
                self._outward_drift_duration_s += dt
                if (
                    self._outward_drift_duration_s
                    >= self.config.predictive_confirm_s
                ):
                    self._correction_direction = direction
                    self._clear_outward_drift_confirmation()
            else:
                # Decay faster than evidence accumulates so alternating gait
                # sway cannot stitch unrelated half-cycles into a correction.
                self._outward_drift_duration_s = max(
                    0.0,
                    self._outward_drift_duration_s - 2.0 * dt,
                )
                if self._outward_drift_duration_s == 0.0:
                    self._outward_drift_direction = 0.0
            return
        if magnitude <= self.config.correction_exit_lateral_error:
            self._correction_direction = 0.0
            self._clear_outward_drift_confirmation()

    def _clear_outward_drift_confirmation(self) -> None:
        self._outward_drift_direction = 0.0
        self._outward_drift_duration_s = 0.0

    def _update_filtered_heading(
        self,
        result: DetectionResult,
        dt: float,
        heading_hold_error_rad: float | None,
    ) -> None:
        """Reject camera-shake spikes and fuse visual heading with the IMU."""

        visual_heading = float(
            np.arctan2(
                np.sin(
                    result.heading_error_rad
                    - self._visual_heading_reference_rad
                ),
                np.cos(
                    result.heading_error_rad
                    - self._visual_heading_reference_rad
                ),
            )
        )
        imu_heading = (
            None
            if heading_hold_error_rad is None
            else float(heading_hold_error_rad)
        )
        visual_is_plausible = (
            result.confidence >= self.config.heading_confidence_floor
            and abs(visual_heading)
            <= self.config.max_visual_heading_error_rad
        )
        if (
            visual_is_plausible
            and imu_heading is not None
            and abs(visual_heading - imu_heading)
            > self.config.max_heading_imu_disagreement_rad
        ):
            visual_is_plausible = False

        if visual_is_plausible:
            measurement = visual_heading
        elif imu_heading is not None:
            measurement = imu_heading
        else:
            measurement = self._filtered_heading

        measurement = float(
            np.clip(
                measurement,
                -self.config.max_visual_heading_error_rad,
                self.config.max_visual_heading_error_rad,
            )
        )
        alpha = float(np.clip(self.config.heading_filter_alpha, 0.0, 1.0))
        target = (
            alpha * measurement
            + (1.0 - alpha) * self._filtered_heading
        )
        max_delta = self.config.max_visual_heading_rate_rps * dt
        self._filtered_heading += float(
            np.clip(
                target - self._filtered_heading,
                -max_delta,
                max_delta,
            )
        )

    @staticmethod
    def _perception_is_usable(result: DetectionResult) -> bool:
        """Reject malformed valid frames before they contaminate filters."""

        return bool(
            result.valid
            and np.isfinite(result.lateral_error)
            and np.isfinite(result.heading_error_rad)
            and np.isfinite(result.confidence)
        )

    @staticmethod
    def _finite_heading_error(heading_error_rad: float | None) -> float:
        if heading_error_rad is None:
            return 0.0
        value = float(heading_error_rad)
        return value if np.isfinite(value) else 0.0

    @staticmethod
    def _unsafe_heading_error(
        body_heading_error: float,
        desired_lane_heading: float,
    ) -> float:
        """Return only heading outside the intentional recovery envelope."""

        desired_direction = float(np.sign(desired_lane_heading))
        if desired_direction == 0.0:
            return float(body_heading_error)
        directed_body_heading = desired_direction * body_heading_error
        if 0.0 <= directed_body_heading <= abs(desired_lane_heading):
            return 0.0
        if directed_body_heading < 0.0:
            return float(body_heading_error)
        return float(
            desired_direction
            * (directed_body_heading - abs(desired_lane_heading))
        )

    @staticmethod
    def _safety_scale(
        value: float,
        slow_start: float,
        full_slow: float,
        minimum_scale: float,
    ) -> float:
        """Return a continuous 1..minimum_scale safety speed multiplier."""

        if value <= slow_start:
            return 1.0
        if full_slow <= slow_start:
            return float(minimum_scale)
        progress = float(
            np.clip(
                (value - slow_start) / (full_slow - slow_start),
                0.0,
                1.0,
            )
        )
        return float(1.0 - progress * (1.0 - minimum_scale))

    def _time_step(self, now: float) -> float:
        elapsed = (
            1.0 / 30.0
            if self._last_time is None
            else max(now - self._last_time, 1e-3)
        )
        self._last_time = now
        return min(elapsed, max(self.config.max_control_dt_s, 1e-3))

    def _heading_hold_target(
        self, heading_error_rad: float | None
    ) -> float:
        heading_error = self._finite_heading_error(heading_error_rad)
        return self._heading_target(heading_error, 0.0)

    def _heading_target(
        self,
        body_heading_error_rad: float,
        desired_heading_rad: float,
    ) -> float:
        return float(
            np.clip(
                -self.config.imu_heading_kp
                * (body_heading_error_rad - desired_heading_rad),
                -self.config.max_yaw_rate_rps,
                self.config.max_yaw_rate_rps,
            )
        )

    def _line_loss_recovery_heading(
        self,
        lost_for_s: float,
    ) -> float | None:
        """Preserve a bounded, last-known-safe correction through dropout."""

        if self._last_valid_time is None:
            return None

        if self._line_loss_recovery_heading_rad is None:
            direction = self._correction_direction
            if (
                direction == 0.0
                and abs(self._filtered_lateral)
                >= self.config.line_loss_recovery_trigger_m
            ):
                direction = float(np.sign(self._filtered_lateral))
            if direction == 0.0:
                return None

            desired = self._desired_lane_heading()
            minimum = self.config.line_loss_recovery_min_heading_rad
            if abs(desired) < minimum:
                desired = -direction * minimum
            self._line_loss_recovery_heading_rad = float(
                np.clip(
                    desired,
                    -self.config.max_lane_correction_heading_rad,
                    self.config.max_lane_correction_heading_rad,
                )
            )

        hold_s = max(0.0, self.config.line_loss_recovery_hold_s)
        decay_s = max(0.0, self.config.line_loss_recovery_decay_s)
        if lost_for_s <= hold_s:
            scale = 1.0
        elif decay_s > 0.0 and lost_for_s < hold_s + decay_s:
            scale = 1.0 - (lost_for_s - hold_s) / decay_s
        else:
            self._line_loss_recovery_heading_rad = None
            return None

        return float(self._line_loss_recovery_heading_rad * scale)

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


@dataclass(frozen=True)
class Walk0p5mConfig:
    """Python-side redundant safety limits for the Num7 1.0 m walk.

    ``Walk0p5m`` remains the protocol/class name for backward compatibility.
    """

    max_vx_mps: float = 0.50
    max_wz_rps: float = 0.25
    target_distance_m: float = 1.00
    # Open-loop calibration only; this is not measured odometry.
    distance_scale: float = 0.60
    max_duration_s: float = 7.0
    # Kept for compatibility with existing launch files. Line loss no longer
    # stops Num7; independent safety gates still latch hard stops.
    stop_after_lost_s: float = 0.30


class Walk0p5mGate:
    """Python redundancy for the Num7 mission.

    This is a secondary guard on top of the C++ FSM: it re-clamps vx/vy/wz,
    integrates the *sent* safe vx, latches zero + hard_stop on distance,
    timeout or an independent safety failure, and keeps sending zero after a
    stop. White-line validity is deliberately not a motion gate: the policy
    walks straight under IMU heading hold when vision is unavailable.

    The gate has an explicit lifecycle. It does NOT start counting when
    constructed: time and distance accumulate only after ``activate()`` is
    called, which happens once the C++ FSM reports ``G1_VISION_WALK_0P5M 1``.
    Before activation the gate outputs zero and never inherits a previous
    mission's hard_stop / distance / timeout.
    """

    def __init__(self, config: Walk0p5mConfig | None = None) -> None:
        self.config = config or Walk0p5mConfig()
        self.reset()

    @property
    def active(self) -> bool:
        return self._active

    def reset(self) -> None:
        self._active = False
        self._started = False
        self._stopped = False
        self._distance_m = 0.0
        self._start_time: float | None = None
        self._last_time: float | None = None

    def activate(self, now: float | None = None) -> None:
        """Start a fresh Num7 mission. Time/distance reset to zero."""
        self._active = True
        self._started = False
        self._stopped = False
        self._distance_m = 0.0
        now = time.monotonic() if now is None else float(now)
        self._start_time = now
        self._last_time = None

    def deactivate(self) -> None:
        """Leave the Num7 mission: output zero, clear all task state so the
        next activation is a brand-new mission."""
        self._active = False
        self.reset()

    def update(
        self,
        desired: ControllerCommand,
        now: float | None = None,
    ) -> ControllerCommand:
        now = time.monotonic() if now is None else float(now)

        if not self._active:
            # Not in a Num7 mission yet: hold zero, do not accumulate time or
            # distance, and do not emit a hard_stop that could be inherited by
            # a future session.
            return ControllerCommand(0.0, 0.0, 0.0, "NUM7_INACTIVE", False, False)

        dt = (
            1.0 / 30.0
            if self._last_time is None
            else max(now - self._last_time, 1e-3)
        )
        self._last_time = now

        if self._start_time is None:
            self._start_time = now

        if self._stopped:
            # Keep sending zero + hard_stop until the FSM exits the state.
            return ControllerCommand(0.0, 0.0, 0.0, "NUM7_STOPPED", False, True)

        # Re-clamp to the Num7 hard limits.
        vx = float(np.clip(desired.vx, 0.0, self.config.max_vx_mps))
        vz = float(
            np.clip(
                desired.wz,
                -self.config.max_wz_rps,
                self.config.max_wz_rps,
            )
        )
        if desired.hard_stop:
            self._stopped = True
            return ControllerCommand(
                0.0,
                0.0,
                0.0,
                desired.state,
                desired.perception_valid,
                True,
            )

        clamped = ControllerCommand(
            vx, 0.0, vz, desired.state, desired.perception_valid, False
        )

        # Integrate the sent safe vx (Python redundancy).
        if self._started:
            self._distance_m += vx * dt * self.config.distance_scale
        elif vx > 0.0:
            self._started = True
            self._distance_m += vx * dt * self.config.distance_scale

        if self._distance_m >= self.config.target_distance_m:
            self._stopped = True
            return ControllerCommand(0.0, 0.0, 0.0, "NUM7_DISTANCE", True, True)

        if now - self._start_time >= self.config.max_duration_s:
            self._stopped = True
            return ControllerCommand(0.0, 0.0, 0.0, "NUM7_TIMEOUT", True, True)

        return clamped
