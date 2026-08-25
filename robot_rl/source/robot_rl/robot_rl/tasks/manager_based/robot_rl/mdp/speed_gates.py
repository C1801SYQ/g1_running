# Copyright 2026 Zachary Olkin. All rights reserved.

"""Velocity-conditioned reward gates.

This module intentionally imports only ``torch`` (no isaaclab, no omni) so the
gating logic can be unit-tested in a plain Python environment.  Everything here
operates on the *actual* velocity command the policy receives (i.e. the
``max_acc``-limited command), never on ``is_standing_env`` or any other
instantaneous mode label.

The continuous gates replace the rectangular / ``is_standing_env`` hard switches
that caused the gait_v3 start-line twitch (see the sprint deployment notes).
"""

from __future__ import annotations

import torch


def _assert_increasing_edges(edge0: float, edge1: float, name: str) -> None:
    """Fail fast on invalid smooth gate edges before they create NaNs."""
    if not edge0 < edge1:
        raise ValueError(f"{name} requires edge0 < edge1, got {edge0!r} >= {edge1!r}.")


def smoothstep(x: torch.Tensor, edge0: float = 0.0, edge1: float = 1.0) -> torch.Tensor:
    """Vectorized cubic smoothstep of ``x`` between ``edge0`` and ``edge1``.

    Returns 0 for ``x <= edge0``, 1 for ``x >= edge1`` and a C1-continuous ramp
    in between.  The output is monotone non-decreasing in ``x``.

    Args:
        x: Input tensor of any shape.
        edge0: Lower edge of the smooth ramp.
        edge1: Upper edge of the smooth ramp.

    Returns:
        Tensor of the same shape as ``x`` in ``[0, 1]``.
    """
    _assert_increasing_edges(edge0, edge1, "smoothstep")
    t = torch.clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def command_speed(
    command: torch.Tensor,
    weights: tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> torch.Tensor:
    """Weighted magnitude of a velocity command ``[vx, vy, yaw]``.

    The different physical units (m/s vs rad/s) must not be naively maxed
    together; the ``weights`` let the caller normalize each axis.  A weight of 0
    drops an axis entirely.

    Args:
        command: Tensor of shape ``[..., 3]`` holding ``[vx, vy, yaw]``.
        weights: Per-axis scaling ``(w_vx, w_vy, w_yaw)``.

    Returns:
        Tensor of shape ``[...]`` with the weighted norm per environment.
    """
    w = torch.as_tensor(weights, dtype=command.dtype, device=command.device)
    w = w.reshape((1,) * (command.ndim - 1) + (3,))
    return torch.sqrt(torch.sum((command * w) ** 2, dim=-1))


def smooth_speed_gate(
    speed: torch.Tensor,
    low: float = 0.08,
    high: float = 0.60,
) -> torch.Tensor:
    """Standing-weight gate: 1 for ``speed <= low``, 0 for ``speed >= high``.

    Monotone non-increasing, C1-continuous everywhere (no step at the band
    edges).  ``low``/``high`` are expressed in m/s of the (weighted) command
    magnitude.

    Args:
        speed: Scalar speed tensor of shape ``[...]``.
        low: Speed at or below which the standing weight is full (1.0).
        high: Speed at or above which the standing weight is zero.

    Returns:
        Gate tensor of shape ``[...]`` in ``[0, 1]``.
    """
    return 1.0 - smoothstep(speed, low, high)


def smooth_high_speed_gate(
    speed: torch.Tensor,
    low: float = 3.0,
    high: float = 4.0,
) -> torch.Tensor:
    """High-speed gate: 0 for ``speed <= low``, 1 for ``speed >= high``.

    Monotone non-decreasing, C1-continuous everywhere.  Used by the sprint
    rewards (``forward_progress``, ``high_speed_stability``) and the
    lateral/heading fallback path, which must be *active* at high speed (the
    opposite of ``smooth_speed_gate``).

    Args:
        speed: Scalar speed tensor of shape ``[...]``.
        low: Speed at or below which the gate is zero.
        high: Speed at or above which the gate is one.

    Returns:
        Gate tensor of shape ``[...]`` in ``[0, 1]``.
    """
    return smoothstep(speed, low, high)


def smooth_band_gate(
    speed: torch.Tensor,
    rise_low: float,
    rise_high: float,
    fall_low: float,
    fall_high: float,
) -> torch.Tensor:
    """Smooth band gate: 0 outside ``[rise_high, fall_low]``, 1 inside.

    Replaces the rectangular 0.3--1.5 m/s mask used by ``slow_speed_hip_yaw``.
    The rising and falling edges are smoothstep ramps, so there is no reward
    step at the band boundaries.

    Args:
        speed: Scalar speed tensor of shape ``[...]``.
        rise_low: Speed below which the gate is zero.
        rise_high: Speed at/above which the gate reaches its plateau (1.0).
        fall_low: Speed at/above which the gate starts falling from 1.0.
        fall_high: Speed at/above which the gate is zero.

    Returns:
        Gate tensor of shape ``[...]`` in ``[0, 1]``.
    """
    _assert_increasing_edges(rise_low, rise_high, "smooth_band_gate rise")
    _assert_increasing_edges(fall_low, fall_high, "smooth_band_gate fall")
    if rise_high > fall_low:
        raise ValueError(
            "smooth_band_gate requires rise_high <= fall_low for a non-overlapping plateau, "
            f"got {rise_high!r} > {fall_low!r}."
        )
    rise = smoothstep(speed, rise_low, rise_high)
    fall = 1.0 - smoothstep(speed, fall_low, fall_high)
    return torch.minimum(rise, fall)


def phase_frequency_from_speed(
    speed: torch.Tensor,
    freq_max: float = 3.0,
    speed_max: float = 6.0,
    freq_min: float = 0.0,
) -> torch.Tensor:
    """Map forward speed to a gait phase frequency (Hz).

    This is the speed-conditioned frequency mapping required by the SPRINT
    spectral-prior idea.  At ``speed == 0`` the frequency is ``freq_min``
    (ideally 0), and it grows linearly toward ``freq_max`` at ``speed_max``.

    NOTE: this helper is currently *not* wired into the trajectory phase
    integrator; doing so changes the phase observation and breaks checkpoint
compatibility (see the sprint deployment notes).  It is provided and tested
    so a future feature-flagged implementation can reuse it.

    Args:
        speed: Scalar forward speed tensor of shape ``[...]``.
        freq_max: Phase frequency (Hz) at ``speed_max``.
        speed_max: Speed at which ``freq_max`` is reached.
        freq_min: Phase frequency (Hz) at zero speed.

    Returns:
        Frequency tensor of shape ``[...]`` in ``[freq_min, freq_max]``.
    """
    frac = torch.clamp(speed / speed_max, 0.0, 1.0)
    return freq_min + (freq_max - freq_min) * frac


def forward_distance(
    pos: torch.Tensor,
    start_pos: torch.Tensor,
    start_heading: torch.Tensor,
) -> torch.Tensor:
    """Projected forward distance along the track (start heading) direction.

    Args:
        pos: Current position of shape ``[..., 3]``.
        start_pos: Episode-start position of shape ``[..., 3]``.
        start_heading: Episode-start yaw (rad) of shape ``[...]``.

    Returns:
        Scalar forward distance (m) of shape ``[...]``.
    """
    dx = pos[..., 0] - start_pos[..., 0]
    dy = pos[..., 1] - start_pos[..., 1]
    return dx * torch.cos(start_heading) + dy * torch.sin(start_heading)


def lateral_distance(
    pos: torch.Tensor,
    start_pos: torch.Tensor,
    start_heading: torch.Tensor,
) -> torch.Tensor:
    """Projected lateral deviation from the track centerline.

    Args:
        pos: Current position of shape ``[..., 3]``.
        start_pos: Episode-start position of shape ``[..., 3]``.
        start_heading: Episode-start yaw (rad) of shape ``[...]``.

    Returns:
        Scalar signed lateral error (m) of shape ``[...]``.
    """
    dx = pos[..., 0] - start_pos[..., 0]
    dy = pos[..., 1] - start_pos[..., 1]
    return -dx * torch.sin(start_heading) + dy * torch.cos(start_heading)


def heading_error_relative(
    current_heading: torch.Tensor,
    start_heading: torch.Tensor,
) -> torch.Tensor:
    """Heading error relative to the track direction, wrapped to ``[-pi, pi]``.

    Args:
        current_heading: Current yaw (rad) of shape ``[...]``.
        start_heading: Episode-start yaw (rad) of shape ``[...]``.

    Returns:
        Signed heading error (rad) of shape ``[...]``.
    """
    return torch.atan2(
        torch.sin(current_heading - start_heading),
        torch.cos(current_heading - start_heading),
    )


def forward_progress_value(
    v_track: torch.Tensor,
    command_vx: torch.Tensor,
    v_scale: float = 5.0,
    gate_low: float = 3.0,
    gate_high: float = 4.0,
) -> torch.Tensor:
    """Forward-progress reward value: high-speed-gated ``tanh(v_track / v_scale)``.

    This is the exact computation used by ``forward_progress_reward`` so the
    reward's gating semantics can be unit-tested without isaaclab.  At command
    speeds >= ``gate_high`` the gate is 1 and forward velocity is rewarded; at
    command speeds <= ``gate_low`` the term is inactive.

    Args:
        v_track: Forward velocity projected onto the track direction.
        command_vx: Commanded forward speed (magnitude).
        v_scale: Speed at which ``tanh`` saturates.
        gate_low: Speed below which the term is inactive.
        gate_high: Speed at/above which the term is fully active.

    Returns:
        Reward tensor of shape ``[...]``.
    """
    gate = smooth_high_speed_gate(torch.abs(command_vx), gate_low, gate_high)
    return gate * torch.tanh(v_track / v_scale)


def reset_episode_command_state(
    current_vel_b: torch.Tensor,
    vel_command_b: torch.Tensor,
    vel_target_b: torch.Tensor,
    ramp_start_speed: torch.Tensor,
    ramp_target_speed: torch.Tensor,
    ramp_elapsed: torch.Tensor,
    ramp_duration: torch.Tensor,
    ramp_active: torch.Tensor,
    env_ids,
):
    """Unconditionally clear the per-episode command/ramp state for ``env_ids``.

    Used by ``VelocityTrackingCommand.reset`` so a robot that fell early never
    inherits the previous episode's command or acceleration ramp.  This is called
    for *every* reset (independent of ``time_left``), and must happen before the
    command is resampled so acceleration ramps start from rest.

    Args:
        current_vel_b: Current (max_acc-limited) command tensor ``[N, 3]``.
        vel_command_b: Command output tensor ``[N, 3]``.
        vel_target_b: Command target tensor ``[N, 3]``.
        ramp_start_speed: Acceleration ramp start speed tensor ``[N]``.
        ramp_target_speed: Acceleration ramp target speed tensor ``[N]``.
        ramp_elapsed: Acceleration ramp elapsed time tensor ``[N]``.
        ramp_duration: Acceleration ramp duration tensor ``[N]``.
        ramp_active: Acceleration ramp active mask tensor ``[N]`` (bool).
        env_ids: Environment ids to clear (tensor or slice).
    """
    current_vel_b[env_ids] = 0.0
    vel_command_b[env_ids] = 0.0
    vel_target_b[env_ids] = 0.0
    ramp_start_speed[env_ids] = 0.0
    ramp_target_speed[env_ids] = 0.0
    ramp_elapsed[env_ids] = 0.0
    ramp_duration[env_ids] = 1.0
    ramp_active[env_ids] = False


def limit_command(
    desired: torch.Tensor,
    current: torch.Tensor,
    max_acc: torch.Tensor,
    dt: float,
) -> torch.Tensor:
    """Clamp a desired command to a per-axis slew-rate (max_acc) limit.

    This is the exact command-acceleration limiter used by
    ``VelocityTrackingCommand._update_command``.  Because the clamp is applied
    to the *difference* from the current command, the output is continuous in
    ``desired`` and never produces a step, even when ``max_acc * dt`` is large.

    Args:
        desired: Desired command of shape ``[..., 3]``.
        current: Current (last applied) command of shape ``[..., 3]``.
        max_acc: Per-axis limit of shape ``[3]``.
        dt: Control timestep in seconds.

    Returns:
        The rate-limited command of shape ``[..., 3]``.
    """
    max_delta = max_acc.unsqueeze(0) * dt
    return torch.clamp(
        desired,
        min=current - max_delta,
        max=current + max_delta,
    )


def advance_acceleration_ramp(
    start: torch.Tensor,
    target: torch.Tensor,
    elapsed: torch.Tensor,
    duration: torch.Tensor,
    active: torch.Tensor,
    dt: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Advance a linear acceleration ramp by one control step.

    The ramp produces ``vx = start + (target - start) * clamp(elapsed/duration, 0, 1)``
    so it is a *continuous* velocity trajectory, not a fixed random target.
    ``active`` becomes False once the ramp completes.

    Args:
        start: Ramp start speed of shape ``[...]``.
        target: Ramp target speed of shape ``[...]``.
        elapsed: Elapsed time since ramp start, shape ``[...]``.
        duration: Ramp duration (seconds), shape ``[...]`` (must be > 0).
        active: Whether the ramp is still running, shape ``[...]`` (bool).
        dt: Control timestep in seconds.

    Returns:
        ``(vx, new_elapsed, new_active)`` each of shape ``[...]``.
    """
    new_elapsed = elapsed + dt
    dur = torch.clamp(duration, min=1e-6)
    frac = torch.clamp(new_elapsed / dur, 0.0, 1.0)
    vx = start + (target - start) * frac
    done = new_elapsed >= duration
    new_active = active & (~done)
    return vx, new_elapsed, new_active


# The mutually-exclusive sprint command categories.  ``transition`` is optional
# and defaults to probability 0 in legacy configs.
SPRINT_CATEGORIES = ("zero_stand", "low_speed", "transition", "acceleration", "sprint", "correction")


def assign_dr_group_ids(
    num_envs: int,
    nominal_rel_envs: float,
    seed: int | None = None,
    device: str | torch.device | None = None,
) -> torch.Tensor:
    """Assign a persistent DR group: nominal (True) vs disturbed (False).

    The mask is drawn once and shared by every subset DR event, so a nominal
    environment is never touched by friction / mass / COM / gain / joint
    friction / armature / push randomization.  ``nominal_rel_envs`` is the
    fraction of environments assigned to the nominal group.

    Sampling is done deterministically on CPU and the result is moved to
    ``device``, which is always valid (no CUDA-generator issues).

    Args:
        num_envs: Number of environments.
        nominal_rel_envs: Fraction of environments in the nominal group.
        seed: Optional RNG seed for deterministic assignment.
        device: Target device for the returned mask (e.g. ``"cuda:0"``).

    Returns:
        A bool tensor of shape ``[num_envs]`` (True = nominal).
    """
    g = torch.Generator()
    if seed is not None:
        g.manual_seed(seed)
    rel = float(min(max(nominal_rel_envs, 0.0), 1.0))
    mask = torch.rand(num_envs, generator=g) < rel
    if device is not None:
        mask = mask.to(device)
    return mask


def non_nominal_ids(env_ids: torch.Tensor, nominal_mask: torch.Tensor) -> torch.Tensor:
    """Return the environment ids that are NOT in the nominal group.

    ``nominal_mask`` is moved to ``env_ids.device`` so advanced indexing works
    regardless of where the mask was created.
    """
    return env_ids[~nominal_mask.to(env_ids.device)[env_ids]]


def sample_env_subset_from_mask(
    env_ids: torch.Tensor,
    nominal_mask: torch.Tensor,
    rel_envs: float,
    seed: int | None = None,
) -> torch.Tensor:
    """Sample ``rel_envs`` fraction of the NON-nominal environments.

    This is the shared-mask logic behind ``subset_randomization.sample_env_subset``.
    It guarantees that a nominal environment is never returned, no matter how
    many times it is called (the mask is fixed, not re-sampled per event).

    All tensors (env_ids, nominal_mask, and the sampled subset) share
    ``env_ids.device``.

    Args:
        env_ids: Candidate environment ids.
        nominal_mask: Persistent bool mask (True = nominal).
        rel_envs: Fraction of non-nominal candidates to keep.
        seed: Optional RNG seed.

    Returns:
        A tensor of sampled non-nominal environment ids (same device as
        ``env_ids``).
    """
    device = env_ids.device
    candidates = non_nominal_ids(env_ids, nominal_mask.to(device))
    if len(candidates) == 0:
        return candidates
    rel = float(min(max(rel_envs, 0.0), 1.0))
    g = torch.Generator()
    if seed is not None:
        g.manual_seed(seed)
    mask = torch.rand(len(candidates), generator=g).to(device) < rel
    return candidates[mask]


def stage_dr_spec(stage: str) -> dict:
    """Domain-randomization spec for a sprint curriculum stage (pure).

    This is the single source of truth for what DR each stage enables.  The
    sprint config applies the returned dict to its event terms, and the unit
    tests assert the spec directly (so an inherited-but-unused event cannot go
    unnoticed).

    Keys:

    * ``friction_range``: ``(static, dynamic)`` ground-friction range; ``None``
      disables ground-friction randomization.
    * ``joint_friction``: bool (whether joint friction/armature DR is enabled).
    * ``base_mass_range`` / ``com_range``: ``None`` disables; else the range.
    * ``gain_range``: ``None`` disables actuator gain DR.
    * ``push_rel_envs``: fraction of envs that receive pushes (0 = off).
    * ``push_velocity_range``: dict for the subset push.
    * ``nominal_rel_envs``: fraction of envs left at nominal physics.

    Args:
        stage: ``"A"``, ``"B"``, or ``"C"``.

    Returns:
        A dict with the DR configuration for the stage.

    Raises:
        ValueError: If the stage is unknown.
    """
    spec = {
        "A": {
            "friction_range": None,          # nominal friction
            "joint_friction": False,
            "base_mass_range": None,
            "com_range": None,
            "gain_range": None,
            "push_rel_envs": 0.0,
            "push_velocity_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)},
            "nominal_rel_envs": 1.0,
        },
        "B": {
            "friction_range": (0.5, 1.6),
            "joint_friction": False,
            "base_mass_range": (0.9, 1.1),
            "com_range": {"x": (-0.02, 0.02), "y": (-0.02, 0.02), "z": (-0.01, 0.01)},
            "gain_range": (0.95, 1.05),
            "push_rel_envs": 0.2,
            "push_velocity_range": {"x": (-0.3, 0.3), "y": (-0.4, 0.4), "yaw": (-0.2, 0.2)},
            "nominal_rel_envs": 0.5,
        },
        "C": {
            "friction_range": (0.3, 2.0),
            "joint_friction": True,
            "base_mass_range": (0.75, 1.25),
            "com_range": {"x": (-0.06, 0.06), "y": (-0.06, 0.06), "z": (-0.02, 0.02)},
            "gain_range": (0.85, 1.15),
            "push_rel_envs": 0.4,
            "push_velocity_range": {"x": (-0.6, 0.6), "y": (-0.8, 0.8), "yaw": (-0.4, 0.4)},
            "nominal_rel_envs": 0.25,
        },
    }
    if stage not in spec:
        raise ValueError(f"Unknown sprint stage: {stage!r}. Expected one of {sorted(spec)}.")
    return spec[stage]


def sprint_profile_cutoffs(
    probs: tuple[float, ...],
    device: str | torch.device | None = None,
) -> torch.Tensor:
    """Normalize sprint-category probabilities and return cumulative cutoffs.

    ``probs`` must have entries in ``SPRINT_CATEGORIES`` order
    ``(zero_stand, low_speed, transition, acceleration, sprint, correction)``.

    Args:
        probs: Raw category probabilities (need not sum to 1).
        device: Device for the returned tensor (must match the ``mode`` tensor
            used in :func:`sprint_category_masks`).

    Returns:
        Cumulative cutoffs tensor of shape ``[5]`` in ``[0, 1]``.

    Raises:
        ValueError: If the probabilities sum to a non-positive value.
    """
    if len(probs) != len(SPRINT_CATEGORIES):
        raise ValueError(f"Expected {len(SPRINT_CATEGORIES)} probabilities, got {len(probs)}.")
    if any(float(p) < 0.0 for p in probs):
        raise ValueError("Sprint profile probabilities must be non-negative.")
    total = float(sum(probs))
    if total <= 0.0:
        raise ValueError("Sprint profile probabilities must sum to a positive value.")
    cum = torch.tensor([sum(probs[: i + 1]) / total for i in range(len(probs))])
    if device is not None:
        cum = cum.to(device)
    return cum


def validate_sprint_profile_ranges(
    low_speed_vx_range: tuple[float, float],
    transition_vx_range: tuple[float, float],
    transition_lat_range: tuple[float, float],
    transition_yaw_range: tuple[float, float],
    low_speed_lat_range: tuple[float, float],
    low_speed_yaw_range: tuple[float, float],
    sprint_vx_range: tuple[float, float],
    correction_vx_range: tuple[float, float],
    accel_targets: tuple[float, ...],
    accel_ramp_duration_range: tuple[float, float],
) -> None:
    """Validate sprint-profile ranges so bad low-speed configs fail early.

    The low-speed category is a walking category, not a standing alias.  Its
    forward range must therefore stay strictly positive; otherwise an accidental
    ``(0, x)`` or reversed range can silently reintroduce standing-mode pollution.
    """

    def check_range(name: str, value: tuple[float, float], *, positive_low: bool = False) -> None:
        if len(value) != 2:
            raise ValueError(f"{name} must contain exactly two values.")
        low, high = float(value[0]), float(value[1])
        if not low < high:
            raise ValueError(f"{name} requires low < high, got {value!r}.")
        if positive_low and low <= 0.0:
            raise ValueError(f"{name} must start above 0 for non-standing walking, got {value!r}.")

    check_range("low_speed_vx_range", low_speed_vx_range, positive_low=True)
    check_range("transition_vx_range", transition_vx_range, positive_low=True)
    check_range("transition_lat_range", transition_lat_range)
    check_range("transition_yaw_range", transition_yaw_range)
    check_range("low_speed_lat_range", low_speed_lat_range)
    check_range("low_speed_yaw_range", low_speed_yaw_range)
    check_range("sprint_vx_range", sprint_vx_range, positive_low=True)
    check_range("correction_vx_range", correction_vx_range, positive_low=True)
    check_range("accel_ramp_duration_range", accel_ramp_duration_range, positive_low=True)
    if len(accel_targets) == 0:
        raise ValueError("accel_targets must not be empty.")
    if any(float(v) <= 0.0 for v in accel_targets):
        raise ValueError("accel_targets must be strictly positive.")


def sprint_mode_assignment(
    zero_stand: torch.Tensor,
    low_speed: torch.Tensor,
    transition: torch.Tensor,
    acceleration: torch.Tensor,
    sprint: torch.Tensor,
    correction: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Map category masks to controller-mode masks.

    Only ``zero_stand`` is marked standing (and thus zeroed).  ``low_speed`` and
    ``transition`` are open-loop (NOT zeroed).  ``acceleration`` and ``sprint``
    are open-loop; ``correction`` is closed-loop.

    Returns ``(is_sprint_straight, is_open_loop, is_closed_loop, is_standing)``.
    """
    is_sprint_straight = sprint
    is_open_loop = low_speed | transition | acceleration | sprint
    is_closed_loop = correction
    is_standing = zero_stand
    return is_sprint_straight, is_open_loop, is_closed_loop, is_standing


def sprint_category_masks(
    mode: torch.Tensor,
    cum: torch.Tensor,
) -> tuple[torch.Tensor, ...]:
    """Partition a ``[0,1)`` uniform sample into the sprint categories.

    ``mode`` is drawn once at resample time, so the resulting category masks
    (including the sprint-straight marker) are a function of ``mode`` only and
    cannot be flipped by a later closed-loop correction.

    Args:
        mode: Uniform ``[0, 1)`` tensor of shape ``[...]``.
        cum: Cumulative cutoffs tensor of shape ``[5]`` (from
            :func:`sprint_profile_cutoffs`).

    Returns:
        Boolean tensors in ``SPRINT_CATEGORIES`` order, each of shape ``[...]``.
    """
    # Keep the cutoffs on the same device as the mode tensor (torch.searchsorted
    # requires both inputs on one device).
    idx = torch.searchsorted(cum.to(mode.device), mode, right=False).clamp(0, len(cum) - 1)
    masks = tuple((idx == i) for i in range(len(SPRINT_CATEGORIES)))
    return masks


# ---------------------------------------------------------------------------
# Straightness-recovery reward helpers (pure torch, testable without isaaclab)
# ---------------------------------------------------------------------------


def pseudo_huber_penalty(error: torch.Tensor, delta: float = 0.4) -> torch.Tensor:
    """Pseudo-Huber penalty ``delta * (sqrt(1 + (error/delta)^2) - 1)``.

    Unlike a narrow Gaussian (``exp(-(error/std)^2)``), whose gradient vanishes
    at large errors, this term is quadratic near zero and asymptotically linear
    (absolute slope -> 1) for large errors, so the corrective gradient never
    disappears.  It is the *separate* straightness penalty used by the
    straightness-recovery task.

    Properties (unit-tested):
    - ``penalty(0) == 0``, non-negative, symmetric in ``error``.
    - monotone increasing in ``|error|``.
    - approximately quadratic for ``|error| << delta``.
    - ``d/d|error| -> 1`` for ``|error| >> delta`` (never zero gradient).

    Args:
        error: Signed error tensor of any shape (e.g. ``[num_envs]`` or batched).
        delta: Huber width (``lateral_delta=0.40`` m, ``heading_delta=0.20`` rad).

    Returns:
        Penalty tensor of the same shape, dtype and device as ``error``.
    """
    d = torch.as_tensor(delta, dtype=error.dtype, device=error.device)
    r = error / d
    return d * (torch.sqrt(1.0 + r * r) - 1.0)


def straightness_speed_gate(
    speed: torch.Tensor,
    off: float = 0.15,
    on: float = 0.50,
) -> torch.Tensor:
    """Smooth straightness gate: 0 for ``speed <= off``, 1 for ``speed >= on``.

    Used to apply the straight-line penalties to every non-zero command category
    (low-speed forward, acceleration, sprint, correction) while fully closing
    the penalty on genuine zero-stand environments (0 m/s).  The ramp is
    ``smoothstep(speed, off, on)`` so there is no reward step at the band edges.

    Args:
        speed: Commanded forward speed tensor of shape ``[...]``.
        off: Speed (m/s) at/below which the gate is fully closed (0).
        on: Speed (m/s) at/above which the gate is fully open (1).

    Returns:
        Gate tensor of shape ``[...]`` in ``[0, 1]``.
    """
    return smoothstep(speed, off, on)


def straightness_factor(
    lateral: torch.Tensor,
    heading: torch.Tensor,
    lateral_scale: float = 0.75,
    heading_scale: float = 0.25,
    factor_floor: float = 0.15,
) -> torch.Tensor:
    """Bounded, smooth straightness factor in ``[floor, 1]`` coupling forward
    progress to track deviation.

    ``factor = floor + (1 - floor) * lat_f * hdg_f`` with
    ``lat_f = 1/(1 + (lateral/lateral_scale)^2)`` and
    ``hdg_f = 1/(1 + (heading/heading_scale)^2)``.  Straight running gives
    ``factor ~ 1``; large deviation pushes it toward ``floor`` (never negative,
    never below ``floor``) so the progress signal is attenuated but the separate
    pseudo-Huber penalty remains the primary corrective pull.

    Args:
        lateral: Signed lateral track error (m), shape ``[...]``.
        heading: Signed track heading error (rad), shape ``[...]``.
        lateral_scale: Lateral error at which ``lat_f = 0.5`` (m).
        heading_scale: Heading error at which ``hdg_f = 0.5`` (rad).
        factor_floor: Minimum factor (0.10-0.20 recommended; default 0.15).

    Returns:
        Factor tensor of shape ``[...]`` in ``[factor_floor, 1]``.
    """
    lat_f = 1.0 / (1.0 + (lateral.abs() / lateral_scale) ** 2)
    hdg_f = 1.0 / (1.0 + (heading.abs() / heading_scale) ** 2)
    floor = float(min(max(factor_floor, 0.0), 1.0))
    return floor + (1.0 - floor) * lat_f * hdg_f


def coupled_progress_value(base_progress: torch.Tensor, factor: torch.Tensor) -> torch.Tensor:
    """Straightness-coupled forward progress ``base_progress * factor``.

    ``factor`` is non-negative (>= ``factor_floor``), so a negative
    ``base_progress`` (moving backward) stays a penalty and can never be turned
    into a reward by the multiplication.

    Args:
        base_progress: Un-coupled progress value (e.g. gated ``v_track / L``).
        factor: Straightness factor of the same shape.

    Returns:
        Coupled progress tensor of the same shape.
    """
    return base_progress * factor


# Deterministic per-stage initial-state error ranges for the recovery phase.
RECOVERY_ERROR_STAGES = {
    # stage -> {"y": m, "yaw": rad, "lat_vel": m/s, "yaw_rate": rad/s}
    # The event applies uniform samples from ``(-v, +v)`` per axis.
    0: {"y": 0.05, "yaw": 0.02, "lat_vel": 0.05, "yaw_rate": 0.03},
    1: {"y": 0.15, "yaw": 0.05, "lat_vel": 0.10, "yaw_rate": 0.06},
}


def recovery_initial_error_ranges(stage: int) -> dict:
    """Deterministic per-stage initial-error ranges (pure, no hidden state).

    Returns a dict mapping axis name -> ``(lo, hi)`` signed range:

    * ``y``: lateral offset from the frozen track centerline (m).
    * ``yaw``: heading offset from the frozen track direction (rad).
    * ``lat_vel``: lateral velocity relative to the track (m/s).
    * ``yaw_rate``: yaw rate offset (rad/s).

    Args:
        stage: Curriculum stage index (``0`` small, ``1`` larger).

    Returns:
        Dict of ``{axis: (lo, hi)}`` symmetric ranges.

    Raises:
        ValueError: If the stage is unknown.
    """
    spec = RECOVERY_ERROR_STAGES.get(int(stage))
    if spec is None:
        raise ValueError(
            f"Unknown recovery initial-error stage: {stage!r}. Expected one of {sorted(RECOVERY_ERROR_STAGES)}."
        )
    return {k: (-v, v) for k, v in spec.items()}


def recovery_error_group_mask(
    num_envs: int,
    rel_nominal: float = 0.40,
    seed: int = 0,
    device: str | torch.device | None = None,
) -> torch.Tensor:
    """Persistent, deterministic nominal mask for recovery initial-state errors.

    ``rel_nominal`` fraction of environments are nominal (no initial error);
    the remaining environments receive the staged initial errors.  Deterministic
    for a given ``(num_envs, rel_nominal, seed)`` so the split is testable and
    reproducible.

    Args:
        num_envs: Number of environments.
        rel_nominal: Fraction of environments left error-free (>= 0.25-0.40).
        seed: Deterministic seed.
        device: Target device for the returned bool mask.

    Returns:
        Bool tensor of shape ``[num_envs]`` (True = nominal / no error).
    """
    return assign_dr_group_ids(num_envs, rel_nominal, seed, device)


# ---------------------------------------------------------------------------
# Straightness-recovery training hyperparameters (single source of truth).
# The runner config reads these so the pure tests can assert them directly.
# ---------------------------------------------------------------------------
RECOVERY_MAX_ITERATIONS = 600
RECOVERY_SAVE_INTERVAL = 50
RECOVERY_LEARNING_RATE = 3.0e-5
RECOVERY_ENTROPY_COEF = 0.002
RECOVERY_DESIRED_KL = 0.0075
