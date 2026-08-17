// Copyright 2026 Zachary Olkin. All rights reserved.
//
// Canonical gait-phase hold state machine.
//
// Must stay identical to the Python implementation in
// robot_rl/.../mdp/phase_hold.py and to the IsaacLab training side
// (trajectory_cmd.update_phasing_var).  Kept in a standalone header so a small
// C++ golden-vector test can compile and exercise it independently of the full
// rl_sar runtime.
#pragma once

#include <cmath>

namespace rl_sar_phase
{

inline float RawPhase(float time, float phase_period)
{
    return std::fmod(time, phase_period) / phase_period;
}

inline float PhaseHoldStep(
    float prev_phi,
    float raw_phi,
    bool prev_should_hold,
    bool should_hold,
    int &boundaries_crossed,
    float &hold_phi_value,
    bool episode_reset,
    int phasing_boundaries)
{
    // On an episode reset, prev_phi belongs to the previous episode; using it
    // would create a spurious boundary crossing into the new episode.
    if (episode_reset)
    {
        prev_phi = raw_phi;
    }

    bool newly_holding = should_hold && !prev_should_hold;
    if (newly_holding || episode_reset)
    {
        boundaries_crossed = 0;
        hold_phi_value = -1.0f;
    }

    if (!should_hold)
    {
        hold_phi_value = -1.0f;
        boundaries_crossed = 0;
    }

    bool crosses_zero = false;
    bool crosses_half = false;
    if (should_hold && hold_phi_value < 0.0f)
    {
        crosses_zero = (raw_phi < prev_phi) && (prev_phi > 0.0f);
        crosses_half = (prev_phi < 0.5f) && (raw_phi >= 0.5f);
        if (crosses_zero || crosses_half)
        {
            boundaries_crossed += 1;
        }
        if (crosses_zero && boundaries_crossed >= phasing_boundaries)
        {
            hold_phi_value = 0.0f;
        }
        else if (crosses_half && boundaries_crossed >= phasing_boundaries)
        {
            hold_phi_value = 0.5f;
        }
    }

    if (hold_phi_value >= 0.0f)
    {
        return hold_phi_value;
    }
    return raw_phi;
}

}  // namespace rl_sar_phase
