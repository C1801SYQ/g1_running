from __future__ import annotations


def mujoco_state_was_reset(
    previous_time: float | None,
    current_time: float,
    previous_x: float | None,
    current_x: float,
    *,
    minimum_time_rewind_s: float = 0.5,
    minimum_backward_jump_m: float = 1.0,
) -> bool:
    """Detect a MuJoCo viewer reset from discontinuous simulation state.

    The passive viewer resets ``MjData`` directly and does not notify the
    Python vision mission or the C++ FSM. Simulation time rewinding is the
    authoritative signal; the X jump is a fallback for reset implementations
    that preserve time.
    """

    time_rewound = (
        previous_time is not None
        and current_time < previous_time - minimum_time_rewind_s
    )
    position_rewound = (
        previous_x is not None
        and current_x < previous_x - minimum_backward_jump_m
    )
    return time_rewound or position_rewound
