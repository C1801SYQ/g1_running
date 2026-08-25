"""Headless smoke test for the lateral-recovery sprint env (8 envs).

Verifies: obs [8,98], action 29, device cuda:0, command on cuda, the lateral
push state machine (pushes fire only on non-nominal envs, +/- counters balanced,
reset clears dirty push state), rewards finite, and several NaN-free steps.

Run via the kit python with the standard PYTHONPATH (see methodology), e.g.:
  .../_isaac_sim/python.sh smoke_lateral_recovery_env.py --task G1-running-sprint-lateral-recovery-29dof
"""

import argparse
import json
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Smoke test the lateral-recovery sprint env.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--steps", type=float, default=8.0, help="sim seconds to run")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
args_cli.headless = True
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import robot_rl.tasks  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robot_rl.tasks.manager_based.robot_rl.mdp.lateral_push import (  # noqa: E402
    lateral_push_counter_summary,
)


def main():
    task = args_cli.task
    result = {"task": task, "created": False}
    env = None
    try:
        env_cfg = parse_env_cfg(task, num_envs=8, device="cuda:0")
        env = gym.make(task, cfg=env_cfg)
        unw = env.unwrapped
        obs, _ = env.reset()
        action_dim = int(unw.single_action_space.shape[0])
        obs_shape = list(obs["policy"].shape) if isinstance(obs, dict) else list(obs.shape)
        device = str(unw.device)
        cmd = unw.command_manager.get_term("base_velocity")

        action = torch.zeros((8, action_dim), device=unw.device)
        n_steps = int(args_cli.steps / unw.step_dt)
        reward_finite = True
        nan_steps = 0
        first_push_step = None
        diag = {
            "interval_events": [
                (tc.func.__name__, str(tc.interval_range_s), str(tc.params))
                for tc in unw.event_manager._mode_term_cfgs["interval"]
            ],
            "reset_events": [tc.func.__name__ for tc in unw.event_manager._mode_term_cfgs["reset"]],
            "startup_events": [tc.func.__name__ for tc in unw.event_manager._mode_term_cfgs.get("startup", [])],
            "dr_nominal_count": int(unw._dr_nominal.sum().item()),
            "cmd_vx_after_reset": [round(float(v), 2) for v in cmd.command[:, 0]],
        }
        for _ in range(n_steps):
            obs, reward, _, _, _ = env.step(action)
            if not torch.isfinite(reward).all().item():
                reward_finite = False
            if not torch.isfinite(obs["policy"]).all().item():
                nan_steps += 1
            st = getattr(unw, "_lateral_push_state", None)
            if st is not None and int(st["pushed"].sum().item()) > 0 and first_push_step is None:
                first_push_step = _
        if first_push_step is not None:
            diag["cmd_vx_at_push"] = [round(float(v), 2) for v in cmd.command[:, 0]]
        with open("/tmp/g1_running/smoke_lat_diag.json", "w") as _f:
            json.dump(diag, _f, indent=2)

        state = getattr(unw, "_lateral_push_state", None)
        counters = lateral_push_counter_summary(state) if state else {}
        nominal = getattr(unw, "_dr_nominal", None)
        nominal_leak = None
        if state is not None and nominal is not None:
            pushed_ids = state["pushed"].nonzero(as_tuple=False).flatten()
            leak = pushed_ids[nominal.to(state["pushed"].device)[pushed_ids]]
            nominal_leak = int(leak.numel())

        # --- Deterministic push-mechanism verification ----------------------
        # The zero-action robot falls before the 2-4 s interval fires, so to
        # validate the mechanism we force the command gate open on a few envs
        # and invoke the push event once, then verify velocity, state, counters
        # and that nominal envs are never pushed.
        push_mech = {"invoked": False, "pushed": 0, "plus": 0, "minus": 0,
                     "nominal_leak": 0, "vel_delta_norm": 0.0}
        from robot_rl.tasks.manager_based.robot_rl.mdp.lateral_push import lateral_push_curriculum  # noqa: PLC0415
        from robot_rl.tasks.manager_based.robot_rl.mdp.events.lateral_push_events import (  # noqa: PLC0415
            apply_lateral_recovery_push,
        )
        if state is not None and nominal is not None:
            before_vel = unw.scene["robot"].data.root_lin_vel_w.clone()
            # force the command gate open on all envs for the manual invocation
            cmd.command[:, 0] = 5.0
            cmd.command[:, 1] = 0.0
            cmd.command[:, 2] = 0.0
            all_ids = torch.arange(8, device=unw.device)
            # actual_vx_min=0.0 bypasses the ACTUAL-velocity gate (unit-tested
            # separately) so the mechanism (impulse / counters / symmetry /
            # no-leak) can be exercised on the stationary zero-action robot.
            apply_lateral_recovery_push(
                unw, all_ids, command_name="base_velocity", chunk=1,
                rel_disturbed=0.30, seed=0, min_elapsed_s=0.0,
                vx_min=3.0, actual_vx_min=0.0, vy_max=0.12, yaw_max=0.12,
            )
            after_vel = unw.scene["robot"].data.root_lin_vel_w.clone()
            push_mech["invoked"] = True
            st = unw._lateral_push_state
            push_mech["pushed"] = int(st["pushed"].sum().item())
            push_mech["plus"] = st["plus_count"]
            push_mech["minus"] = st["minus_count"]
            push_mech["direction_mask"] = [int(x) for x in st["direction"].tolist()]
            push_mech["nominal_mask"] = [int(x) for x in nominal.tolist()]
            pushed_ids = st["pushed"].nonzero(as_tuple=False).flatten()
            if pushed_ids.numel() > 0:
                leak = pushed_ids[nominal.to(st["pushed"].device)[pushed_ids]]
                push_mech["nominal_leak"] = int(leak.numel())
                push_mech["vel_delta_norm"] = float(
                    (after_vel[pushed_ids, :2] - before_vel[pushed_ids, :2]).norm(dim=-1).max().item()
                )
            result["push_mechanism"] = push_mech

            # --- recovery-reward magnitude (round-2 observability check) -----
            # Right after a push the pushed envs are inside the recovery window;
            # the recovery reward must be a non-trivial negative value there
            # (per-pushed-env magnitude O(1)), not ~1e-4.  We step the push time
            # 0.6 s into the past so the 0.5 s ramp is fully open.
            from robot_rl.tasks.manager_based.robot_rl.mdp.lateral_push_rewards import (  # noqa: PLC0415
                lateral_push_recovery_reward,
            )
            elapsed_now = unw.episode_length_buf * unw.step_dt
            pushed_now = st["pushed"]
            # Guarantee an in-window state even if the zero-action robot's
            # episode just reset (elapsed ~ 0): push_time is clamped >= 0 so the
            # recovery-window mask stays active.
            st["push_time"][pushed_now] = torch.clamp(elapsed_now[pushed_now] - 0.6, min=0.0)
            rrec = lateral_push_recovery_reward(
                unw, command_name="base_velocity", window=2.5, ramp_s=0.5,
                delta_vlat=0.25, delta_wz=0.25, delta_heading=0.25,
                delta_lat=0.5, delta_vx=0.5, scale=2.0,
            )
            rec_on_pushed = rrec[pushed_now]
            result["recovery_reward_mean_on_pushed"] = float(rec_on_pushed.mean().item()) if pushed_now.any() else 0.0
            result["recovery_reward_observable"] = (
                bool(pushed_now.any()) and torch.isfinite(rec_on_pushed).all().item()
                and abs(float(rec_on_pushed.mean().item())) > 0.05
            )
            result["recovery_reward_diag"] = {
                "pushed_at_check": int(pushed_now.sum().item()),
                "reward_all": [round(float(v), 3) for v in rrec.tolist()],
                "elapsed_s": round(float(elapsed_now.max().item()), 2),
            }

        # dirty-state reset check: push state must clear on a fresh reset.
        obs, _ = env.reset()
        pushed_after_reset = int(state["pushed"].sum().item()) if state is not None else None

        result.update({
            "created": True,
            "obs_shape": obs_shape,
            "action_dim": action_dim,
            "device": device,
            "command_device": str(cmd.device),
            "cmd_on_device": str(cmd.device) == device,
            "sim_steps": n_steps,
            "lateral_push_state_present": state is not None,
            "push_counters": counters,
            "pushes_fired": counters.get("total", 0) if counters else 0,
            "first_push_step": first_push_step,
            "balance_ok": counters.get("total", 0) > 0 and abs(counters.get("balance", 0) - 0.5) <= 0.3,
            "nominal_leak_count": nominal_leak,
            "nominal_leak_ok": nominal_leak == 0,
            "rewards_finite": reward_finite,
            "nan_steps": nan_steps,
            "pushed_after_reset": pushed_after_reset,
            "reset_clears_push": pushed_after_reset == 0,
        })
        print(f"[CREATED_OK] {task}", flush=True)
    except Exception as exc:  # noqa: BLE001
        import traceback

        result["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        print(f"[FAIL] {task}: {type(exc).__name__}: {exc}", flush=True)
    finally:
        if env is not None:
            try:
                env.close()
            except Exception:  # noqa: BLE001
                pass

    print("SMOKE_RESULT=" + json.dumps(result, indent=2, sort_keys=True), flush=True)
    simulation_app.close()
    if not result.get("created"):
        sys.exit(1)


if __name__ == "__main__":
    main()
