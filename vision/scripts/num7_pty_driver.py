#!/usr/bin/env python3
"""PTY driver for the Num7 headless smoke.

Spawns rl_real_g1 on a pseudo-terminal (so kbhit() sees a real TTY), feeds
the 0 -> 1 -> 7 key sequence with correct timing, and captures all output to
a log. Never touches a real robot.
"""
from __future__ import annotations

import os
import pty
import select
import signal
import subprocess
import sys
import time


def main() -> int:
    if len(sys.argv) < 7:
        print(
            "usage: num7_pty_driver.py <binary> <log> <target_m> "
            "<run_seconds> <command_port> <status_port>"
        )
        return 2
    binary = sys.argv[1]
    log_path = sys.argv[2]
    target_m = sys.argv[3]
    run_seconds = float(sys.argv[4])
    command_port = int(sys.argv[5])
    status_port = int(sys.argv[6])

    env = dict(os.environ)
    env.update(
        {
            "G1_NUM7_TARGET_M": target_m,
            "G1_NUM7_MAX_DURATION_S": "7.0",
            "G1_NUM7_STOP_MARGIN_M": "0.0",
            "G1_NUM7_DISTANCE_SCALE": os.environ.get(
                "G1_NUM7_DISTANCE_SCALE", "0.60"
            ),
            "G1_NUM7_START_TIMEOUT_S": "2.0",
            "G1_NUM7_SETTLE_S": "0.5",
            "G1_VISION_MAX_VX": os.environ.get("G1_VISION_MAX_VX", "0.50"),
            "G1_VISION_MAX_WZ": os.environ.get("G1_VISION_MAX_WZ", "0.25"),
            "G1_VISION_UDP_PORT": str(command_port),
            "G1_VISION_STATUS_PORT": str(status_port),
        }
    )

    master, slave = pty.openpty()
    proc = subprocess.Popen(
        [binary, "lo"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        env=env,
        close_fds=True,
        start_new_session=True,
    )
    os.close(slave)

    deadline = time.monotonic() + 6.0 + run_seconds + 8.0
    # Keys are sent repeatedly at staggered times because the FSM only
    # consumes a key edge when the current state is ready for it (e.g. Num1
    # is only valid once GetUp has finished). Re-sending covers lost edges.
    key_plan = [
        (2.0, b"0"),   # Passive -> GetUp
        (8.0, b"1"),   # GetUp -> state 1
        (13.0, b"1"),
        (18.0, b"7"),  # state 1 -> Num7 walk
        # Leave enough time after mission 1 for the 0.5 s C++ settle plus the
        # independent one-second post-stop zero/slide observation window.
        (27.0, b"7"),
    ]
    last_key_at = 0.0
    key_idx = 0

    with open(log_path, "wb") as log:
        start = time.monotonic()
        while time.monotonic() < deadline:
            if key_idx < len(key_plan):
                when, key = key_plan[key_idx]
                if time.monotonic() - start >= when:
                    os.write(master, key)
                    log.write(b"\n[KEY] " + key + b"\n")
                    log.flush()
                    key_idx += 1
                    last_key_at = time.monotonic()
            r, _, _ = select.select([master], [], [], 0.05)
            if r:
                try:
                    data = os.read(master, 4096)
                except OSError:
                    break
                if not data:
                    break
                log.write(data)
                log.flush()
            if proc.poll() is not None and time.monotonic() - last_key_at > 3.0:
                break
            time.sleep(0.02)

        # Stop the loopback controller before draining.  The controller emits
        # a continuous status line, so trying to drain while it is still alive
        # can never reach EOF and leaves stale rl_real_g1 processes/UDP binds.
        terminated_by_driver = proc.poll() is None
        if terminated_by_driver:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

        # Drain only the finite output already buffered after process exit.
        drain_deadline = time.monotonic() + 1.0
        while time.monotonic() < drain_deadline:
            r, _, _ = select.select([master], [], [], 0.1)
            if not r:
                break
            try:
                data = os.read(master, 4096)
            except OSError:
                break
            if not data:
                break
            log.write(data)
            log.flush()

    os.close(master)
    # Reaching the requested test deadline is an expected cooperative stop.
    # An earlier controller exit is not expected and must fail the smoke.
    if terminated_by_driver and proc.returncode in (-signal.SIGTERM, 0):
        return 0
    return proc.returncode if proc.returncode is not None else 1


if __name__ == "__main__":
    sys.exit(main())
