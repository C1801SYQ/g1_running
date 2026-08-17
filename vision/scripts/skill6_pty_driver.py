#!/usr/bin/env python3
"""PTY driver for the Skill 6 (100 m sprint) headless smoke.

Spawns rl_real_g1 on a pseudo-terminal (so kbhit() sees a real TTY), feeds the
0 -> 1 -> 6 key sequence, and captures all output to a log.  Never touches a
real robot.
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
    if len(sys.argv) < 6:
        print(
            "usage: skill6_pty_driver.py <binary> <log> <run_seconds> "
            "<command_port> <status_port>"
        )
        return 2
    binary = sys.argv[1]
    log_path = sys.argv[2]
    run_seconds = float(sys.argv[3])
    command_port = int(sys.argv[4])
    status_port = int(sys.argv[5])

    env = dict(os.environ)
    env.update(
        {
            "G1_VISION_MAX_VX": os.environ.get("G1_VISION_MAX_VX", "5.10"),
            "G1_VISION_MAX_WZ": os.environ.get("G1_VISION_MAX_WZ", "0.35"),
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
    # 0 -> 1 -> 6 (Passive -> GetUp -> state 1 -> Skill 6 sprint).  Keys are
    # re-sent at staggered times because the FSM only consumes an edge when the
    # current state is ready for it.
    key_plan = [
        (2.0, b"0"),   # Passive -> GetUp
        (8.0, b"1"),   # GetUp -> state 1
        (13.0, b"1"),
        (18.0, b"6"),  # state 1 -> Skill 6 sprint
        (27.0, b"6"),
    ]
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
            if proc.poll() is not None and time.monotonic() - start > 8.0:
                break
            time.sleep(0.02)

        terminated_by_driver = proc.poll() is None
        if terminated_by_driver:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

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
    if terminated_by_driver and proc.returncode in (-signal.SIGTERM, 0):
        return 0
    return proc.returncode if proc.returncode is not None else 1


if __name__ == "__main__":
    sys.exit(main())
