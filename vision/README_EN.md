# Vision-Guided 100 m Sprint for Unitree G1

[中文](README.md) | English

A closed-loop 100 m sprint stack for the **29-DoF Unitree G1**, integrating MuJoCo, a D435i-like onboard camera, strict two-boundary lane perception, IMU heading fusion, an `rl_sar` finite-state machine, and a pretrained reinforcement-learning running policy.

> The current results are validated in MuJoCo simulation. This repository does not claim a completed 100 m run on physical G1 hardware.

![Two-boundary perception dashboard](g1_camera_dashboard_official_track.png)

## Highlights

- Reuses the [`C1801SYQ/g1_running`](https://github.com/C1801SYQ/g1_running) 29-DoF TorchScript running policy while keeping perception and joint control separated.
- Detects two real white lane boundaries using exposure-adaptive segmentation, local contrast, motion-blur-aware morphology, and robust Huber line fitting.
- Creates an immutable start-frame lane anchor; short-term tracking may follow camera shake but cannot walk the lock into an adjacent lane.
- Refits fragmented boundaries from real pixels near the locked pair and separates common camera shake from lane-shape changes; it never invents a missing second line.
- Lets the trained policy and G1 IMU hold the straight heading; vision stays neutral inside a center corridor and only applies bounded, hysteretic correction outside it.
- Adds an isolated **Skill 6** to `rl_sar`: state-1-only entry, lane lock, sprint, timed 100 m crossing, visually guided deceleration, and automatic return to Passive.
- Uses the deployed gait-v2 `model_175197` policy for Skills 5/6 and verifies its SHA-256 before building.
- Uses averaged full-attitude homography stabilization, lane-lock zero-bias calibration, IMU straight-heading hold, and bounded vision recovery through the 100 m line.
- Includes watchdog behavior, fall detection, hard adjacent-lane guards, repeatable scene generation, launch scripts, and 118 unit tests.

## Simulation Result

| Metric | Result |
| --- | ---: |
| Two consecutive simulated 100 m runs | 29.17–44.32 s |
| Average forward speed | 2.24–3.43 m/s |
| Maximum velocity command | 4.54–4.58 m/s |
| Valid two-line frame ratio | 99.2%–99.3% |
| Maximum pelvis lateral deviation | 0.543–0.874 m |
| Full stop position | 100.67–101.88 m |
| Falls / adjacent-lane switches / identity loss | 0 / 0 / 0 |

The table is a historical pre-optimization baseline and does not claim results for this revision. The current defaults use gait-v2 `model_175197`, a `0.35 rad/s` yaw-rate limit, `3.00 m/s²` command acceleration, event-driven state-1 readiness, and a 0.60 s Skill-6 camera settling window. Timing begins only after lane lock releases acceleration; hardware deployment still requires staged low-speed validation.

## Architecture

```mermaid
flowchart LR
    A["MuJoCo G1 + onboard camera"] --> B["RGB perception"]
    B --> C["Two-boundary validation and lane lock"]
    C --> D["Lateral and visual heading errors"]
    E["G1 IMU heading"] --> F["Fused velocity controller"]
    D --> F
    F --> G["UDP vx / vy=0 / wz"]
    G --> H["rl_sar Skill 6"]
    H --> I["Pretrained running policy"]
    I --> A
```

## Quick Start

Requirements include Ubuntu 22.04, Python 3.11, MuJoCo 3.x, Unitree SDK2, `unitree_sdk2_python`, and `unitree_mujoco`.

```bash
cd ~/g1_race_vision
conda create -n g1race python=3.11 -y
conda activate g1race
python -m pip install -r requirements.txt

bash scripts/install_cyclonedds_runtime.sh
bash scripts/install_g1_running.sh

install -m 0755 scripts/start_vm_gui.sh ~/start_g1_race.sh
install -m 0755 scripts/stop_vm_gui.sh ~/stop_g1_race.sh
bash ~/start_g1_race.sh
```

After launch, send the keys in the `rl_sar` controller terminal in this order:

```text
Passive --0--> GetUp --1--> state 1 --6--> Skill 6
                                          └--> visual sprint → post-finish deceleration → Passive
```

Key `6` is accepted only from state 1 and never performs an automatic stand-up.

Stop all processes with:

```bash
bash ~/stop_g1_race.sh
```

## Tests

```bash
conda activate g1race
python -m unittest discover -s tests -v
```

The 118 tests cover the deployed policy checksum, strict two-line validation, low light and exposure changes, motion blur, full camera-attitude stabilization, immutable adjacent-lane locking, straight-corridor neutrality, predictive drift confirmation, correction hysteresis, mission reset, perception dropout, finish-line braking, the Skill 7 safety chain, and the event-driven Skill 6 startup handshake.

## Sim-to-Real Status

`scripts/run_realsense_ros2.py` provides a RealSense ROS 2 input path that reuses the same controller. Full attitude homography compensation is currently validated only in the MuJoCo camera path; hardware still needs a synchronized G1/D435i attitude input plus timestamp and extrinsic calibration. Physical deployment also requires exposure and motion-blur testing, an independent emergency-stop chain, low-speed staged validation, and real-track robustness evaluation.

Do not apply the simulated `5.10 m/s` command directly during initial hardware testing.

## Acknowledgements

- [Unitree Robotics / unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [C1801SYQ / g1_running](https://github.com/C1801SYQ/g1_running)
- [MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)
- [RealSense ROS](https://github.com/realsenseai/realsense-ros)

This repository stores integration code and reproducible patches, but does not redistribute complete third-party repositories or their policy binaries. Follow each upstream project's license when installing dependencies.
