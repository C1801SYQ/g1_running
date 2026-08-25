import hashlib
import os
from pathlib import Path
import unittest


VISION_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_CANDIDATE = Path(
    os.environ.get("G1_RUNNING_ROOT", VISION_ROOT.parent)
).expanduser()
VM_REPOSITORY = Path.home() / "unitree_ws/g1_running"
INTEGRATION_ROOT = VISION_ROOT / "integrations/g1_running"

if (
    REPOSITORY_CANDIDATE
    / "rl_sar/src/rl_sar/include/vision_udp_command.hpp"
).is_file():
    FSM_PATH = (
        REPOSITORY_CANDIDATE
        / "rl_sar/src/rl_sar/fsm_robot/fsm_g1.hpp"
    )
    VISION_COMMAND_PATH = (
        REPOSITORY_CANDIDATE
        / "rl_sar/src/rl_sar/include/vision_udp_command.hpp"
    )
    USING_PATCH_ARTIFACT = False
elif (
    VM_REPOSITORY / "rl_sar/src/rl_sar/include/vision_udp_command.hpp"
).is_file():
    FSM_PATH = VM_REPOSITORY / "rl_sar/src/rl_sar/fsm_robot/fsm_g1.hpp"
    VISION_COMMAND_PATH = (
        VM_REPOSITORY / "rl_sar/src/rl_sar/include/vision_udp_command.hpp"
    )
    USING_PATCH_ARTIFACT = False
else:
    FSM_PATH = INTEGRATION_ROOT / "g1_running_skill6.patch"
    VISION_COMMAND_PATH = INTEGRATION_ROOT / "vision_udp_command.hpp"
    USING_PATCH_ARTIFACT = True
RUN_SCRIPT_PATH = VISION_ROOT / "scripts/run_vm_full_demo.sh"
START_SCRIPT_PATH = VISION_ROOT / "scripts/start_vm_gui.sh"
ENV_SCRIPT_PATH = VISION_ROOT / "scripts/activate_g1race_dds.sh"
SIM_SCRIPT_PATH = VISION_ROOT / "scripts/run_unitree_camera_sim.py"
INSTALL_SCRIPT_PATH = VISION_ROOT / "scripts/install_g1_running.sh"
BUILD_SCRIPT_PATH = VISION_ROOT / "scripts/build_skill6.sh"
PREPARE_SCRIPT_PATH = VISION_ROOT / "scripts/run_skill6_realsense_prepare.sh"
DRY_RUN_SCRIPT_PATH = VISION_ROOT / "scripts/run_g1_realsense_dry_run.sh"
POLICY_REPOSITORY = (
    REPOSITORY_CANDIDATE
    if (
        REPOSITORY_CANDIDATE / "rl_sar/policy/g1/running/policy.pt"
    ).is_file()
    else VM_REPOSITORY
)
POLICY_PATH = POLICY_REPOSITORY / "rl_sar/policy/g1/running/policy.pt"
POLICY_SHA256 = "c4890005e430895feeb1aa8a80fa71f5e7fe96f7c7b45652bb709807d24930b2"
RUNNING_ADAPTER_PATH = (
    REPOSITORY_CANDIDATE
    / "rl_sar/src/rl_sar/fsm_robot/fsm_g1_running_adapter.hpp"
)


class Skill6TransitionTests(unittest.TestCase):
    def test_latest_running_policy_checksum_matches_build_scripts(self) -> None:
        digest = hashlib.sha256(POLICY_PATH.read_bytes()).hexdigest()
        self.assertEqual(digest, POLICY_SHA256)
        self.assertIn(
            POLICY_SHA256,
            INSTALL_SCRIPT_PATH.read_text(encoding="utf-8"),
        )
        self.assertIn(
            POLICY_SHA256,
            BUILD_SCRIPT_PATH.read_text(encoding="utf-8"),
        )

    def test_num6_has_exactly_one_fsm_entry(self) -> None:
        if not RUNNING_ADAPTER_PATH.is_file():
            self.skipTest("running adapter is unavailable in patch-only mode")
        source = RUNNING_ADAPTER_PATH.read_text(encoding="utf-8")
        state1 = source.split(
            "class RLFSMStateLocomotionRunning", 1
        )[1].split("class RLFSMStateRunning", 1)[0]
        self.assertEqual(state1.count("Input::Keyboard::Num6"), 1)
        self.assertIn('return "RLFSMStateRLRunningStraight110m";', state1)

    def test_automatic_skill6_launch_is_removed(self) -> None:
        fsm_source = FSM_PATH.read_text(encoding="utf-8")
        command_source = VISION_COMMAND_PATH.read_text(encoding="utf-8")
        combined = fsm_source + command_source

        self.assertNotIn("G1_AUTO_VISION_SPRINT", combined)
        self.assertNotIn("RequestLaunch", combined)
        self.assertNotIn("ConsumeLaunchRequest", combined)
        self.assertFalse(
            (INTEGRATION_ROOT / "g1_running_skill6_one_key.patch").exists()
        )

    def test_launcher_keeps_rl_sar_interactive(self) -> None:
        source = RUN_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertNotIn("printf '6'", source)
        self.assertIn('"${RUNNING_BINARY}" "${G1_RACE_INTERFACE}"', source)
        self.assertIn('echo "  0  ', source)
        self.assertIn('echo "  1  ', source)
        self.assertIn('echo "  6  ', source)

    def test_g1race_dds_configuration_is_explicit(self) -> None:
        source = ENV_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("conda activate g1race", source)
        self.assertIn("cyclonedds-0.10.2/install", source)
        self.assertIn("G1_RACE_DOMAIN_ID=0", source)
        self.assertIn('G1_RACE_INTERFACE="${G1_RACE_INTERFACE:-lo}"', source)
        self.assertIn("<SharedMemory><Enable>false</Enable></SharedMemory>", source)

    def test_home_launcher_finds_vm_deployment(self) -> None:
        source = START_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn(
            '${HOME}/g1_race_vision/scripts/run_vm_full_demo.sh', source
        )

    def test_startup_support_waits_for_actual_skill6(self) -> None:
        simulator_source = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        run_source = RUN_SCRIPT_PATH.read_text(encoding="utf-8")
        command_source = VISION_COMMAND_PATH.read_text(encoding="utf-8")

        self.assertIn("UdpSkill6StatusReceiver", simulator_source)
        self.assertIn('"WAIT_FOR_SKILL6"', simulator_source)
        self.assertIn("skill6_enabled.is_set()", simulator_source)
        self.assertIn(
            '"WAIT_FOR_POLICY_SUPPORT_RELEASE"', simulator_source
        )
        self.assertIn("correction-only", simulator_source)
        self.assertIn(
            'race_timing["heading_yaw"] = support_yaw',
            simulator_source,
        )
        self.assertIn("--startup-support-until-skill6", run_source)
        self.assertIn("G1_SKILL6_STABILIZE_SECONDS:-0.60", run_source)
        self.assertIn("G1_SKILL6_RAMP_TO_1:-0.60", run_source)
        self.assertIn("G1_SKILL6_RAMP_TO_3:-0.80", run_source)
        self.assertIn("G1_SKILL6_RAMP_TO_MAX:-1.00", run_source)
        self.assertIn("max_camera_reference_samples", simulator_source)
        self.assertIn("G1_RACE_ACCEL:-3.00", run_source)
        self.assertIn("--vision-enable-delay 0.0", run_source)
        self.assertIn("--startup-support-fade-seconds 3.0", run_source)
        self.assertIn("G1_VISION_STATUS_PORT", run_source)
        self.assertIn("--max-yaw-rate", run_source)
        self.assertIn("G1_RACE_MAX_YAW_RATE:-0.35", run_source)
        self.assertIn("G1_RACE_LATERAL_KP:-1.20", run_source)
        self.assertIn("G1_RACE_HEADING_KP:-0.20", run_source)
        self.assertIn("G1_RACE_IMU_HEADING_KP:-1.20", run_source)
        self.assertIn("G1_RACE_ERROR_FILTER_ALPHA:-0.32", run_source)
        self.assertIn("G1_VISION_MAX_WZ", run_source)
        self.assertIn("G1_VISION_SPRINT 1", command_source)
        self.assertIn("G1_VISION_STATUS_PORT", command_source)

    def test_skill6_speed_ramp_starts_after_fsm_enable(self) -> None:
        simulator_source = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        realsense_source = (
            VISION_ROOT / "scripts/run_realsense_ros2.py"
        ).read_text(encoding="utf-8")

        self.assertIn("sprint_ramp_started_at = now", simulator_source)
        self.assertIn("sprint_speed_cap", simulator_source)
        self.assertIn("Sprint FSM enable received", realsense_source)
        self.assertIn('"WAIT_FOR_SKILL6"', realsense_source)
        self.assertIn("clamp_command_vx", realsense_source)

    def test_skill6_prepare_is_dry_run_only(self) -> None:
        prepare_source = PREPARE_SCRIPT_PATH.read_text(encoding="utf-8")
        dry_run_source = DRY_RUN_SCRIPT_PATH.read_text(encoding="utf-8")
        scripts_readme = (VISION_ROOT / "scripts/README.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("G1_AUDIT_MISSION=sprint100m", prepare_source)
        self.assertIn("G1_VISION_SPRINT 1", prepare_source)
        self.assertIn("command_output_enabled=false", prepare_source)
        self.assertIn("rl_real_g1", prepare_source)
        self.assertNotIn('"${RUNNING_BINARY}"', prepare_source)
        self.assertNotIn("systemctl", prepare_source)
        self.assertIn('STATUS_PORT="${G1_VISION_STATUS_PORT:-15002}"', dry_run_source)
        self.assertIn("skill6_ramp_to_1_s", dry_run_source)
        self.assertIn("Skill 6 当前只有仿真和 prepare-only", scripts_readme)

    def test_auto_start_waits_for_state1_ack_not_fixed_three_seconds(self) -> None:
        simulator_source = SIM_SCRIPT_PATH.read_text(encoding="utf-8")

        self.assertIn("def wait_for_state1", simulator_source)
        self.assertIn("if not wait_for_state1():", simulator_source)
        sprint_section = simulator_source.split(
            "# Default: Skill 6 (sprint100m)", 1
        )[1].split("# The SDK filters analog triggers", 1)[0]
        self.assertNotIn("stop.wait(3.0)", sprint_section)

    def test_skill6_announces_mode_only_after_policy_load(self) -> None:
        fsm_source = FSM_PATH.read_text(encoding="utf-8")
        skill6 = fsm_source.split(
            "class RLFSMStateRLVisionSprint100m", 1
        )[1].split("class RLFSMStateRLVisionWalk0p5m", 1)[0]

        enable_index = skill6.index("VisionSprintMode::SetEnabled(true)")
        if "rl.HasLoadedPolicy(robot_config_path)" in skill6:
            load_index = skill6.index("rl.HasLoadedPolicy(robot_config_path)")
        else:
            load_index = skill6.index("rl.InitRL(robot_config_path)")
        self.assertLess(load_index, enable_index)

    def test_skill6_clears_stale_policy_targets(self) -> None:
        sdk_path = (
            REPOSITORY_CANDIDATE
            / "rl_sar/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp"
        )
        if not sdk_path.is_file():
            self.skipTest("rl_sdk source is unavailable in patch-only mode")
        sdk_source = sdk_path.read_text(encoding="utf-8")

        self.assertIn("output_dof_pos_queue.try_pop(stale_output)", sdk_source)


if __name__ == "__main__":
    unittest.main()
