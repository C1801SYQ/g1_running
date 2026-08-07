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
POLICY_REPOSITORY = (
    REPOSITORY_CANDIDATE
    if (
        REPOSITORY_CANDIDATE / "rl_sar/policy/g1/running/policy.pt"
    ).is_file()
    else VM_REPOSITORY
)
POLICY_PATH = POLICY_REPOSITORY / "rl_sar/policy/g1/running/policy.pt"
POLICY_SHA256 = "167b444f7404a21a4751336b8f7e54c5b0b7cd4b7c979b5ad895288867330305"


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
        source = FSM_PATH.read_text(encoding="utf-8")
        self.assertEqual(source.count("Input::Keyboard::Num6"), 1)

        if USING_PATCH_ARTIFACT:
            self.assertIn("RLFSMStateRLRoboMimicLocomotion", source)
            self.assertIn('return "RLFSMStateRLVisionSprint100m";', source)
        else:
            state1 = source.split(
                "class RLFSMStateRLRoboMimicLocomotion", 1
            )[1].split("class RLFSMStateRLRoboMimicCharleston", 1)[0]
            self.assertIn("Input::Keyboard::Num6", state1)
            self.assertIn('return "RLFSMStateRLVisionSprint100m";', state1)

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
        self.assertIn("lane_lock_acquired.is_set()", simulator_source)
        self.assertIn("--startup-support-until-skill6", run_source)
        self.assertIn("G1_SKILL6_STABILIZE_SECONDS:-5.0", run_source)
        self.assertIn("G1_VISION_STATUS_PORT", run_source)
        self.assertIn("--max-yaw-rate", run_source)
        self.assertIn("G1_RACE_MAX_YAW_RATE:-0.80", run_source)
        self.assertIn("G1_RACE_LATERAL_KP:-1.25", run_source)
        self.assertIn("G1_RACE_HEADING_KP:-0.20", run_source)
        self.assertIn("G1_RACE_IMU_HEADING_KP:-1.10", run_source)
        self.assertIn("G1_RACE_ERROR_FILTER_ALPHA:-0.45", run_source)
        self.assertIn("G1_VISION_MAX_WZ", run_source)
        self.assertIn("G1_VISION_SPRINT 1", command_source)
        self.assertIn("G1_VISION_STATUS_PORT", command_source)


if __name__ == "__main__":
    unittest.main()
