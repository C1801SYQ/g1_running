import os
from pathlib import Path
import unittest


VISION_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_CANDIDATE = Path(
    os.environ.get("G1_RUNNING_ROOT", VISION_ROOT.parent)
).expanduser()
VM_REPOSITORY = Path.home() / "unitree_ws/g1_running"

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
    FSM_PATH = VISION_ROOT / "integrations/g1_running/g1_running_skill6.patch"
    VISION_COMMAND_PATH = VISION_ROOT / "integrations/g1_running/vision_udp_command.hpp"
    USING_PATCH_ARTIFACT = True

NUM7_SCRIPT_PATH = VISION_ROOT / "scripts/run_num7_full_demo.sh"
ACTIVE_SCRIPT_PATH = VISION_ROOT / "scripts/run_g1_num7_active.sh"
SMOKE_SCRIPT_PATH = VISION_ROOT / "scripts/num7_headless_smoke.sh"
CONTROLLER_HELPER_PATH = (
    VISION_ROOT / "scripts/run_g1_num7_print_controller_cmd.sh"
)
DRY_RUN_SCRIPT_PATH = VISION_ROOT / "scripts/run_g1_realsense_dry_run.sh"
AUDIT_SCRIPT_PATH = VISION_ROOT / "scripts/run_g1_realsense_audit.sh"
SIM_SCRIPT_PATH = VISION_ROOT / "scripts/run_unitree_camera_sim.py"


class Skill7TransitionTests(unittest.TestCase):
    def test_num7_has_exactly_one_fsm_entry(self) -> None:
        source = FSM_PATH.read_text(encoding="utf-8")
        self.assertEqual(source.count("Input::Keyboard::Num7"), 1)
        self.assertEqual(source.count("Input::Gamepad::LB_DPadLeft"), 1)
        self.assertIn('return "RLFSMStateRLVisionWalk0p5m";', source)

    def test_num7_only_reachable_from_locomotion(self) -> None:
        source = FSM_PATH.read_text(encoding="utf-8")
        if USING_PATCH_ARTIFACT:
            self.skipTest("patch artifact has no full FSM source")
        state1 = source.split(
            "class RLFSMStateRLRoboMimicLocomotion", 1
        )[1].split("class RLFSMStateRLRoboMimicCharleston", 1)[0]
        self.assertIn("Input::Keyboard::Num7", state1)
        self.assertIn('return "RLFSMStateRLVisionWalk0p5m";', state1)
        # Num7 must not be reachable from Passive or GetUp.
        passive = source.split("class RLFSMStatePassive", 1)[1].split(
            "class RLFSMStateGetUp", 1
        )[0]
        self.assertNotIn("RLFSMStateRLVisionWalk0p5m", passive)
        getup = source.split("class RLFSMStateGetUp", 1)[1].split(
            "class RLFSMStateGetDown", 1
        )[0]
        self.assertNotIn("RLFSMStateRLVisionWalk0p5m", getup)

    def test_factory_creates_num7(self) -> None:
        source = FSM_PATH.read_text(encoding="utf-8")
        self.assertIn(
            'else if (state_name == "RLFSMStateRLVisionWalk0p5m")',
            source,
        )
        self.assertIn(
            "std::make_shared<RLFSMStateRLVisionWalk0p5m>(rl)",
            source,
        )

    def test_num7_in_supported_states(self) -> None:
        source = FSM_PATH.read_text(encoding="utf-8")
        self.assertIn('"RLFSMStateRLVisionWalk0p5m"', source)

    def test_num6_and_num7_are_distinct_states(self) -> None:
        source = FSM_PATH.read_text(encoding="utf-8")
        self.assertIn("RLFSMStateRLVisionSprint100m", source)
        self.assertIn("RLFSMStateRLVisionWalk0p5m", source)
        sprint_enter = source.split(
            "class RLFSMStateRLVisionSprint100m", 1
        )[1].split("class RLFSMStateRLVisionWalk0p5m", 1)[0]
        walk_enter = source.split(
            "class RLFSMStateRLVisionWalk0p5m", 1
        )[1].split("class G1FSMFactory", 1)[0]
        self.assertIn("VisionSprintMode::SetEnabled(true)", sprint_enter)
        self.assertIn("VisionSprintMode::SetWalk0p5m(true)", walk_enter)

    def test_num7_walk_mode_payload_present(self) -> None:
        source = VISION_COMMAND_PATH.read_text(encoding="utf-8")
        self.assertIn("G1_VISION_WALK_0P5M 1", source)
        self.assertIn("G1_VISION_WALK_0P5M 0", source)

    def test_num7_exit_disables_walk_mode(self) -> None:
        fsm_source = FSM_PATH.read_text(encoding="utf-8")
        walk_class = fsm_source.split(
            "class RLFSMStateRLVisionWalk0p5m", 1
        )[1].split("class G1FSMFactory", 1)[0]
        self.assertIn("void Exit() override", walk_class)
        self.assertIn("VisionSprintMode::SetWalk0p5m(false)", walk_class)
        self.assertIn("rl.vision_udp_command.ClearForceZero()", walk_class)

    def test_num1_and_num7_reuse_loaded_model(self) -> None:
        """Num1->Num7->Num1 must not re-run InitRL.

        Num1 (locomotion) gates loading behind HasLoadedPolicy.
        Num7 must NEVER load a model in its Enter/control cycle: if the
        locomotion policy is not already loaded it must reject Num7 and
        switch to Passive (behaviour A).
        """
        if USING_PATCH_ARTIFACT:
            self.skipTest("patch artifact has no full FSM source")
        source = FSM_PATH.read_text(encoding="utf-8")
        # Num1 (locomotion) Enter must gate on HasLoadedPolicy.
        loco = source.split("class RLFSMStateRLRoboMimicLocomotion", 1)[
            1
        ].split("class RLFSMStateRLRoboMimicCharleston", 1)[0]
        self.assertIn(
            "if (!rl.HasLoadedPolicy(robot_config_path))", loco
        )
        self.assertIn("rl.InitRL(robot_config_path)", loco)
        # Num7 Enter must NOT load a model in the control cycle (behaviour A):
        # reject and go Passive when the locomotion policy is not loaded.
        walk = source.split("class RLFSMStateRLVisionWalk0p5m", 1)[
            1
        ].split("class G1FSMFactory", 1)[0]
        self.assertIn("vision_walk_entry::Evaluate(", walk)
        self.assertIn("rl.HasLoadedPolicy(robot_config_path)", walk)
        # No InitRL call is allowed inside the Num7 state body.
        self.assertNotIn("rl.InitRL(robot_config_path)", walk)
        self.assertIn('RequestStateChange("RLFSMStatePassive")', walk)
        self.assertIn("rejecting Num7", walk)
        self.assertIn("rl.vision_udp_command.IsReady()", walk)
        # The RL base class must track the loaded config path.
        sdk = Path(REPOSITORY_CANDIDATE) / "rl_sar/src/rl_sar/library/core/rl_sdk/rl_sdk.hpp"
        if sdk.is_file():
            sdk_src = sdk.read_text(encoding="utf-8")
            self.assertIn("loaded_robot_config_path", sdk_src)
            self.assertIn("HasLoadedPolicy", sdk_src)
            self.assertIn("model != nullptr", sdk_src)

    def test_num7_support_release_does_not_wait_for_lane_lock(self) -> None:
        """Startup support protects policy stabilization only; line lock is
        retained for correction calibration and must not authorize motion."""
        if USING_PATCH_ARTIFACT:
            self.skipTest("patch artifact has no full FSM source")
        sim = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        # physics loop fade trigger must include Num7.
        self.assertIn("or num7_enabled.is_set()", sim)
        self.assertIn("correction-only", sim)
        self.assertNotIn("elif not lane_lock_acquired.is_set()", sim)
        # Num7 still records visual lock for correction diagnostics.
        self.assertIn("lane_lock_acquired.set()", sim)
        # Num7 exit clears the correction lock for the next entry.
        self.assertIn("lane_lock_acquired.clear()", sim)
        # Num7 waits only for policy support to fade, not for white lines.
        self.assertIn("NUM7_RESTRAINT_ENGAGED", sim)
        self.assertIn("straight walk started independently of line", sim)
        # New CLI parameter name used by the Num7 launcher.
        self.assertIn("--startup-support-until-mission", sim)
        self.assertIn("startup_support_until_mission", sim)
        # The old Skill 6 flag is retained as a compatibility alias.
        self.assertIn("--startup-support-until-skill6", sim)
        self.assertIn('dest="startup_support_until_mission"', sim)
        # Skill 6 launcher keeps working with the legacy flag name.
        skill6 = VISION_ROOT / "scripts/run_vm_full_demo.sh"
        self.assertIn(
            "--startup-support-until-skill6",
            skill6.read_text(encoding="utf-8"),
        )

    def test_num7_resets_detector_and_controller_on_each_rising_edge(self) -> None:
        """A previous lane-identity loss must not poison the next mission."""
        sim = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        entry = sim.split(
            "if mode_walk and not num7_enabled.is_set():", 1
        )[1].split("elif not mode_walk", 1)[0]
        self.assertIn("detector.reset()", entry)
        self.assertIn("num7_controller.reset()", entry)
        self.assertIn("result = detector.detect(", entry)

        real = (VISION_ROOT / "scripts/run_realsense_ros2.py").read_text(
            encoding="utf-8"
        )
        lifecycle = real.split(
            "def _apply_pending_num7_transitions", 1
        )[1].split("def on_depth", 1)[0]
        self.assertIn("apply_num7_mode_transition(", lifecycle)
        self.assertIn("self._mode_events.put(current_mode)", real)

    def test_real_camera_heading_uses_d435i_color_intrinsics(self) -> None:
        real = (VISION_ROOT / "scripts/run_realsense_ros2.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("/camera/camera/color/camera_info", real)
        self.assertIn("def on_camera_info", real)
        self.assertIn("CameraIntrinsics(", real)
        self.assertIn("self.detector.detect(rgb, depth, intrinsics)", real)

    def test_num7_startup_restraint_fades_before_start_timeout(self) -> None:
        source = NUM7_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("--startup-support-fade-seconds 0.60", source)
        self.assertIn('G1_NUM7_START_TIMEOUT_S:-2.0', source)

    def test_num7_uses_straight_first_predictive_controller(self) -> None:
        sim = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        num7 = sim.split("num7_controller = LaneFollowerController", 1)[1]
        num7 = num7.split("sender = UdpCommandSender", 1)[0]
        self.assertIn("lateral_kp=1.20", num7)

        real = (VISION_ROOT / "scripts/run_realsense_ros2.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("lateral_kp=1.20", real)

    def test_num7_post_stop_report_present(self) -> None:
        """The sim must report commanded vs actual displacement, slide after
        stop, and residual restraint force."""
        sim = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("cmd_distance", sim)
        self.assertIn("actual_displacement", sim)
        self.assertIn("slide_after_stop", sim)
        self.assertIn("residual_xfrc_norm", sim)

    def test_num7_enter_resets_complete_mission_state(self) -> None:
        """A second Num7 mission must start fresh (blocking issue 4)."""
        if USING_PATCH_ARTIFACT:
            self.skipTest("patch artifact has no full FSM source")
        source = FSM_PATH.read_text(encoding="utf-8")
        walk = source.split("class RLFSMStateRLVisionWalk0p5m", 1)[
            1
        ].split("class G1FSMFactory", 1)[0]
        for field in (
            "elapsed_s_ = 0.0f;",
            "estimated_distance_m_ = 0.0f;",
            "started_ = false;",
            "stop_latched_ = false;",
            "mission_complete_ = false;",
            "stop_reason_.clear();",
            "stop_hold_started_at_ = std::chrono::steady_clock::time_point{};",
            "target_m_ = 0.0f;",
            "max_duration_s_ = 0.0f;",
            "rl.vision_udp_command.ClearSession();",
        ):
            self.assertIn(field, walk)

    def test_num7_heartbeat_repeats_enable(self) -> None:
        if USING_PATCH_ARTIFACT:
            self.skipTest("patch artifact has no full FSM source")
        source = FSM_PATH.read_text(encoding="utf-8")
        walk = source.split("class RLFSMStateRLVisionWalk0p5m", 1)[
            1
        ].split("class G1FSMFactory", 1)[0]
        self.assertIn("VisionSprintMode::Heartbeat()", walk)
        self.assertIn("std::chrono::milliseconds(500)", walk)


    def test_num7_hard_limits_in_receiver(self) -> None:
        source = VISION_COMMAND_PATH.read_text(encoding="utf-8")
        self.assertIn("kNum7MaxVx", source)
        self.assertIn("kNum7MaxWz", source)

    def test_num7_full_demo_script_requirements(self) -> None:
        source = NUM7_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("UNITREE_ROOT", source)
        self.assertIn("G1_RUNNING_ROOT", source)
        self.assertIn("--mission walk0p5m", source)
        self.assertIn("--speed 0.50", source)
        self.assertIn('G1_NUM7_TARGET_M:-1.00', source)
        self.assertIn("echo \"  7  ", source)
        self.assertIn("G1_NUM7_TARGET_M", source)
        self.assertIn("G1_NUM7_MAX_DURATION_S", source)
        # Uses the mission-aware support flag, not the Skill 6-only name.
        self.assertIn("--startup-support-until-mission", source)
        # Must not reuse the 100 m finish/stop logic.
        self.assertNotIn("--finish-line-x 100.0", source)
        self.assertNotIn("--stop-at-x 115.0", source)

    def test_num7_active_script_requires_opt_in(self) -> None:
        source = ACTIVE_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("G1_NUM7_COMMAND_OUTPUT_ENABLED", source)
        self.assertIn("HARDWARE_RELEASE_READY", source)
        self.assertIn("policy_sha256", source)
        self.assertIn("fsm_sha256", source)
        self.assertIn("receiver_sha256", source)
        self.assertIn("measured-qpos simulation gate has not passed", source)
        self.assertIn("exit 3", source)
        self.assertIn("command_output_enabled:=true", source)
        self.assertIn("mission:=walk0p5m", source)
        self.assertIn("G1_ROBOT_INTERFACE", source)
        self.assertIn('ROBOT_INTERFACE}" == "lo"', source)
        self.assertIn("rgb_camera.color_profile:=640,480,15", source)
        self.assertIn("depth_module.depth_profile:=640,480,15", source)
        self.assertIn("enable_gyro:=false", source)
        self.assertIn("enable_accel:=false", source)
        self.assertIn("aligned_depth_to_color/image_raw --once", source)
        self.assertNotIn("./cmake_build/bin/rl_real_g1 lo", source)

    def test_num7_audit_uses_real_configuration_without_motion(self) -> None:
        dry_run = DRY_RUN_SCRIPT_PATH.read_text(encoding="utf-8")
        audit = AUDIT_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn('G1_AUDIT_MISSION:-walk0p5m', dry_run)
        self.assertIn('G1_NUM7_TARGET_M:-1.00', dry_run)
        self.assertIn('G1_VISION_CRUISE_SPEED_MPS:-0.50', dry_run)
        self.assertIn("command_output_enabled:=false", dry_run)
        self.assertIn("rgb_camera.color_profile:=640,480,15", dry_run)
        self.assertIn("depth_module.depth_profile:=640,480,15", dry_run)
        self.assertIn("aligned_depth_to_color/image_raw --once", dry_run)
        self.assertIn("G1_AUDIT_MISSION=walk0p5m", audit)
        self.assertIn("G1_VISION_WALK_0P5M 1", audit)
        self.assertIn("command_output_enabled=false", audit)

    def test_num7_controller_helper_requires_physical_interface(self) -> None:
        source = CONTROLLER_HELPER_PATH.read_text(encoding="utf-8")
        self.assertIn("G1_ROBOT_INTERFACE", source)
        self.assertIn("ip link show dev", source)
        self.assertIn("simulation-only", source)
        self.assertNotIn("./cmake_build/bin/rl_real_g1 lo", source)

    def test_num7_smoke_has_strict_runtime_assertions(self) -> None:
        source = SMOKE_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("G1_NUM7_SMOKE_COMMAND_PORT", source)
        self.assertIn("G1_NUM7_SMOKE_STATUS_PORT", source)
        self.assertIn("could not bind|Address already in use", source)
        self.assertIn("first velocity command received", source)
        self.assertIn("DISTANCE_REACHED|HARD_STOP", source)
        self.assertIn("post-stop report:.*zero_command=True", source)
        self.assertIn("G1_NUM7_MIN_ACTUAL_DISPLACEMENT_M", source)
        self.assertIn("actual_displacement", source)
        self.assertIn("measured qpos displacement was outside", source)
        self.assertIn("HARDWARE_RELEASE_READY", source)
        self.assertIn("NUM7_HEADLESS_SMOKE_PASSED", source)

    def test_sim_script_supports_walk0p5m(self) -> None:
        source = SIM_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("walk0p5m", source)
        self.assertIn("Walk0p5mGate", source)
        self.assertIn("VisionMode.WALK0P5M", source)

    def test_rl_sim_joystick_mapping_not_inverted(self) -> None:
        repo = REPOSITORY_CANDIDATE
        if not USING_PATCH_ARTIFACT:
            rl_sim = repo / "rl_sar/src/rl_sar/src/rl_sim.cpp"
            if rl_sim.is_file():
                source = rl_sim.read_text(encoding="utf-8")
                self.assertIn(
                    "axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);",
                    source,
                )
                self.assertIn(
                    "axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);",
                    source,
                )


if __name__ == "__main__":
    unittest.main()
