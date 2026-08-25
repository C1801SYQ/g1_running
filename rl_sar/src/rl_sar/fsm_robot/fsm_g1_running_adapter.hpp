/*
 * Incremental G1 running-policy adapter.
 *
 * This file is intentionally separate from the upstream G1 FSM states.  It
 * adds a guarded entry for policy/g1/running without changing the existing
 * robomimic or whole-body-tracking state implementations.
 */
#ifndef G1_RUNNING_ADAPTER_HPP
#define G1_RUNNING_ADAPTER_HPP

#include "fsm_g1.hpp"
#include "g1_leg_odometry.hpp"
#include "vision_udp_command.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <vector>

namespace g1_running_adapter
{

class RLFSMStateLocomotionRunning
    : public g1_fsm::RLFSMStateRLRoboMimicLocomotion
{
public:
    explicit RLFSMStateLocomotionRunning(RL* rl)
        : g1_fsm::RLFSMStateRLRoboMimicLocomotion(rl) {}

    void Enter() override
    {
        g1_fsm::RLFSMStateRLRoboMimicLocomotion::Enter();
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num5 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRLRunning";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num6 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
        {
            return "RLFSMStateRLRunningStraight110m";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num7 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadLeft)
        {
            return "RLFSMStateRLRunningVision110m";
        }
        return g1_fsm::RLFSMStateRLRoboMimicLocomotion::CheckChange();
    }
};

class RLFSMStateRunning : public RLFSMState
{
public:
    explicit RLFSMStateRunning(RL* rl)
        : RLFSMState(*rl, "RLFSMStateRLRunning") {}

    void Enter() override
    {
        entry_failed_ = false;
        rl.episode_length_buf = 0;

        // Exact locomotion -> dance handoff shape: select the policy,
        // initialize it, then capture the live FSM state.  No running-only
        // pose hold, action-queue cleanup, command gate, or phase reset is
        // inserted here.
        rl.config_name = "running";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            entry_failed_ = true;
            std::cout << LOGGER::ERROR << "Running InitRL() failed: "
                      << e.what() << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (entry_failed_)
            return;
        if (!rl.rl_init_done)
            rl.rl_init_done = true;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (entry_failed_)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
            return "RLFSMStateGetDown";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num6 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
            return "RLFSMStateRLRunningStraight110m";
        return state_name_;
    }

protected:
    bool EntrySucceeded() const { return !entry_failed_; }

private:
    bool entry_failed_ = false;
};

// Additive Skill 6: run the loaded 96D running policy in a straight mission
// without a camera.  Heading is held against the initial IMU yaw and distance
// is closed by the additive leg odometer.  No pose hold, action-queue cleanup,
// phase reset, or running-specific handoff is introduced.  The handoff is
// exactly the inherited locomotion -> running path above.
class RLFSMStateRunningStraight110m : public RLFSMStateRunning
{
public:
    explicit RLFSMStateRunningStraight110m(RL* rl)
        : RLFSMStateRunning(rl)
    {
        state_name_ = "RLFSMStateRLRunningStraight110m";
    }

    void Enter() override
    {
        RLFSMStateRunning::Enter();
        if (!EntrySucceeded())
            return;

        target_distance_m_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_TARGET_M", 110.0f, 1.0f, 500.0f);
        cruise_speed_mps_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_SPEED_MPS", 3.5f, 3.0f, 4.0f);
        ramp_seconds_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_RAMP_S", 5.0f, 0.1f, 30.0f);
        stop_hold_seconds_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_STOP_HOLD_S", 0.5f, 0.1f, 5.0f);
        max_duration_seconds_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_MAX_DURATION_S", 120.0f, 5.0f, 1200.0f);
        heading_kp_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_HEADING_KP", 1.50f, 0.0f, 8.0f);
        heading_kd_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_HEADING_KD", 0.08f, 0.0f, 2.0f);
        heading_max_command_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_HEADING_MAX_CMD", 0.30f, 0.02f, 1.0f);
        heading_rate_limit_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_HEADING_RATE_LIMIT", 0.60f, 0.05f, 5.0f);
        odom_start_timeout_seconds_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_ODOM_START_TIMEOUT_S", 8.0f, 1.0f, 30.0f);
        odom_stale_timeout_seconds_ = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_ODOM_STALE_TIMEOUT_S", 1.0f, 0.2f, 5.0f);
        odom_min_samples_ = ReadIntEnv(
            "G1_RUNNING_STRAIGHT_ODOM_MIN_SAMPLES", 3, 1, 100);
        require_leg_odom_ = ReadBoolEnv(
            "G1_RUNNING_STRAIGHT_REQUIRE_LEG_ODOM", true);

        G1LegOdometry::Config odom_config;
        odom_config.contact_z_max = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_CONTACT_Z_MAX", -0.52f, -1.5f, -0.10f);
        odom_config.contact_velocity_max = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_CONTACT_VELOCITY_MAX", 2.0f, 0.10f, 8.0f);
        odom_config.velocity_filter = ReadFloatEnv(
            "G1_RUNNING_STRAIGHT_ODOM_VELOCITY_FILTER", 0.35f, 0.01f, 1.0f);
        leg_odometry_.Configure(odom_config);

        elapsed_seconds_ = 0.0f;
        mission_complete_ = false;
        stop_latched_ = false;
        odom_ready_ = false;
        odom_valid_samples_ = 0;
        last_odom_valid_elapsed_seconds_ = 0.0f;
        start_yaw_rad_ = G1LegOdometry::YawFromQuaternion(
            fsm_state->imu.quaternion);
        heading_command_ = 0.0f;
        last_step_at_ = Clock::now();
        stop_hold_started_at_ = Clock::time_point{};
        stop_reason_.clear();
        leg_odometry_.Reset(*fsm_state, start_yaw_rad_);

        // Keep the first running-policy command at zero, then ramp it
        // smoothly.  This is a command profile, not a separate transition
        // path, and does not add any pose/queue/phase handling.
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        std::cout << LOGGER::NOTE
                  << "Skill 6 entered: running policy + straight 110 m "
                     "(IMU heading + leg odometry, no vision), speed="
                  << cruise_speed_mps_ << " m/s, start_yaw="
                  << start_yaw_rad_ << std::endl;
    }

    void Run() override
    {
        if (!EntrySucceeded())
            return;

        const auto now = Clock::now();
        const float dt = std::clamp(
            std::chrono::duration_cast<std::chrono::duration<float>>(
                now - last_step_at_)
                .count(),
            1e-3f,
            0.25f);
        last_step_at_ = now;
        elapsed_seconds_ += dt;

        if (!stop_latched_)
        {
            const float ramp = std::clamp(
                elapsed_seconds_ / std::max(ramp_seconds_, 1e-3f),
                0.0f,
                1.0f);
            const float smooth = ramp * ramp * (3.0f - 2.0f * ramp);
            const float command_speed = cruise_speed_mps_ * smooth;
            rl.control.x = command_speed;
            rl.control.y = 0.0f;

            const float current_yaw = G1LegOdometry::YawFromQuaternion(
                fsm_state->imu.quaternion);
            const float yaw_error = WrapAngle(start_yaw_rad_ - current_yaw);
            const float gyro_z = fsm_state->imu.gyroscope.size() >= 3 &&
                                 std::isfinite(fsm_state->imu.gyroscope[2])
                                     ? fsm_state->imu.gyroscope[2]
                                     : 0.0f;
            const float desired_yaw_command = std::clamp(
                heading_kp_ * yaw_error - heading_kd_ * gyro_z,
                -heading_max_command_, heading_max_command_);
            const float max_delta = heading_rate_limit_ * dt;
            heading_command_ = std::clamp(
                desired_yaw_command,
                heading_command_ - max_delta,
                heading_command_ + max_delta);
            rl.control.yaw = heading_command_;

            const auto odom = leg_odometry_.Update(
                *fsm_state, dt, command_speed, current_yaw);
            if (odom.valid)
            {
                ++odom_valid_samples_;
                last_odom_valid_elapsed_seconds_ = elapsed_seconds_;
                if (!odom_ready_ && odom_valid_samples_ >= odom_min_samples_)
                {
                    odom_ready_ = true;
                    std::cout << LOGGER::NOTE
                              << "Skill 6 leg odometry ready: contacts="
                              << odom.contacts << ", forward="
                              << odom.forward_m << " m, lateral="
                              << odom.lateral_m << " m" << std::endl;
                }
            }
            if (odom_ready_ && odom.forward_m >= target_distance_m_)
                RequestStop("DISTANCE_REACHED_LEG_ODOM");
            else if (require_leg_odom_ &&
                     !odom_ready_ &&
                     elapsed_seconds_ >= odom_start_timeout_seconds_)
                RequestStop("NO_LEG_ODOM");
            else if (odom_ready_ &&
                     elapsed_seconds_ - last_odom_valid_elapsed_seconds_ >=
                         odom_stale_timeout_seconds_)
                RequestStop("LEG_ODOM_STALE");
            else if (elapsed_seconds_ >= max_duration_seconds_)
                RequestStop("MAX_DURATION");
        }

        if (stop_latched_)
        {
            rl.control.x = 0.0f;
            rl.control.y = 0.0f;
            rl.control.yaw = 0.0f;
            if (stop_hold_started_at_ == Clock::time_point{})
            {
                stop_hold_started_at_ = now;
                std::cout << LOGGER::NOTE
                          << "Skill 6 straight stop latched: "
                          << stop_reason_ << std::endl;
            }
            const float stopped_for =
                std::chrono::duration_cast<std::chrono::duration<float>>(
                    now - stop_hold_started_at_)
                    .count();
            if (stopped_for >= stop_hold_seconds_)
            {
                mission_complete_ = true;
                std::cout << LOGGER::NOTE
                          << "Skill 6 straight mission complete; distance="
                          << leg_odometry_.LastEstimate().forward_m << " m"
                          << ", lateral="
                          << leg_odometry_.LastEstimate().lateral_m << " m"
                          << " (leg odometry, samples="
                          << leg_odometry_.LastEstimate().samples << ")"
                          << std::endl;
            }
        }

        RLFSMStateRunning::Run();
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
            return "RLFSMStateGetDown";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
            rl.control.current_gamepad == Input::Gamepad::RB_DPadUp ||
            mission_complete_)
            return "RLFSMStateRLRoboMimicLocomotion";
        if (rl.control.current_keyboard == Input::Keyboard::Num5 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
            return "RLFSMStateRLRunning";
        return state_name_;
    }

    void Exit() override
    {
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        RLFSMStateRunning::Exit();
    }

private:
    using Clock = std::chrono::steady_clock;

    static float ReadFloatEnv(
        const char *name, float fallback, float minimum, float maximum)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
            return fallback;
        char *end = nullptr;
        const float value = std::strtof(text, &end);
        if (end == text || *end != '\0' || !std::isfinite(value))
            return fallback;
        return std::clamp(value, minimum, maximum);
    }

    static int ReadIntEnv(
        const char *name, int fallback, int minimum, int maximum)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
            return fallback;
        char *end = nullptr;
        const long value = std::strtol(text, &end, 10);
        if (end == text || *end != '\0')
            return fallback;
        return std::clamp(static_cast<int>(value), minimum, maximum);
    }

    static bool ReadBoolEnv(const char *name, bool fallback)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
            return fallback;
        return std::string(text) == "1" ||
               std::string(text) == "true" ||
               std::string(text) == "TRUE" ||
               std::string(text) == "yes";
    }

    static float WrapAngle(float angle)
    {
        constexpr float kPi = 3.14159265358979323846f;
        while (angle > kPi)
            angle -= 2.0f * kPi;
        while (angle < -kPi)
            angle += 2.0f * kPi;
        return angle;
    }

    void RequestStop(const std::string &reason)
    {
        if (stop_latched_)
            return;
        stop_latched_ = true;
        stop_reason_ = reason;
    }

    float target_distance_m_ = 110.0f;
    float cruise_speed_mps_ = 3.5f;
    float ramp_seconds_ = 5.0f;
    float stop_hold_seconds_ = 0.5f;
    float max_duration_seconds_ = 120.0f;
    float elapsed_seconds_ = 0.0f;
    float start_yaw_rad_ = 0.0f;
    float heading_kp_ = 1.50f;
    float heading_kd_ = 0.08f;
    float heading_max_command_ = 0.30f;
    float heading_rate_limit_ = 0.60f;
    float odom_start_timeout_seconds_ = 8.0f;
    float odom_stale_timeout_seconds_ = 1.0f;
    int odom_min_samples_ = 3;
    int odom_valid_samples_ = 0;
    float heading_command_ = 0.0f;
    float last_odom_valid_elapsed_seconds_ = 0.0f;
    bool require_leg_odom_ = true;
    bool odom_ready_ = false;
    bool stop_latched_ = false;
    bool mission_complete_ = false;
    std::string stop_reason_;
    G1LegOdometry leg_odometry_;
    Clock::time_point last_step_at_{};
    Clock::time_point stop_hold_started_at_{};
};

// Num7 running-backed visual mission.  The policy handoff is inherited from
// RLFSMStateRunning so it remains exactly InitRL(running) -> live-state
// capture -> RLControl, matching the existing locomotion/dance path.
class RLFSMStateRunningVision110m : public RLFSMStateRunning
{
public:
    explicit RLFSMStateRunningVision110m(RL* rl)
        : RLFSMStateRunning(rl)
    {
        state_name_ = "RLFSMStateRLRunningVision110m";
    }

    void Enter() override
    {
        RLFSMStateRunning::Enter();

        if (!EntrySucceeded())
        {
            std::cout << LOGGER::ERROR
                      << "Running vision policy initialization failed; "
                         "entry rejected"
                      << std::endl;
            return;
        }
        if (!vision_udp_command_.IsReady())
        {
            std::cout << LOGGER::ERROR
                      << "Running vision UDP receiver is unavailable; "
                         "entry rejected"
                      << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        vision_udp_command_.ClearSession();
        VisionSprintMode::SetWalk0p5m(true);
        started_ = false;
        stop_latched_ = false;
        mission_complete_ = false;
        elapsed_s_ = 0.0f;
        stop_reason_.clear();
        const auto now = Clock::now();
        last_step_at_ = now;
        stop_hold_started_at_ = Clock::time_point{};
        last_heartbeat_at_ = Clock::time_point{};
        std::cout << LOGGER::NOTE
                  << "Num7 entered: running policy + visual 110 m mission"
                  << std::endl;
    }

    void Run() override
    {
        const auto now = Clock::now();
        const float dt = std::max(
            1e-3f,
            std::chrono::duration_cast<std::chrono::duration<float>>(
                now - last_step_at_)
                .count());
        last_step_at_ = now;
        elapsed_s_ += dt;

        if (now - last_heartbeat_at_ >= std::chrono::milliseconds(500))
        {
            VisionSprintMode::Heartbeat();
            last_heartbeat_at_ = now;
        }

        // RL_Real applies the same receiver in GetState().  Applying here is
        // idempotent and gives MuJoCo the identical UDP command lifecycle.
        vision_udp_command_.Apply(
            rl.control.x, rl.control.y, rl.control.yaw);

        if (!stop_latched_)
        {
            if (!started_)
            {
                if (vision_udp_command_.HasFreshCommand() &&
                    rl.control.x > 0.0f)
                {
                    started_ = true;
                    std::cout << LOGGER::NOTE
                              << "Running vision forward command received"
                              << std::endl;
                }
                else if (elapsed_s_ >= ReadFloatEnv(
                             "G1_RUNNING_VISION_START_TIMEOUT_S", 2.0f,
                             0.5f, 10.0f))
                {
                    RequestStop("START_TIMEOUT_NO_FORWARD_CMD");
                }
            }
            if (started_)
            {
                if (vision_udp_command_.HardStopRequested())
                    RequestStop("VISION_HARD_STOP");
                else if (vision_udp_command_.LastReceiveAgeMs() >
                         ReadIntEnv("G1_RUNNING_VISION_TIMEOUT_MS", 300, 50,
                                    5000))
                    RequestStop("UDP_STALE");
                else if (elapsed_s_ >= ReadFloatEnv(
                             "G1_RUNNING_VISION_MAX_DURATION_S", 260.0f,
                             1.0f, 1200.0f))
                    RequestStop("MAX_DURATION");
            }
        }

        if (stop_latched_)
        {
            rl.control.x = 0.0f;
            rl.control.y = 0.0f;
            rl.control.yaw = 0.0f;
            vision_udp_command_.ForceZero();
            if (stop_hold_started_at_ == Clock::time_point{})
            {
                stop_hold_started_at_ = now;
                std::cout << LOGGER::NOTE
                          << "Running vision stop latched: " << stop_reason_
                          << std::endl;
            }
            const float settle_s = ReadFloatEnv(
                "G1_RUNNING_VISION_SETTLE_S", 0.5f, 0.1f, 5.0f);
            if (std::chrono::duration_cast<std::chrono::duration<float>>(
                    now - stop_hold_started_at_)
                    .count() >= settle_s)
            {
                mission_complete_ = true;
            }
        }

        // Preserve the inherited running tilt protection and RLControl path.
        RLFSMStateRunning::Run();
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P ||
            rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num9 ||
            rl.control.current_gamepad == Input::Gamepad::B)
            return "RLFSMStateGetDown";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 ||
            rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num1 ||
            rl.control.current_gamepad == Input::Gamepad::RB_DPadUp ||
            rl.control.current_keyboard == Input::Keyboard::Num5 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
            return "RLFSMStateRLRunning";
        if (mission_complete_)
            return "RLFSMStateRLRoboMimicLocomotion";
        return state_name_;
    }

    void Exit() override
    {
        VisionSprintMode::SetWalk0p5m(false);
        vision_udp_command_.ClearSession();
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        RLFSMStateRunning::Exit();
    }

private:
    using Clock = std::chrono::steady_clock;

    static int ReadIntEnv(
        const char *name, int fallback, int minimum, int maximum)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
            return fallback;
        char *end = nullptr;
        const long value = std::strtol(text, &end, 10);
        if (end == text || *end != '\0')
            return fallback;
        return std::clamp(static_cast<int>(value), minimum, maximum);
    }

    static float ReadFloatEnv(
        const char *name, float fallback, float minimum, float maximum)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
            return fallback;
        char *end = nullptr;
        const float value = std::strtof(text, &end);
        if (end == text || *end != '\0' || !std::isfinite(value))
            return fallback;
        return std::clamp(value, minimum, maximum);
    }

    bool started_ = false;
    bool stop_latched_ = false;
    bool mission_complete_ = false;
    VisionUdpCommandReceiver vision_udp_command_;
    float elapsed_s_ = 0.0f;
    std::string stop_reason_;
    Clock::time_point last_step_at_{};
    Clock::time_point stop_hold_started_at_{};
    Clock::time_point last_heartbeat_at_{};

    void RequestStop(const std::string &reason)
    {
        if (stop_latched_)
            return;
        stop_latched_ = true;
        stop_reason_ = reason;
        vision_udp_command_.ForceZero();
    }
};

class G1RunningFSMFactory : public FSMFactory
{
public:
    explicit G1RunningFSMFactory(const std::string& initial)
        : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(
        void* context, const std::string& state_name) override
    {
        RL* rl = static_cast<RL*>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<g1_fsm::RLFSMStatePassive>(rl);
        if (state_name == "RLFSMStateGetUp")
            return std::make_shared<g1_fsm::RLFSMStateGetUp>(rl);
        if (state_name == "RLFSMStateGetDown")
            return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        if (state_name == "RLFSMStateRLRunning")
            return std::make_shared<RLFSMStateRunning>(rl);
        if (state_name == "RLFSMStateRLRunningStraight110m")
            return std::make_shared<RLFSMStateRunningStraight110m>(rl);
        if (state_name == "RLFSMStateRLRunningVision110m")
            return std::make_shared<RLFSMStateRunningVision110m>(rl);
        if (state_name == "RLFSMStateRLRoboMimicLocomotion")
            return std::make_shared<RLFSMStateLocomotionRunning>(rl);
        if (state_name == "RLFSMStateRLRoboMimicCharleston")
            return std::make_shared<g1_fsm::RLFSMStateRLRoboMimicCharleston>(rl);
        if (state_name == "RLFSMStateRLWholeBodyTrackingDance102")
            return std::make_shared<g1_fsm::RLFSMStateRLWholeBodyTrackingDance102>(rl);
        if (state_name == "RLFSMStateRLWholeBodyTrackingGangnamStyle")
            return std::make_shared<g1_fsm::RLFSMStateRLWholeBodyTrackingGangnamStyle>(rl);
        return nullptr;
    }

    std::string GetType() const override { return "g1"; }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive", "RLFSMStateGetUp", "RLFSMStateGetDown",
            "RLFSMStateRLRunning", "RLFSMStateRLRunningStraight110m",
            "RLFSMStateRLRoboMimicLocomotion",
            "RLFSMStateRLRunningVision110m",
            "RLFSMStateRLRoboMimicCharleston",
            "RLFSMStateRLWholeBodyTrackingDance102",
            "RLFSMStateRLWholeBodyTrackingGangnamStyle"
        };
    }

    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

}  // namespace g1_running_adapter

// Registered after the upstream G1 factory so this additive adapter owns the
// same robot type while preserving all original G1 states.
REGISTER_FSM_FACTORY(g1_running_adapter::G1RunningFSMFactory, "RLFSMStatePassive")

#endif  // G1_RUNNING_ADAPTER_HPP
