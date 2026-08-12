/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef G1_FSM_HPP
#define G1_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include "vision_udp_command.hpp"

#include <cstdlib>

namespace g1_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_getup = 0.0f;

    void Enter() override
    {
        percent_getup = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            const char *auto_running = std::getenv("G1_AUTO_RUNNING");
            if (auto_running != nullptr && std::string(auto_running) == "1")
            {
                return "RLFSMStateRLRunning";
            }
            if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
            {
                return "RLFSMStateRLRunning";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLRoboMimicLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override   // GetUp state
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRLRunning";
        }
        return state_name_;
    }
};

class RLFSMStateRLRoboMimicLocomotion : public RLFSMState
{
public:
RLFSMStateRLRoboMimicLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLRoboMimicLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/locomotion";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            // Reuse an already-loaded locomotion model (e.g. returning from
            // Skill 7) so the control loop is not blocked by a model reload.
            if (!rl.HasLoadedPolicy(robot_config_path))
            {
                rl.InitRL(robot_config_path);
            }
            rl.now_state = *fsm_state;
            // Only notify the Python simulation after the locomotion policy
            // is fully loaded and the robot can actually stand. Notifying any
            // earlier makes the simulation fade the startup tether while the
            // model is still being initialized, so the robot collapses.
            VisionSprintMode::NotifyState1();
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLRoboMimicCharleston";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRLWholeBodyTrackingDance102";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRLWholeBodyTrackingGangnamStyle";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRLRunning";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
        {
            // Skill 6 is intentionally reachable only from state 1. The
            // operator must first complete 0 -> GetUp and then press 1 to
            // enter this stable locomotion/standing state.
            return "RLFSMStateRLVisionSprint100m";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::LB_DPadLeft)
        {
            // Skill 7 is intentionally reachable only from state 1. The
            // operator must first complete 0 -> GetUp and then press 1 to
            // enter this stable locomotion/standing state.
            return "RLFSMStateRLVisionWalk0p5m";
        }
        return state_name_;
    }
};

class RLFSMStateRLRoboMimicCharleston : public RLFSMState
{
public:
    RLFSMStateRLRoboMimicCharleston(RL *rl) : RLFSMState(*rl, "RLFSMStateRLRoboMimicCharleston") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/charleston";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

        rl.motion_length = 18.0;
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        return state_name_;
    }
};

class RLFSMStateRLWholeBodyTrackingDance102 : public RLFSMState
{
public:
    RLFSMStateRLWholeBodyTrackingDance102(RL *rl) : RLFSMState(*rl, "RLFSMStateRLWholeBodyTrackingDance102") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "whole_body_tracking/dance_102";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);

            // Initialize motion loader
            std::string motion_file_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + rl.params.Get<std::string>("motion_file");
            float fps = 1.0f / (rl.params.Get<float>("dt") * rl.params.Get<int>("decimation"));
            rl.motion_loader = std::make_unique<MotionLoader>(motion_file_path, fps);
            rl.motion_length = rl.motion_loader->GetDuration();

            auto waist_sdk_indices = rl.params.Get<std::vector<int>>("waist_joint_indices");
            std::vector<float> waist_angles = {
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[0])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[1])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[2])]
            };
            rl.motion_loader->Reset(fsm_state->imu.quaternion, waist_angles);

            std::cout << LOGGER::INFO << "Motion duration: " << rl.motion_length << "s" << std::endl;

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        // Calculate motion time and progress
        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = std::fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        rl.motion_loader->Update(motion_time);

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

class RLFSMStateRLWholeBodyTrackingGangnamStyle : public RLFSMState
{
public:
RLFSMStateRLWholeBodyTrackingGangnamStyle(RL *rl) : RLFSMState(*rl, "RLFSMStateRLWholeBodyTrackingGangnamStyle") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "whole_body_tracking/gangnam_style";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);

            // Initialize motion loader
            std::string motion_file_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + rl.params.Get<std::string>("motion_file");
            float fps = 1.0f / (rl.params.Get<float>("dt") * rl.params.Get<int>("decimation"));
            rl.motion_loader = std::make_unique<MotionLoader>(motion_file_path, fps);
            rl.motion_length = rl.motion_loader->GetDuration();

            auto waist_sdk_indices = rl.params.Get<std::vector<int>>("waist_joint_indices");
            std::vector<float> waist_angles = {
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[0])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[1])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[2])]
            };
            rl.motion_loader->Reset(fsm_state->imu.quaternion, waist_angles);

            std::cout << LOGGER::INFO << "Motion duration: " << rl.motion_length << "s" << std::endl;

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        // Calculate motion time and progress
        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = std::fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        rl.motion_loader->Update(motion_time);

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        return state_name_;
    }
};

} // namespace g1_fsm

class RLFSMStateRLRunning : public RLFSMState
{
public:
    RLFSMStateRLRunning(RL *rl) : RLFSMState(*rl, "RLFSMStateRLRunning") {}

    float percent_transition = 0.0f;
    float target_distance = 100.0f;
    float run_speed = 5.0f;
    float decel_distance = 30.0f;
    float start_x = 0.0f, start_y = 0.0f;
    bool start_pos_valid = false;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        rl.config_name = "running";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        const char *verbose_running = std::getenv("G1_VERBOSE_RUNNING");
        if (verbose_running != nullptr && std::string(verbose_running) == "1")
        {
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Run x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        }
        RLControl();
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            return "RLFSMStateGetDown";
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            return "RLFSMStateRLRoboMimicLocomotion";
        return state_name_;
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }
};

class RLFSMStateRLVisionSprint100m : public RLFSMState
{
public:
    RLFSMStateRLVisionSprint100m(RL *rl)
        : RLFSMState(*rl, "RLFSMStateRLVisionSprint100m") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        // Skill 6 deliberately reuses GitHub Skill 5's trained running
        // policy. Only the visual command source and race lifecycle differ.
        rl.config_name = "running";
        const std::string robot_config_path =
            rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
            // Announce Skill 6 only after the running model is fully loaded.
            // Python can then use a short camera settling window instead of
            // hiding model-load time behind another fixed multi-second wait.
            VisionSprintMode::SetEnabled(true);
            std::cout << LOGGER::NOTE
                      << "Skill 6 entered: visual 100 m sprint"
                      << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR
                      << "Skill 6 InitRL() failed: " << e.what()
                      << std::endl;
            VisionSprintMode::SetEnabled(false);
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;
        RLControl();
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
        if (rl.control.current_keyboard == Input::Keyboard::Num5 ||
            rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
            return "RLFSMStateRLRunning";
        return state_name_;
    }

    void Exit() override
    {
        VisionSprintMode::SetEnabled(false);
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        rl.rl_init_done = false;
    }
};

class RLFSMStateRLVisionWalk0p5m : public RLFSMState
{
public:
    RLFSMStateRLVisionWalk0p5m(RL *rl)
        : RLFSMState(*rl, "RLFSMStateRLVisionWalk0p5m") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;

        // Complete per-mission reset: every field must be re-initialised so a
        // second Num7 mission starts fresh (distance 0, time 0, no inherited
        // hard_stop / timeout / mission_complete / settle state).
        elapsed_s_ = 0.0f;
        estimated_distance_m_ = 0.0f;
        started_ = false;
        stop_latched_ = false;
        mission_complete_ = false;
        stop_reason_.clear();
        last_step_time_ = std::chrono::steady_clock::now();
        mission_started_at_ = std::chrono::steady_clock::now();
        stop_hold_started_at_ = std::chrono::steady_clock::time_point{};
        last_heartbeat_at_ = std::chrono::steady_clock::time_point{};
        target_m_ = 0.0f;
        stop_margin_m_ = 0.0f;
        distance_scale_ = 0.0f;
        max_duration_s_ = 0.0f;
        start_timeout_s_ = 0.0f;
        settle_s_ = 0.0f;

        // Num7 is only reachable from the stable locomotion state (Num1), so
        // the locomotion policy MUST already be loaded. We deliberately do
        // NOT load it here: model loading must never happen inside the Num7
        // Enter / control cycle because it would block the loop. If the
        // locomotion policy is not loaded, reject Num7, stay at zero velocity
        // and fall back safely to Passive.
        rl.config_name = "robomimic/locomotion";
        const std::string robot_config_path =
            rl.robot_name + "/" + rl.config_name;
        const auto entry_decision = vision_walk_entry::Evaluate(
            rl.HasLoadedPolicy(robot_config_path),
            rl.vision_udp_command.IsReady());
        if (entry_decision == vision_walk_entry::Decision::POLICY_MISSING)
        {
            std::cout << LOGGER::ERROR
                      << "Skill 7 requires the locomotion policy "
                      << robot_config_path
                      << " to already be loaded; rejecting Num7 and "
                         "switching to Passive"
                      << std::endl;
            VisionSprintMode::SetWalk0p5m(false);
            rl.control.x = 0.0f;
            rl.control.y = 0.0f;
            rl.control.yaw = 0.0f;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        if (entry_decision ==
            vision_walk_entry::Decision::RECEIVER_UNAVAILABLE)
        {
            std::cout << LOGGER::ERROR
                      << "Skill 7 command receiver is unavailable; rejecting "
                         "Num7 and switching to Passive"
                      << std::endl;
            VisionSprintMode::SetWalk0p5m(false);
            rl.control.x = 0.0f;
            rl.control.y = 0.0f;
            rl.control.yaw = 0.0f;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        try
        {
            rl.now_state = *fsm_state;
            // Activate the Num7 vision mode and flush any stale UDP backlog.
            VisionSprintMode::SetWalk0p5m(true);
            rl.vision_udp_command.ClearForceZero();
            rl.vision_udp_command.ClearSession();
            std::cout << LOGGER::NOTE
                      << "Skill 7 entered: 1.0 m vision walk (locomotion "
                         "policy, vx<=0.50, |wz|<=0.25)"
                      << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR
                      << "Skill 7 entry failed: " << e.what()
                      << std::endl;
            VisionSprintMode::SetWalk0p5m(false);
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done)
        {
            rl.rl_init_done = true;
        }
        // Periodic enable heartbeat so a late-starting Python listener can
        // discover that Num7 is active even if it missed the transition
        // datagram (the single notification is not reliable).
        const auto now_run = std::chrono::steady_clock::now();
        if (now_run - last_heartbeat_at_ >= std::chrono::milliseconds(500))
        {
            VisionSprintMode::Heartbeat();
            last_heartbeat_at_ = now_run;
        }
        StepMission();
        RLControl();
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
            rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            return "RLFSMStateRLRoboMimicLocomotion";
        if (stop_latched_ && mission_complete_)
        {
            // Automatic return to the stable locomotion state after the
            // mission has finished and the zero-velocity hold elapsed.
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        return state_name_;
    }

    void Exit() override
    {
        VisionSprintMode::SetWalk0p5m(false);
        rl.vision_udp_command.ClearForceZero();
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        rl.rl_init_done = false;
    }

private:
    float target_m_ = 0.0f;
    float stop_margin_m_ = 0.0f;
    float distance_scale_ = 0.0f;
    float max_duration_s_ = 0.0f;
    float start_timeout_s_ = 0.0f;
    float settle_s_ = 0.0f;

    float elapsed_s_ = 0.0f;
    float estimated_distance_m_ = 0.0f;
    bool started_ = false;
    bool stop_latched_ = false;
    bool mission_complete_ = false;
    std::string stop_reason_;
    std::chrono::steady_clock::time_point last_step_time_;
    std::chrono::steady_clock::time_point mission_started_at_;
    std::chrono::steady_clock::time_point stop_hold_started_at_;
    std::chrono::steady_clock::time_point last_heartbeat_at_;

    static float ReadFloatEnv(const char *name, float fallback, float lo, float hi)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
        {
            return fallback;
        }
        char *end = nullptr;
        const float value = std::strtof(text, &end);
        if (end == text || *end != '\0')
        {
            return fallback;
        }
        return std::clamp(value, lo, hi);
    }

    float ReadParam(const char *name, float fallback, float lo, float hi)
    {
        (void)name;
        return ReadFloatEnv(name, fallback, lo, hi);
    }

    void StepMission()
    {
        using namespace std::chrono;
        const steady_clock::time_point now = steady_clock::now();
        const float dt = std::max(
            1e-3f,
            duration_cast<duration<float>>(now - last_step_time_).count());
        last_step_time_ = now;

        // Read tunables once per state entry (cheap; cached in locals).
        if (target_m_ == 0.0f)
        {
            target_m_ = ReadParam("G1_NUM7_TARGET_M", 1.00f, 0.05f, 1.00f);
            stop_margin_m_ = ReadParam(
                "G1_NUM7_STOP_MARGIN_M", 0.0f, 0.0f, 0.20f);
            // Open-loop distance calibration, not odometry.  MuJoCo with the
            // deployed policy advances about 0.8 m per 1.0 m of integrated
            // velocity command at 0.50 m/s. Keep it explicit and identical
            // in C++ and Python so the intended 1 m stop can be audited.
            distance_scale_ = ReadParam(
                "G1_NUM7_DISTANCE_SCALE", 0.80f, 0.10f, 2.0f);
            max_duration_s_ = ReadParam(
                "G1_NUM7_MAX_DURATION_S", 7.0f, 1.0f, 60.0f);
            start_timeout_s_ = ReadParam(
                "G1_NUM7_START_TIMEOUT_S", 2.0f, 0.5f, 10.0f);
            settle_s_ = ReadParam("G1_NUM7_SETTLE_S", 0.5f, 0.1f, 5.0f);
        }

        elapsed_s_ += dt;

        // Hard safety caps that environment variables cannot override.
        constexpr float kMaxVx = 0.50f;
        constexpr float kMaxWz = 0.25f;
        const float applied_vx = std::clamp(rl.control.x, 0.0f, kMaxVx);
        rl.control.y = 0.0f;
        const float applied_wz = std::clamp(
            rl.control.yaw, -kMaxWz, kMaxWz);
        rl.control.x = applied_vx;
        rl.control.yaw = applied_wz;

        if (!stop_latched_)
        {
            // Start timeout: require the first fresh, safe motion command
            // within start_timeout_s of entering the state.
            if (!started_)
            {
                if (rl.vision_udp_command.HasFreshCommand())
                {
                    if (applied_vx > 0.0f)
                    {
                        started_ = true;
                    }
                    else if (elapsed_s_ > start_timeout_s_)
                    {
                        RequestStop("START_TIMEOUT_NO_FORWARD_CMD");
                    }
                }
                else if (elapsed_s_ > start_timeout_s_)
                {
                    RequestStop("START_TIMEOUT_NO_CMD");
                }
            }

            if (!stop_latched_ && started_)
            {
                // Integrate the applied (clamped) safe vx, NOT the raw UDP
                // request. This is an estimate, not a measured odometry value.
            estimated_distance_m_ += applied_vx * dt * distance_scale_;

                // Freshness: UDP must be newer than 300 ms.
                if (rl.vision_udp_command.LastReceiveAgeMs() > 300)
                {
                    RequestStop("UDP_STALE");
                }
                if (rl.vision_udp_command.HardStopRequested())
                {
                    RequestStop("HARD_STOP");
                }
                if (rl.control.x <= 0.0f)
                {
                    // Visual loss / obstacle in Python is reported as zero vx
                    // or hard_stop; latch a stop if the command stays zero for
                    // a short window.
                    RequestStop("VISION_ZERO_CMD");
                }
                if (estimated_distance_m_ >=
                    std::max(0.0f, target_m_ - stop_margin_m_))
                {
                    RequestStop("DISTANCE_REACHED");
                }
            }

            if (!stop_latched_ && elapsed_s_ >= max_duration_s_)
            {
                RequestStop("MAX_DURATION");
            }
        }

        if (stop_latched_)
        {
            // Lock zero command and hold for the settle duration.
            rl.control.x = 0.0f;
            rl.control.y = 0.0f;
            rl.control.yaw = 0.0f;
            rl.vision_udp_command.ForceZero();
            if (stop_hold_started_at_ == steady_clock::time_point{})
            {
                stop_hold_started_at_ = now;
                std::cout << "[skill7] stop latched: " << stop_reason_
                          << " estimated_commanded_distance="
                          << estimated_distance_m_ << " m" << std::endl;
            }
            if (duration_cast<duration<float>>(now - stop_hold_started_at_)
                    .count()
                >= settle_s_)
            {
                mission_complete_ = true;
            }
        }
    }

    void RequestStop(const std::string &reason)
    {
        if (stop_latched_)
        {
            return;
        }
        stop_latched_ = true;
        stop_reason_ = reason;
        rl.vision_udp_command.ForceZero();
    }
};

class G1FSMFactory : public FSMFactory
{
public:
    G1FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<g1_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<g1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLRoboMimicLocomotion")
            return std::make_shared<g1_fsm::RLFSMStateRLRoboMimicLocomotion>(rl);
        else if (state_name == "RLFSMStateRLRoboMimicCharleston")
            return std::make_shared<g1_fsm::RLFSMStateRLRoboMimicCharleston>(rl);
        else if (state_name == "RLFSMStateRLWholeBodyTrackingDance102")
            return std::make_shared<g1_fsm::RLFSMStateRLWholeBodyTrackingDance102>(rl);
        else if (state_name == "RLFSMStateRLWholeBodyTrackingGangnamStyle")
            return std::make_shared<g1_fsm::RLFSMStateRLWholeBodyTrackingGangnamStyle>(rl);
        else if (state_name == "RLFSMStateRLRunning")
            return std::make_shared<RLFSMStateRLRunning>(rl);
        else if (state_name == "RLFSMStateRLVisionSprint100m")
            return std::make_shared<RLFSMStateRLVisionSprint100m>(rl);
        else if (state_name == "RLFSMStateRLVisionWalk0p5m")
            return std::make_shared<RLFSMStateRLVisionWalk0p5m>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLRoboMimicLocomotion",
            "RLFSMStateRLRoboMimicCharleston",
            "RLFSMStateRLWholeBodyTrackingDance102",
            "RLFSMStateRLWholeBodyTrackingGangnamStyle",
            "RLFSMStateRLRunning",
            "RLFSMStateRLVisionSprint100m",
            "RLFSMStateRLVisionWalk0p5m"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(G1FSMFactory, "RLFSMStatePassive")

#endif // G1_FSM_HPP
