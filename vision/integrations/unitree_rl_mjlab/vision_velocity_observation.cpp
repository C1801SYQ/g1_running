// Copy the includes and the code inside namespace isaaclab into
// deploy/robots/g1/src/State_RLBase.cpp.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "isaaclab/envs/mdp/observations/observations.h"

namespace isaaclab
{

// State_RLBase.cpp calls this no-op so the linker keeps this translation unit
// and therefore runs the REGISTER_OBSERVATION static initializers.
void ensure_vision_velocity_observations_linked()
{
}

namespace
{
class VisionVelocityReceiver
{
public:
    VisionVelocityReceiver()
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            return;
        }
        const int flags = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(15001);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~VisionVelocityReceiver()
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    std::array<float, 3> command()
    {
        poll();
        const auto age = std::chrono::steady_clock::now() - last_receive_;
        if (!has_command_ || age > std::chrono::milliseconds(300)) {
            return {0.0f, 0.0f, 0.0f};
        }
        return command_;
    }

private:
    void poll()
    {
        if (fd_ < 0) {
            return;
        }
        char buffer[128];
        while (true) {
            const auto count = ::recv(fd_, buffer, sizeof(buffer) - 1, 0);
            if (count <= 0) {
                break;
            }
            buffer[count] = '\0';
            std::array<float, 3> parsed{};
            if (std::sscanf(
                    buffer, "%f %f %f", &parsed[0], &parsed[1], &parsed[2]
                ) == 3) {
                command_ = parsed;
                last_receive_ = std::chrono::steady_clock::now();
                has_command_ = true;
            }
        }
    }

    int fd_{-1};
    bool has_command_{false};
    std::array<float, 3> command_{0.0f, 0.0f, 0.0f};
    std::chrono::steady_clock::time_point last_receive_{};
};

VisionVelocityReceiver& vision_receiver()
{
    static VisionVelocityReceiver receiver;
    return receiver;
}
}  // namespace

REGISTER_OBSERVATION(vision_velocity_commands)
{
    const auto command = vision_receiver().command();
    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];
    return std::vector<float>{
        std::clamp(
            command[0],
            cfg["lin_vel_x"][0].as<float>(),
            cfg["lin_vel_x"][1].as<float>()
        ),
        std::clamp(
            command[1],
            cfg["lin_vel_y"][0].as<float>(),
            cfg["lin_vel_y"][1].as<float>()
        ),
        std::clamp(
            command[2],
            cfg["ang_vel_z"][0].as<float>(),
            cfg["ang_vel_z"][1].as<float>()
        )
    };
}

REGISTER_OBSERVATION(vision_gait_phase)
{
    const auto command = vision_receiver().command();
    const float period = params["period"].as<float>();
    const float delta_phase = env->step_dt * (1.0f / period);
    env->global_phase = std::fmod(env->global_phase + delta_phase, 1.0f);

    const float command_norm = std::sqrt(
        command[0] * command[0]
        + command[1] * command[1]
        + command[2] * command[2]
    );
    if (command_norm < 0.1f) {
        return std::vector<float>{0.0f, 0.0f};
    }
    constexpr float kPi = 3.14159265358979323846f;
    return std::vector<float>{
        std::sin(env->global_phase * 2.0f * kPi),
        std::cos(env->global_phase * 2.0f * kPi)
    };
}

}  // namespace isaaclab
