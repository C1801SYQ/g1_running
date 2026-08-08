/*
 * Vision velocity adapter for C1801SYQ/g1_running.
 *
 * Receives ASCII UDP datagrams in the form:
 *     vx vy wz
 *
 * The socket only listens on 127.0.0.1. Once the first valid command has
 * arrived, a timeout produces a zero command instead of falling back to the
 * gamepad. This prevents a camera or perception crash from leaving the robot
 * running on the last command.
 */

#ifndef G1_RACE_VISION_UDP_COMMAND_HPP
#define G1_RACE_VISION_UDP_COMMAND_HPP

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

class VisionSprintMode
{
public:
    static void SetEnabled(bool enabled)
    {
        const bool was_enabled = EnabledFlag().exchange(
            enabled, std::memory_order_acq_rel);
        if (enabled && !was_enabled)
        {
            GenerationCounter().fetch_add(1, std::memory_order_acq_rel);
            NotifyState(true);
            std::cout
                << "[skill6] visual 100 m sprint command mode enabled"
                << std::endl;
        }
        else if (!enabled && was_enabled)
        {
            NotifyState(false);
            std::cout << "[skill6] visual sprint command mode disabled"
                      << std::endl;
        }
    }

    static bool IsEnabled()
    {
        return EnabledFlag().load(std::memory_order_acquire);
    }

    static void NotifyState1()
    {
        const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0)
        {
            return;
        }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<uint16_t>(StatusPort()));
        const char *payload = "G1_VISION_STATE1 1\n";
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            ::sendto(
                socket_fd,
                payload,
                std::strlen(payload),
                0,
                reinterpret_cast<const sockaddr *>(&address),
                sizeof(address));
        }
        ::close(socket_fd);
        std::cout << "[skill6] state 1 entered; releasing startup support"
                  << std::endl;
    }

    static std::uint64_t Generation()
    {
        return GenerationCounter().load(std::memory_order_acquire);
    }

private:
    static std::atomic<bool> &EnabledFlag()
    {
        static std::atomic<bool> flag{false};
        return flag;
    }

    static std::atomic<std::uint64_t> &GenerationCounter()
    {
        static std::atomic<std::uint64_t> generation{0};
        return generation;
    }

    static int StatusPort()
    {
        constexpr int default_port = 15002;
        const char *text = std::getenv("G1_VISION_STATUS_PORT");
        if (text == nullptr)
        {
            return default_port;
        }
        char *end = nullptr;
        const long value = std::strtol(text, &end, 10);
        if (end == text || *end != '\0' || value < 1 || value > 65535)
        {
            return default_port;
        }
        return static_cast<int>(value);
    }

    static void NotifyState(bool enabled)
    {
        const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0)
        {
            return;
        }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<uint16_t>(StatusPort()));
        const char *payload = enabled
            ? "G1_VISION_SPRINT 1\n"
            : "G1_VISION_SPRINT 0\n";
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            ::sendto(
                socket_fd,
                payload,
                std::strlen(payload),
                0,
                reinterpret_cast<const sockaddr *>(&address),
                sizeof(address));
        }
        ::close(socket_fd);
    }

};

class VisionUdpCommandReceiver
{
public:
    VisionUdpCommandReceiver()
    {
        port_ = ReadIntEnvironment("G1_VISION_UDP_PORT", 15001, 1, 65535);
        timeout_ms_ = ReadIntEnvironment(
            "G1_VISION_TIMEOUT_MS", 300, 50, 5000);
        max_vx_ = ReadFloatEnvironment(
            "G1_VISION_MAX_VX", 1.0f, 0.05f, 5.1f);
        max_wz_ = ReadFloatEnvironment(
            "G1_VISION_MAX_WZ", 0.5f, 0.05f, 1.5f);

        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0)
        {
            Warn("could not create UDP socket");
            return;
        }

        const int current_flags = ::fcntl(socket_fd_, F_GETFL, 0);
        if (current_flags < 0 ||
            ::fcntl(socket_fd_, F_SETFL, current_flags | O_NONBLOCK) < 0)
        {
            Warn("could not make UDP socket non-blocking");
            Close();
            return;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<uint16_t>(port_));
        if (::bind(
                socket_fd_,
                reinterpret_cast<const sockaddr *>(&address),
                sizeof(address)) < 0)
        {
            Warn("could not bind 127.0.0.1:" + std::to_string(port_));
            Close();
            return;
        }

        std::cout
            << "[vision] velocity receiver listening on 127.0.0.1:"
            << port_ << ", timeout=" << timeout_ms_
            << " ms, max_vx=" << max_vx_
            << ", max_wz=" << max_wz_ << std::endl;
    }

    ~VisionUdpCommandReceiver()
    {
        Close();
    }

    VisionUdpCommandReceiver(const VisionUdpCommandReceiver &) = delete;
    VisionUdpCommandReceiver &operator=(
        const VisionUdpCommandReceiver &) = delete;

    bool Apply(float &vx, float &vy, float &wz)
    {
        if (!VisionSprintMode::IsEnabled())
        {
            return false;
        }

        const std::uint64_t generation = VisionSprintMode::Generation();
        if (generation != mode_generation_)
        {
            received_once_ = false;
            timeout_reported_ = false;
            mode_generation_ = generation;
        }

        Poll();
        if (!received_once_)
        {
            vx = 0.0f;
            vy = 0.0f;
            wz = 0.0f;
            return true;
        }

        const auto age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - last_receive_time_)
                .count();
        if (age_ms > timeout_ms_)
        {
            vx = 0.0f;
            vy = 0.0f;
            wz = 0.0f;
            if (!timeout_reported_)
            {
                std::cout
                    << "[vision] command timeout; forcing zero velocity"
                    << std::endl;
                timeout_reported_ = true;
            }
            return true;
        }

        vx = command_vx_;
        // The supplied speed_turn policy was trained with zero lateral
        // velocity. Steering is therefore done with yaw only.
        vy = 0.0f;
        wz = command_wz_;
        return true;
    }

private:
    using Clock = std::chrono::steady_clock;

    static int ReadIntEnvironment(
        const char *name, int fallback, int minimum, int maximum)
    {
        const char *text = std::getenv(name);
        if (text == nullptr)
        {
            return fallback;
        }
        char *end = nullptr;
        const long value = std::strtol(text, &end, 10);
        if (end == text || *end != '\0')
        {
            return fallback;
        }
        return std::clamp(static_cast<int>(value), minimum, maximum);
    }

    static float ReadFloatEnvironment(
        const char *name, float fallback, float minimum, float maximum)
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
        return std::clamp(value, minimum, maximum);
    }

    void Poll()
    {
        if (socket_fd_ < 0)
        {
            return;
        }

        char buffer[256];
        while (true)
        {
            const ssize_t length =
                ::recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
            if (length < 0)
            {
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                    Warn("UDP receive failed");
                }
                return;
            }
            if (length == 0)
            {
                return;
            }

            buffer[length] = '\0';
            float received_vx = 0.0f;
            float received_vy = 0.0f;
            float received_wz = 0.0f;
            if (std::sscanf(
                    buffer,
                    "%f %f %f",
                    &received_vx,
                    &received_vy,
                    &received_wz) != 3)
            {
                continue;
            }

            command_vx_ = std::clamp(received_vx, 0.0f, max_vx_);
            command_wz_ = std::clamp(received_wz, -max_wz_, max_wz_);
            last_receive_time_ = Clock::now();
            timeout_reported_ = false;
            if (!received_once_)
            {
                std::cout
                    << "[vision] first velocity command received; "
                    << "vision override is active" << std::endl;
            }
            received_once_ = true;
        }
    }

    void Close()
    {
        if (socket_fd_ >= 0)
        {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    static void Warn(const std::string &message)
    {
        std::cerr
            << "[vision] warning: " << message << ": "
            << std::strerror(errno) << std::endl;
    }

    int socket_fd_ = -1;
    int port_ = 15001;
    int timeout_ms_ = 300;
    float max_vx_ = 1.0f;
    float max_wz_ = 0.5f;
    float command_vx_ = 0.0f;
    float command_wz_ = 0.0f;
    bool received_once_ = false;
    bool timeout_reported_ = false;
    std::uint64_t mode_generation_ = 0;
    Clock::time_point last_receive_time_{};
};

#endif  // G1_RACE_VISION_UDP_COMMAND_HPP
