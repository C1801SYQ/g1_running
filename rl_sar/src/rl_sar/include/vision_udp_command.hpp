/*
 * Vision velocity adapter for C1801SYQ/g1_running.
 *
 * Receives ASCII UDP datagrams in the form:
 *     vx vy wz
 * or the extended form:
 *     vx vy wz hard_stop
 *
 * The socket only listens on 127.0.0.1. Once the first valid command has
 * arrived, a timeout produces a zero command instead of falling back to the
 * gamepad. This prevents a camera or perception crash from leaving the robot
 * running on the last command.
 *
 * The vision command mode is generalised from a single bool into
 * NONE / SPRINT100M / WALK0P5M so Skill 6 (100 m sprint) and Skill 7
 * (short-distance vision walk; legacy wire name WALK0P5M) share one socket
 * while keeping their state,
 * generation, freshness and hard-stop handling isolated.
 */

#ifndef G1_RACE_VISION_UDP_COMMAND_HPP
#define G1_RACE_VISION_UDP_COMMAND_HPP

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace vision_mode
{
enum class Mode
{
    NONE = 0,
    SPRINT100M = 1,
    WALK0P5M = 2,
};

inline const char *ToString(Mode mode)
{
    switch (mode)
    {
        case Mode::SPRINT100M:
            return "SPRINT100M";
        case Mode::WALK0P5M:
            return "WALK0P5M";
        default:
            return "NONE";
    }
}
}  // namespace vision_mode

// Pure, executable entry guard shared by the Num7 FSM and its standalone
// tests.  Keeping the decision independent of the FSM object graph lets us
// prove that a missing policy or an unavailable command socket can never arm
// the vision walk state.
namespace vision_walk_entry
{
enum class Decision
{
    READY,
    POLICY_MISSING,
    RECEIVER_UNAVAILABLE,
};

constexpr Decision Evaluate(bool has_locomotion_policy, bool receiver_ready)
{
    if (!has_locomotion_policy)
    {
        return Decision::POLICY_MISSING;
    }
    if (!receiver_ready)
    {
        return Decision::RECEIVER_UNAVAILABLE;
    }
    return Decision::READY;
}
}  // namespace vision_walk_entry

class VisionSprintMode
{
public:
    static void SetMode(vision_mode::Mode mode)
    {
        const vision_mode::Mode previous = ModeFlag().exchange(
            mode, std::memory_order_acq_rel);
        if (previous == mode)
        {
            return;
        }
        GenerationCounter().fetch_add(1, std::memory_order_acq_rel);
        NotifyMode(mode);
        std::cout << "[vision] command mode -> "
                  << vision_mode::ToString(mode) << std::endl;
    }

    // Backwards-compatible Skill 6 API.
    static void SetEnabled(bool enabled)
    {
        SetMode(
            enabled ? vision_mode::Mode::SPRINT100M
                    : vision_mode::Mode::NONE);
    }

    static void SetWalk0p5m(bool enabled)
    {
        SetMode(
            enabled ? vision_mode::Mode::WALK0P5M
                    : vision_mode::Mode::NONE);
    }

    static vision_mode::Mode CurrentMode()
    {
        return ModeFlag().load(std::memory_order_acquire);
    }

    static bool IsEnabled()
    {
        return CurrentMode() == vision_mode::Mode::SPRINT100M;
    }

    static bool IsWalk0p5m()
    {
        return CurrentMode() == vision_mode::Mode::WALK0P5M;
    }

    // Re-send the current mode's enable payload so a late-starting Python
    // listener (which may have missed the single transition datagram) can
    // still learn that the mode is active. Idempotent: does not bump the
    // generation counter.
    static void Heartbeat()
    {
        const vision_mode::Mode mode = CurrentMode();
        if (mode == vision_mode::Mode::NONE)
        {
            return;
        }
        NotifyMode(mode);
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
        std::cout << "[vision] state 1 entered; releasing startup support"
                  << std::endl;
    }

    static std::uint64_t Generation()
    {
        return GenerationCounter().load(std::memory_order_acquire);
    }

private:
    static std::atomic<vision_mode::Mode> &ModeFlag()
    {
        static std::atomic<vision_mode::Mode> flag{
            vision_mode::Mode::NONE};
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

    static void NotifyMode(vision_mode::Mode mode)
    {
        const char *payload = nullptr;
        switch (mode)
        {
            case vision_mode::Mode::SPRINT100M:
                payload = "G1_VISION_SPRINT 1\n";
                break;
            case vision_mode::Mode::WALK0P5M:
                payload = "G1_VISION_WALK_0P5M 1\n";
                break;
            default:
                // Unknown disable; send both disable payloads so any listener
                // observing either protocol is correctly told the mode is off.
                SendPayload("G1_VISION_SPRINT 0\n");
                SendPayload("G1_VISION_WALK_0P5M 0\n");
                return;
        }
        SendPayload(payload);
    }

    static void SendPayload(const char *payload)
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
    // Num7 hard limits that environment variables must not override.
    // The deployed robomimic locomotion policy was trained for walking
    // commands up to 1.0 m/s.  Num7 uses 0.50 m/s: inside that distribution,
    // but still conservatively below the policy maximum.
    static constexpr float kNum7MaxVx = 0.50f;
    static constexpr float kNum7MaxWz = 0.25f;

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

    // Applies the vision command on top of gamepad values. Returns true when
    // a vision mode is active (so the caller must keep the zeroed output),
    // false when no vision mode is active (gamepad passes through).
    bool Apply(float &vx, float &vy, float &wz)
    {
        const vision_mode::Mode mode = VisionSprintMode::CurrentMode();
        if (mode == vision_mode::Mode::NONE)
        {
            // Drain and discard stale packets while the mode is off so a
            // backlog can never be mistaken for fresh data after activation.
            DrainStale(32);
            return false;
        }

        // A latched stop from a previous cycle keeps overriding any newer
        // non-zero UDP command until the FSM clears it on state exit.
        if (force_zero_)
        {
            vx = 0.0f;
            vy = 0.0f;
            wz = 0.0f;
            return true;
        }

        const std::uint64_t generation = VisionSprintMode::Generation();
        if (generation != mode_generation_)
        {
            // Mode (re)activation: flush the queue so only packets that
            // arrive after activation can ever move the robot.
            DrainStale(32);
            received_once_ = false;
            timeout_reported_ = false;
            hard_stop_ = false;
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

        if (hard_stop_)
        {
            vx = 0.0f;
            vy = 0.0f;
            wz = 0.0f;
            if (!hard_stop_reported_)
            {
                std::cout
                    << "[vision] hard stop latched; forcing zero velocity"
                    << std::endl;
                hard_stop_reported_ = true;
            }
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

        float effective_max_vx = max_vx_;
        float effective_max_wz = max_wz_;
        if (mode == vision_mode::Mode::WALK0P5M)
        {
            // Num7 hard safety caps: these are absolute and cannot be
            // enlarged by environment variables.
            effective_max_vx = std::min(max_vx_, kNum7MaxVx);
            effective_max_wz = std::min(max_wz_, kNum7MaxWz);
        }

        vx = std::clamp(command_vx_, 0.0f, effective_max_vx);
        // The supplied policies were trained with zero lateral velocity.
        // Steering is therefore done with yaw only.
        vy = 0.0f;
        wz = std::clamp(command_wz_, -effective_max_wz, effective_max_wz);
        return true;
    }

    // True once a fresh, non-zero-positive command has been received in the
    // current mode generation (used for the Num7 start timeout).
    bool HasFreshCommand() const
    {
        return received_once_ && !force_zero_;
    }

    // True while a Python-reported hard stop is latched.
    bool HardStopRequested() const
    {
        return hard_stop_;
    }

    // A vision-controlled FSM state must never arm when the command socket
    // failed to bind.  Waiting for a start timeout is fail-safe for motion,
    // but it hides configuration/port conflicts and makes an integration test
    // look like a valid zero-command mission.
    bool IsReady() const
    {
        return socket_fd_ >= 0;
    }

    // Milliseconds since the last valid packet (used for Num7 freshness).
    long LastReceiveAgeMs() const
    {
        if (!received_once_)
        {
            return std::numeric_limits<long>::max();
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   Clock::now() - last_receive_time_)
            .count();
    }

    // Latch zero output until ClearForceZero() is called (state exit).
    void ForceZero()
    {
        force_zero_ = true;
    }

    void ClearForceZero()
    {
        force_zero_ = false;
        received_once_ = false;
        timeout_reported_ = false;
        hard_stop_ = false;
        hard_stop_reported_ = false;
    }

    // Full session reset: clears force_zero, freshness, hard_stop AND resets
    // the tracked mode generation so the next mode activation is treated as a
    // brand-new session (no inherited packets, no inherited stop).
    void ClearSession()
    {
        force_zero_ = false;
        received_once_ = false;
        timeout_reported_ = false;
        hard_stop_ = false;
        hard_stop_reported_ = false;
        command_vx_ = 0.0f;
        command_wz_ = 0.0f;
        mode_generation_ = 0;
        last_receive_time_ = Clock::time_point{};
    }

    // Test-only hook so the strict parser can be unit tested without
    // opening a socket.
    static bool ParsePacketForTest(
        const char *text,
        float &vx,
        float &vy,
        float &wz,
        int &hard_stop)
    {
        return ParsePacket(text, vx, vy, wz, hard_stop);
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

    // Strictly parse "vx vy wz" or "vx vy wz hard_stop". Rejects NaN/Inf,
    // trailing junk and out-of-range hard_stop tokens. Returns false without
    // touching the watchdog when the packet is invalid.
    static bool ParsePacket(
        const char *text,
        float &out_vx,
        float &out_vy,
        float &out_wz,
        int &out_hard_stop)
    {
        out_vx = 0.0f;
        out_vy = 0.0f;
        out_wz = 0.0f;
        out_hard_stop = 0;

        char buffer[256];
        const size_t length = std::strlen(text);
        if (length == 0 || length >= sizeof(buffer))
        {
            return false;
        }
        std::memcpy(buffer, text, length + 1);

        float values[3] = {0.0f, 0.0f, 0.0f};
        char *cursor = buffer;
        for (int i = 0; i < 3; ++i)
        {
            char *end = nullptr;
            errno = 0;
            const float value = std::strtof(cursor, &end);
            if (end == cursor || errno == ERANGE)
            {
                return false;
            }
            if (!std::isfinite(value))
            {
                return false;
            }
            values[i] = value;
            cursor = end;
        }

        // Optional hard_stop token: 0 or 1 only. ASCII whitespace (space,
        // tab, \n, \r) may terminate a packet, so the Python sender's
        // "vx vy wz hard_stop\n" payload is accepted verbatim.
        char *tail = cursor;
        while (*tail != '\0' && std::isspace(
                   static_cast<unsigned char>(*tail)))
        {
            ++tail;
        }
        if (*tail != '\0')
        {
            char *end = nullptr;
            errno = 0;
            const long stop = std::strtol(tail, &end, 10);
            if (end == tail || errno == ERANGE || (stop != 0 && stop != 1))
            {
                return false;
            }
            char *junk = end;
            while (*junk != '\0' && std::isspace(
                       static_cast<unsigned char>(*junk)))
            {
                ++junk;
            }
            if (*junk != '\0')
            {
                // Trailing garbage after hard_stop is rejected.
                return false;
            }
            out_hard_stop = static_cast<int>(stop);
        }
        else if (*tail == '\0')
        {
            // Three floats only: accepted, hard_stop defaults to 0.
        }

        out_vx = values[0];
        out_vy = values[1];
        out_wz = values[2];
        return true;
    }

    // Read up to `max_packets` datagrams and drop them without touching any
    // command state. Used while a mode is off and on activation to flush the
    // backlog.
    void DrainStale(int max_packets)
    {
        if (socket_fd_ < 0)
        {
            return;
        }
        char buffer[256];
        for (int i = 0; i < max_packets; ++i)
        {
            const ssize_t length =
                ::recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
            if (length < 0)
            {
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                    Warn("UDP receive failed during drain");
                }
                return;
            }
            // Drop.
        }
    }

    void Poll()
    {
        if (socket_fd_ < 0)
        {
            return;
        }

        char buffer[256];
        for (int packets = 0; packets < 32; ++packets)
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
            int received_hard_stop = 0;
            if (!ParsePacket(
                    buffer,
                    received_vx,
                    received_vy,
                    received_wz,
                    received_hard_stop))
            {
                // Invalid packet: does not refresh the watchdog.
                continue;
            }

            command_vx_ = received_vx;
            command_wz_ = received_wz;
            if (received_hard_stop != 0)
            {
                hard_stop_ = true;
            }
            last_receive_time_ = Clock::now();
            timeout_reported_ = false;
            hard_stop_reported_ = false;
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
    bool hard_stop_ = false;
    bool hard_stop_reported_ = false;
    bool force_zero_ = false;
    std::uint64_t mode_generation_ = 0;
    Clock::time_point last_receive_time_{};
};

#endif  // G1_RACE_VISION_UDP_COMMAND_HPP
