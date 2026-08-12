// Standalone unit test for the Skill 7 vision UDP layer.
//
// Build (from the rl_sar source root):
//   g++ -std=c++17 -Wall -Wextra
//       -I. -Isrc/rl_sar/include -Isrc/rl_sar/fsm_robot
//       test/test_vision_udp_num7.cpp -o /tmp/test_vision_udp_num7
//   /tmp/test_vision_udp_num7
//
// This test never opens a robot controller and never moves a robot.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "vision_udp_command.hpp"

namespace
{
int g_failures = 0;

#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)

// ---- Parser tests --------------------------------------------------------

void TestStrictParseValidThreeFloats()
{
    float vx, vy, wz;
    int stop;
    bool ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(std::fabs(vx - 0.05f) < 1e-5f);
    CHECK(std::fabs(vy) < 1e-6f);
    CHECK(std::fabs(wz - 0.01f) < 1e-5f);
    CHECK(stop == 0);
}

void TestStrictParseValidWithHardStop()
{
    float vx, vy, wz;
    int stop;
    bool ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000 1", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 1);

    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000 0", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 0);
}

// Real Python UdpCommandSender payloads always end in "\n":
//   f"{vx:.6f} {vy:.6f} {wz:.6f} {1|0}\n"
void TestPythonSenderPayloadsWithNewline()
{
    float vx, vy, wz;
    int stop;

    // Four fields with "\n".
    bool ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000 0\n", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(std::fabs(vx - 0.05f) < 1e-5f);
    CHECK(stop == 0);

    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000 1\n", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 1);

    // Three fields with "\n" (legacy format).
    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000\n", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 0);

    // Four fields with "\r\n".
    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.050000 0.000000 0.010000 1\r\n", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 1);

    // Trailing whitespace combinations (spaces / tab / newline).
    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.01 1 \t\n", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 1);
    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.01 0 \r\n ", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 0);
    ok = VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.01\t1\n", vx, vy, wz, stop);
    CHECK(ok);
    CHECK(stop == 1);
}

void TestStrictParseRejectsNan()
{
    float vx, vy, wz;
    int stop;
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "nan 0.0 0.0", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 inf 0.0", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 -inf", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 nan", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 inf 1", vx, vy, wz, stop));
}

void TestStrictParseRejectsTrailingJunk()
{
    float vx, vy, wz;
    int stop;
    // Newline followed by more content must be rejected.
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 1\ngarbage", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 1\n0", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 garbage", vx, vy, wz, stop));
    // hard_stop out of range.
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 2", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 -1", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 1 x", vx, vy, wz, stop));
    // Too few fields.
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05", vx, vy, wz, stop));
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0", vx, vy, wz, stop));
    // Too many fields.
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "0.05 0.0 0.0 1 2", vx, vy, wz, stop));
    // Empty.
    CHECK(!VisionUdpCommandReceiver::ParsePacketForTest(
        "", vx, vy, wz, stop));
}

// ---- Mode lifecycle ------------------------------------------------------

void TestModeLifecycle()
{
    VisionSprintMode::SetMode(vision_mode::Mode::NONE);
    CHECK(VisionSprintMode::CurrentMode() == vision_mode::Mode::NONE);
    const std::uint64_t g0 = VisionSprintMode::Generation();

    VisionSprintMode::SetWalk0p5m(true);
    CHECK(VisionSprintMode::IsWalk0p5m());
    CHECK(VisionSprintMode::CurrentMode() == vision_mode::Mode::WALK0P5M);
    CHECK(VisionSprintMode::Generation() > g0);

    VisionSprintMode::SetWalk0p5m(false);
    CHECK(VisionSprintMode::CurrentMode() == vision_mode::Mode::NONE);

    VisionSprintMode::SetEnabled(true);
    CHECK(VisionSprintMode::IsEnabled());
    CHECK(VisionSprintMode::CurrentMode() == vision_mode::Mode::SPRINT100M);
    VisionSprintMode::SetEnabled(false);
    CHECK(VisionSprintMode::CurrentMode() == vision_mode::Mode::NONE);
}

void TestNum7EntryDecision()
{
    using vision_walk_entry::Decision;
    CHECK(vision_walk_entry::Evaluate(true, true) == Decision::READY);
    CHECK(
        vision_walk_entry::Evaluate(false, true) ==
        Decision::POLICY_MISSING);
    CHECK(
        vision_walk_entry::Evaluate(true, false) ==
        Decision::RECEIVER_UNAVAILABLE);
    // Policy absence takes precedence, so an invalid state can never be
    // misreported as a mere socket issue or accidentally armed.
    CHECK(
        vision_walk_entry::Evaluate(false, false) ==
        Decision::POLICY_MISSING);
}

int FindFreeUdpPort()
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(fd >= 0);
    if (fd < 0)
    {
        return -1;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    CHECK(::bind(
        fd,
        reinterpret_cast<const sockaddr *>(&address),
        sizeof(address)) == 0);
    socklen_t length = sizeof(address);
    CHECK(::getsockname(
        fd,
        reinterpret_cast<sockaddr *>(&address),
        &length) == 0);
    const int port = ntohs(address.sin_port);
    ::close(fd);
    return port;
}

void TestReceiverReadinessAndBindConflict()
{
    const char *old_port = std::getenv("G1_VISION_UDP_PORT");
    const bool had_old_port = old_port != nullptr;
    const std::string old_port_value = had_old_port ? old_port : "";

    const int port = FindFreeUdpPort();
    CHECK(port > 0);
    const std::string port_text = std::to_string(port);
    CHECK(::setenv("G1_VISION_UDP_PORT", port_text.c_str(), 1) == 0);
    {
        VisionUdpCommandReceiver first;
        CHECK(first.IsReady());
        VisionUdpCommandReceiver conflicting;
        CHECK(!conflicting.IsReady());
    }
    // The port must be reusable after the owning receiver is destroyed.
    {
        VisionUdpCommandReceiver after_release;
        CHECK(after_release.IsReady());
    }

    if (had_old_port)
    {
        CHECK(::setenv(
            "G1_VISION_UDP_PORT", old_port_value.c_str(), 1) == 0);
    }
    else
    {
        CHECK(::unsetenv("G1_VISION_UDP_PORT") == 0);
    }
}

// ---- End-to-end UDP smoke (no robot, no lowcmd) ---------------------------

// Runs a real UDP receiver on G1_VISION_UDP_PORT (set by the caller) and
// loops until a Python sender has delivered packets or a timeout elapses.
// This mirrors exactly what the real pipeline does: Python UdpCommandSender
// -> UDP socket -> C++ VisionUdpCommandReceiver::Poll.
//
// The end-to-end part only runs when G1_RUN_SMOKE=1 (driven by
// run_udp_smoke.sh); without a Python sender it would always time out.
void TestEndToEndSmoke()
{
    VisionSprintMode::SetMode(vision_mode::Mode::NONE);

    // The receiver reads G1_VISION_UDP_PORT from the environment.
    VisionUdpCommandReceiver receiver;
    VisionSprintMode::SetWalk0p5m(true);

    float vx = 1.0f, vy = 1.0f, wz = 1.0f;
    // No packet yet: must stay zero.
    const bool applied0 = receiver.Apply(vx, vy, wz);
    CHECK(applied0);
    CHECK(vx == 0.0f && vy == 0.0f && wz == 0.0f);

    const char *run_smoke = std::getenv("G1_RUN_SMOKE");
    if (run_smoke == nullptr || std::string(run_smoke) != "1")
    {
        VisionSprintMode::SetWalk0p5m(false);
        VisionSprintMode::SetMode(vision_mode::Mode::NONE);
        std::printf("[smoke] skipped (set G1_RUN_SMOKE=1 to exercise UDP)\n");
        return;
    }

    // Phase 1: the Python sender delivers ONLY invalid packets (NaN, trailing
    // junk, bad hard_stop). They must not refresh freshness / command /
    // hard_stop / generation state, so HasFreshCommand() must stay false.
    std::printf("[smoke] phase 1: only invalid packets (no refresh expected)\n");
    for (int i = 0; i < 100; ++i)
    {
        float zx = 1.0f, zy = 1.0f, zw = 1.0f;
        receiver.Apply(zx, zy, zw);
        usleep(20000);
    }
    if (receiver.HasFreshCommand())
    {
        std::printf("FAIL: invalid packets refreshed freshness\n");
        ++g_failures;
    }
    if (receiver.HardStopRequested())
    {
        std::printf("FAIL: invalid packets latched hard_stop\n");
        ++g_failures;
    }

    // Phase 2: the Python sender now delivers valid clamped commands.
    std::printf("[smoke] phase 2: valid commands expected\n");
    const int deadline_iters = 300;
    bool saw_fwd = false;
    bool saw_hard_stop = false;
    bool saw_clamped = false;
    for (int i = 0; i < deadline_iters; ++i)
    {
        vx = 5.0f; vy = 2.0f; wz = 5.0f;
        const bool applied = receiver.Apply(vx, vy, wz);
        CHECK(applied);
        // Num7 hard caps: vx in [0, 0.50], vy=0, |wz|<=0.25.
        if (vx > 0.0f && vx <= 0.50f && vy == 0.0f &&
            std::fabs(wz) <= 0.25f)
        {
            saw_clamped = true;
            if (vx > 0.0f)
            {
                saw_fwd = true;
            }
        }
        if (receiver.HardStopRequested())
        {
            saw_hard_stop = true;
        }
        if (saw_fwd && saw_hard_stop && saw_clamped)
        {
            break;
        }
        usleep(10000);
    }

    if (!receiver.HasFreshCommand())
    {
        std::printf("FAIL: valid command did not set freshness\n");
        ++g_failures;
    }
    if (!saw_fwd)
    {
        std::printf("FAIL: no forward clamped command received\n");
        ++g_failures;
    }
    if (!saw_hard_stop)
    {
        std::printf("FAIL: no hard-stop packet received\n");
        ++g_failures;
    }

    VisionSprintMode::SetWalk0p5m(false);
    VisionSprintMode::SetMode(vision_mode::Mode::NONE);
}

}  // namespace

int main()
{
    TestStrictParseValidThreeFloats();
    TestStrictParseValidWithHardStop();
    TestPythonSenderPayloadsWithNewline();
    TestStrictParseRejectsNan();
    TestStrictParseRejectsTrailingJunk();
    TestModeLifecycle();
    TestNum7EntryDecision();
    TestReceiverReadinessAndBindConflict();
    TestEndToEndSmoke();

    if (g_failures == 0)
    {
        std::printf("ALL VISION UDP NUM7 TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURES\n", g_failures);
    return 1;
}
