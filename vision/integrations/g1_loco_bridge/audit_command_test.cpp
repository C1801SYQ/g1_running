#include "audit_command.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

int main() {
  using Clock = g1_audit::Watchdog::Clock;
  g1_audit::Command command;
  assert(g1_audit::ParseCommand("0.20 0.30 -0.40\n", command));
  assert(!command.hard_stop);
  assert(g1_audit::ParseCommand("0.20 0.30 -0.40 0\r\n", command));
  assert(!command.hard_stop);
  assert(g1_audit::ParseCommand("0.20 0.30 -0.40 1\n", command));
  assert(command.hard_stop);
  assert(!g1_audit::ParseCommand("0.20 0.0" , command));
  assert(!g1_audit::ParseCommand("nan 0.0 0.0", command));
  assert(!g1_audit::ParseCommand("0.1 0.0 0.0 2", command));
  assert(!g1_audit::ParseCommand("0.1 0.0 0.0 nope", command));
  assert(!g1_audit::ParseCommand("0.1 0.0 0.0 extra", command));

  const auto limited = g1_audit::LimitCommand(
      g1_audit::Command{0.20, 0.30, -0.40},
      g1_audit::Limits{0.10, 0.10});
  assert(std::abs(limited.vx - 0.10) < 1e-9);
  assert(limited.vy == 0.0);
  assert(std::abs(limited.wz + 0.10) < 1e-9);

  const auto hard_stopped = g1_audit::LimitCommand(
      g1_audit::Command{0.20, 0.0, 0.10, true},
      g1_audit::Limits{0.50, 0.25});
  assert(hard_stopped.vx == 0.0 && hard_stopped.vy == 0.0 &&
         hard_stopped.wz == 0.0 && hard_stopped.hard_stop);

  g1_audit::Watchdog watchdog(std::chrono::milliseconds(200));
  const auto start = Clock::now();
  assert(!watchdog.Fresh(start));
  watchdog.Update(g1_audit::Command{0.05, 0.0, 0.02}, start);
  assert(watchdog.Fresh(start + std::chrono::milliseconds(200)));
  assert(!watchdog.Fresh(start + std::chrono::milliseconds(201)));
  const auto stopped = watchdog.Current(start + std::chrono::milliseconds(201));
  assert(stopped.vx == 0.0 && stopped.vy == 0.0 && stopped.wz == 0.0);

  std::cout << "audit command tests passed" << std::endl;
  return 0;
}
