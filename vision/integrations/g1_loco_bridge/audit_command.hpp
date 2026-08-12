#ifndef G1_AUDIT_COMMAND_HPP
#define G1_AUDIT_COMMAND_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>

namespace g1_audit {

struct Command {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  bool hard_stop = false;
};

struct Limits {
  double max_vx = 0.10;
  double max_abs_wz = 0.10;
};

inline bool ParseCommand(const std::string& payload, Command& command) {
  std::istringstream stream(payload);
  Command parsed;
  double hard_stop = 0.0;
  std::string trailing;
  if (!(stream >> parsed.vx >> parsed.vy >> parsed.wz)) {
    return false;
  }
  if (stream >> hard_stop) {
    if (!std::isfinite(hard_stop) || (hard_stop != 0.0 && hard_stop != 1.0)) {
      return false;
    }
    parsed.hard_stop = hard_stop == 1.0;
    if (stream >> trailing) {
      return false;
    }
  } else {
    // EOF after three fields is the legacy-compatible form. Any other parse
    // failure means a malformed fourth field and must not refresh freshness.
    if (!stream.eof()) {
      return false;
    }
  }
  if (!std::isfinite(parsed.vx) || !std::isfinite(parsed.vy) ||
      !std::isfinite(parsed.wz)) {
    return false;
  }
  command = parsed;
  return true;
}

inline Command LimitCommand(const Command& input, const Limits& limits) {
  Command output;
  if (input.hard_stop) {
    output.hard_stop = true;
    return output;
  }
  output.vx = std::clamp(input.vx, 0.0, limits.max_vx);
  output.vy = 0.0;
  output.wz = std::clamp(input.wz, -limits.max_abs_wz, limits.max_abs_wz);
  return output;
}

class Watchdog {
 public:
  using Clock = std::chrono::steady_clock;

  explicit Watchdog(std::chrono::milliseconds timeout) : timeout_(timeout) {}

  void Update(const Command& command, Clock::time_point now) {
    command_ = command;
    last_update_ = now;
    received_ = true;
  }

  bool Fresh(Clock::time_point now) const {
    return received_ && now - last_update_ <= timeout_;
  }

  Command Current(Clock::time_point now) const {
    return Fresh(now) ? command_ : Command{};
  }

 private:
  std::chrono::milliseconds timeout_;
  Command command_{};
  Clock::time_point last_update_{};
  bool received_ = false;
};

}  // namespace g1_audit

#endif
