#include "audit_command.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

namespace {

std::atomic<bool> running{true};

void HandleSignal(int) { running.store(false); }

struct Options {
  int port = 15003;
  int timeout_ms = 200;
  double max_vx = 0.10;
  double max_wz = 0.10;
  std::string network_interface = "lo";
  bool query_status = false;
};

int ParseInt(const std::string& text, int minimum, int maximum) {
  std::size_t used = 0;
  const int value = std::stoi(text, &used);
  if (used != text.size() || value < minimum || value > maximum) {
    throw std::invalid_argument("integer option out of range: " + text);
  }
  return value;
}

double ParseDouble(const std::string& text, double minimum, double maximum) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value) || value < minimum ||
      value > maximum) {
    throw std::invalid_argument("numeric option out of range: " + text);
  }
  return value;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto separator = argument.find('=');
    const std::string key = argument.substr(0, separator);
    const std::string value = separator == std::string::npos
                                  ? ""
                                  : argument.substr(separator + 1);
    if (key == "--mode") {
      if (value != "audit") {
        throw std::invalid_argument(
            "only --mode=audit exists; motion mode is intentionally absent");
      }
    } else if (key == "--port") {
      options.port = ParseInt(value, 1, 65535);
    } else if (key == "--timeout-ms") {
      options.timeout_ms = ParseInt(value, 50, 5000);
    } else if (key == "--max-vx") {
      options.max_vx = ParseDouble(value, 0.01, 0.50);
    } else if (key == "--max-wz") {
      options.max_wz = ParseDouble(value, 0.01, 0.50);
    } else if (key == "--network-interface") {
      if (value.empty()) {
        throw std::invalid_argument("network interface cannot be empty");
      }
      options.network_interface = value;
    } else if (key == "--query-status" && value.empty()) {
      options.query_status = true;
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  return options;
}

void QueryStatus(const std::string& network_interface) {
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::g1::LocoClient client;
  client.Init();
  client.SetTimeout(2.0F);

  int fsm_id = -1;
  int fsm_mode = -1;
  int balance_mode = -1;
  const int fsm_id_result = client.GetFsmId(fsm_id);
  const int fsm_mode_result = client.GetFsmMode(fsm_mode);
  const int balance_result = client.GetBalanceMode(balance_mode);
  std::cout << "[audit] read-only loco status: fsm_id=" << fsm_id
            << " (ret=" << fsm_id_result << "), fsm_mode=" << fsm_mode
            << " (ret=" << fsm_mode_result << "), balance_mode="
            << balance_mode << " (ret=" << balance_result << ")" << std::endl;
}

int OpenSocket(int port) {
  const int descriptor = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (descriptor < 0) {
    throw std::runtime_error(std::string("socket failed: ") +
                             std::strerror(errno));
  }
  const int flags = ::fcntl(descriptor, F_GETFL, 0);
  if (flags < 0 || ::fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) < 0) {
    ::close(descriptor);
    throw std::runtime_error("failed to make audit socket non-blocking");
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(descriptor, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error("bind 127.0.0.1:" + std::to_string(port) +
                             " failed: " + error);
  }
  return descriptor;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout
        << "[audit] AUDIT ONLY: robot motion calls are not implemented in this "
           "binary"
        << std::endl;
    if (options.query_status) {
      QueryStatus(options.network_interface);
    }

    const int socket_fd = OpenSocket(options.port);
    g1_audit::Watchdog watchdog(
        std::chrono::milliseconds(options.timeout_ms));
    const g1_audit::Limits limits{options.max_vx, options.max_wz};
    bool was_fresh = false;
    auto last_report = g1_audit::Watchdog::Clock::now() -
                       std::chrono::seconds(1);

    std::cout << "[audit] listening on 127.0.0.1:" << options.port
              << ", timeout=" << options.timeout_ms
              << " ms, max_vx=" << options.max_vx
              << ", max_abs_wz=" << options.max_wz << std::endl;

    while (running.load()) {
      while (true) {
        char buffer[256]{};
        const ssize_t length = ::recv(socket_fd, buffer, sizeof(buffer) - 1, 0);
        if (length < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
          }
          throw std::runtime_error(std::string("recv failed: ") +
                                   std::strerror(errno));
        }
        g1_audit::Command parsed;
        if (!g1_audit::ParseCommand(std::string(buffer, length), parsed)) {
          std::cerr << "[audit] rejected malformed command" << std::endl;
          continue;
        }
        watchdog.Update(g1_audit::LimitCommand(parsed, limits),
                        g1_audit::Watchdog::Clock::now());
      }

      const auto now = g1_audit::Watchdog::Clock::now();
      const bool fresh = watchdog.Fresh(now);
      const auto command = watchdog.Current(now);
      if (fresh != was_fresh) {
        std::cout << (fresh ? "[audit] command stream fresh"
                            : "[audit] command timeout -> zero")
                  << std::endl;
        was_fresh = fresh;
      }
      if (fresh && now - last_report >= std::chrono::milliseconds(500)) {
        std::cout << std::fixed << std::setprecision(3)
                  << "[audit] limited command vx=" << command.vx
                  << " vy=" << command.vy << " wz=" << command.wz
                  << " hard_stop=" << (command.hard_stop ? 1 : 0)
                  << " (not sent to robot)" << std::endl;
        last_report = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ::close(socket_fd);
    std::cout << "[audit] stopped; no robot command was sent" << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[audit] fatal: " << error.what() << std::endl;
    return 1;
  }
}
