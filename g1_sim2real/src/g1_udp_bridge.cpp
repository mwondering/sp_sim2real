#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <termios.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace g1_bridge {
namespace {

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using SteadyClock = std::chrono::steady_clock;

constexpr char kTypeKey[] = "__udp_latest_type__";
constexpr std::array<uint8_t, 4> kMagic = {'U', 'L', 'D', 'P'};
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = 4 + 1 + 1 + 2 + 8 + 8 + 4 + 4;
constexpr size_t kCrcSize = 4;
constexpr int kG1MotorCount = 29;
constexpr uint64_t kStdinButtonPulseNs = 200'000'000ULL;
constexpr char kAnsiBoldRed[] = "\033[1;31m";
constexpr char kAnsiReset[] = "\033[0m";

std::atomic<bool> g_stop_requested{false};

void handle_signal(int)
{
  g_stop_requested.store(true);
}

uint64_t now_ns()
{
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now().time_since_epoch()).count());
}

class ScopedTerminalRawMode {
 public:
  explicit ScopedTerminalRawMode(int fd) : fd_(fd)
  {
    if (!::isatty(fd_)) {
      return;
    }
    if (::tcgetattr(fd_, &original_) != 0) {
      std::cerr << "[G1Bridge] Warning: tcgetattr(stdin) failed: " << std::strerror(errno) << std::endl;
      return;
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &raw) != 0) {
      std::cerr << "[G1Bridge] Warning: tcsetattr(stdin raw mode) failed: " << std::strerror(errno) << std::endl;
      return;
    }
    enabled_ = true;
  }

  ~ScopedTerminalRawMode()
  {
    if (enabled_) {
      ::tcsetattr(fd_, TCSANOW, &original_);
    }
  }

  bool enabled() const { return enabled_; }

 private:
  int fd_ = -1;
  termios original_{};
  bool enabled_ = false;
};

std::string errno_text(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

void append_u16_be(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  out.push_back(static_cast<uint8_t>(value & 0xff));
}

void append_u32_be(std::vector<uint8_t> & out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  out.push_back(static_cast<uint8_t>(value & 0xff));
}

void append_u64_be(std::vector<uint8_t> & out, uint64_t value)
{
  for (int shift = 56; shift >= 0; shift -= 8) {
    out.push_back(static_cast<uint8_t>((value >> shift) & 0xff));
  }
}

uint32_t read_u32_be(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

uint64_t read_u64_be(const uint8_t * p)
{
  uint64_t value = 0;
  for (size_t i = 0; i < 8; ++i) {
    value = (value << 8) | static_cast<uint64_t>(p[i]);
  }
  return value;
}

uint32_t crc32_core(uint32_t * ptr, uint32_t len)
{
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xffffffff;
  constexpr uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1U << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1;
        crc32 ^= polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) {
        crc32 ^= polynomial;
      }
      xbit >>= 1;
    }
  }
  return crc32;
}

std::vector<std::string> yaml_string_vector(const YAML::Node & node, const std::string & path)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error(path + " must be a sequence");
  }
  std::vector<std::string> out;
  out.reserve(node.size());
  for (const auto & item : node) {
    out.push_back(item.as<std::string>());
  }
  return out;
}

template <typename T>
T yaml_required(const YAML::Node & node, const std::string & path)
{
  if (!node) {
    throw std::runtime_error(path + " is required");
  }
  return node.as<T>();
}

template <typename T>
T yaml_value_or(const YAML::Node & node, const T & fallback)
{
  if (!node) {
    return fallback;
  }
  return node.as<T>();
}

std::unordered_map<std::string, size_t> build_index_map(const std::vector<std::string> & names)
{
  std::unordered_map<std::string, size_t> out;
  out.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    if (!out.emplace(names[i], i).second) {
      throw std::runtime_error("Duplicate joint name: " + names[i]);
    }
  }
  return out;
}

std::vector<size_t> build_source_indices(
    const std::vector<std::string> & target_names, const std::unordered_map<std::string, size_t> & source_index,
    const std::string & description)
{
  std::vector<size_t> out;
  out.reserve(target_names.size());
  for (const auto & name : target_names) {
    const auto it = source_index.find(name);
    if (it == source_index.end()) {
      throw std::runtime_error(description + " missing joint: " + name);
    }
    out.push_back(it->second);
  }
  return out;
}

void validate_udp_port(int port, const std::string & path)
{
  if (port < 1 || port > 65535) {
    throw std::runtime_error(path + " must be in [1, 65535]");
  }
}

enum class StatePublishMode {
  LowStateTick,
  Timer,
};

std::string normalized_mode_name(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

StatePublishMode parse_state_publish_mode(const std::string & value)
{
  const std::string normalized = normalized_mode_name(value);
  if (normalized == "lowstate_tick" || normalized == "lowstate" || normalized == "tick" ||
      normalized == "callback") {
    return StatePublishMode::LowStateTick;
  }
  if (normalized == "timer" || normalized == "periodic") {
    return StatePublishMode::Timer;
  }
  throw std::runtime_error(
      "freq.state_publish_mode must be one of lowstate_tick or timer, got: " + value);
}

const char * state_publish_mode_name(StatePublishMode mode)
{
  switch (mode) {
    case StatePublishMode::LowStateTick:
      return "lowstate_tick";
    case StatePublishMode::Timer:
      return "timer";
  }
  return "unknown";
}

struct UdpConfig {
  std::string state_host;
  int state_port = 0;
  std::string cmd_bind_host;
  int cmd_port = 0;
  std::string cmd_allowed_host;
  int recvbuf_bytes = 1 << 20;
  int sndbuf_bytes = 1 << 20;
};

struct FrequencyConfig {
  double physical_hz = 1000.0;
  size_t state_decimation = 10;
  StatePublishMode state_publish_mode = StatePublishMode::LowStateTick;
};

struct LowLevelConfig {
  uint8_t mode_pr = 0;
  double damping_kd = 8.0;
  double wait_lowstate_timeout_s = 0.0;
  bool release_motion_service = true;
  bool release_required = true;
  int release_max_attempts = 5;
  double release_sleep_s = 1.0;
};

struct SafetyConfig {
  bool enabled = false;
  double command_timeout_s = 0.0;
  bool startup_damping = true;
  double damping_publish_hz = 50.0;
};

struct DiagnosticsConfig {
  bool enabled = false;
  double report_interval_s = 1.0;
  double motor_casing_warn_c = 75.0;
  double motor_casing_limit_c = 85.0;
  double motor_winding_warn_c = 100.0;
  double motor_winding_limit_c = 120.0;
  double joint_velocity_warn_rad_s = 8.0;
  double joint_velocity_limit_rad_s = 10.0;
  double imu_angular_velocity_warn_rad_s = 5.0;
  double imu_angular_velocity_limit_rad_s = 6.0;
  double joint_position_warn_margin_rad = 0.05;
  double joint_torque_warn_ratio = 0.9;
  bool check_joint_position = true;
  bool check_joint_torque = true;
  std::vector<double> joint_position_lower;
  std::vector<double> joint_position_upper;
  std::vector<double> joint_torque_limit;
};

enum MotorDiagnosticFlag : uint32_t {
  kMotorStateFault = 1U << 0,
  kCasingTemperatureWarning = 1U << 1,
  kCasingTemperatureLimit = 1U << 2,
  kWindingTemperatureWarning = 1U << 3,
  kWindingTemperatureLimit = 1U << 4,
  kJointPositionWarning = 1U << 5,
  kJointPositionLimit = 1U << 6,
  kJointVelocityWarning = 1U << 7,
  kJointVelocityLimit = 1U << 8,
  kJointTorqueWarning = 1U << 9,
  kJointTorqueLimit = 1U << 10,
  kMotorStateNonFinite = 1U << 11,
};

enum ImuDiagnosticFlag : uint32_t {
  kImuAngularVelocityWarning = 1U << 0,
  kImuAngularVelocityLimit = 1U << 1,
  kImuStateNonFinite = 1U << 2,
};

constexpr uint32_t kMotorWarningMask =
    kCasingTemperatureWarning | kWindingTemperatureWarning | kJointPositionWarning |
    kJointVelocityWarning | kJointTorqueWarning;
constexpr uint32_t kMotorCriticalMask =
    kMotorStateFault | kCasingTemperatureLimit | kWindingTemperatureLimit |
    kJointPositionLimit | kJointVelocityLimit | kJointTorqueLimit | kMotorStateNonFinite;
constexpr uint32_t kImuWarningMask = kImuAngularVelocityWarning;
constexpr uint32_t kImuCriticalMask = kImuAngularVelocityLimit | kImuStateNonFinite;

struct DiagnosticSnapshot {
  std::vector<uint32_t> motor_flags;
  uint32_t imu_flags = 0;
  uint32_t warning_count = 0;
  uint32_t critical_count = 0;
};

struct BridgeConfig {
  std::string lowcmd_topic;
  std::string lowstate_topic;
  UdpConfig udp;
  FrequencyConfig freq;
  LowLevelConfig low_level;
  SafetyConfig safety;
  DiagnosticsConfig diagnostics;
  std::vector<std::string> policy_joint_names;
  std::vector<std::string> real_joint_names;
};

struct StateSnapshot {
  LowState low_state;
  std::vector<float> tau_real_average;
  DiagnosticSnapshot diagnostics;
};

const std::unordered_map<std::string, std::pair<double, double>> & g1_joint_position_limits()
{
  // Mirror sim2real/config/g1/assets/g1.xml. These are diagnostic boundaries,
  // not a replacement for firmware limits.
  static const std::unordered_map<std::string, std::pair<double, double>> limits{
      {"left_hip_pitch_joint", {-2.5307, 2.8798}},
      {"left_hip_roll_joint", {-0.5236, 2.9671}},
      {"left_hip_yaw_joint", {-2.7576, 2.7576}},
      {"left_knee_joint", {-0.087267, 2.8798}},
      {"left_ankle_pitch_joint", {-0.87267, 0.5236}},
      {"left_ankle_roll_joint", {-0.2618, 0.2618}},
      {"right_hip_pitch_joint", {-2.5307, 2.8798}},
      {"right_hip_roll_joint", {-2.9671, 0.5236}},
      {"right_hip_yaw_joint", {-2.7576, 2.7576}},
      {"right_knee_joint", {-0.087267, 2.8798}},
      {"right_ankle_pitch_joint", {-0.87267, 0.5236}},
      {"right_ankle_roll_joint", {-0.2618, 0.2618}},
      {"waist_yaw_joint", {-2.618, 2.618}},
      {"waist_roll_joint", {-0.52, 0.52}},
      {"waist_pitch_joint", {-0.52, 0.52}},
      {"left_shoulder_pitch_joint", {-3.0892, 2.6704}},
      {"left_shoulder_roll_joint", {-1.5882, 2.2515}},
      {"left_shoulder_yaw_joint", {-2.618, 2.618}},
      {"left_elbow_joint", {-1.0472, 2.0944}},
      {"left_wrist_roll_joint", {-1.97222, 1.97222}},
      {"left_wrist_pitch_joint", {-1.61443, 1.61443}},
      {"left_wrist_yaw_joint", {-1.61443, 1.61443}},
      {"right_shoulder_pitch_joint", {-3.0892, 2.6704}},
      {"right_shoulder_roll_joint", {-2.2515, 1.5882}},
      {"right_shoulder_yaw_joint", {-2.618, 2.618}},
      {"right_elbow_joint", {-1.0472, 2.0944}},
      {"right_wrist_roll_joint", {-1.97222, 1.97222}},
      {"right_wrist_pitch_joint", {-1.61443, 1.61443}},
      {"right_wrist_yaw_joint", {-1.61443, 1.61443}},
  };
  return limits;
}

const std::unordered_map<std::string, double> & g1_joint_torque_limits()
{
  // Mirror actuator ctrlrange values in sim2real/config/g1/assets/g1.xml.
  static const std::unordered_map<std::string, double> limits{
      {"left_hip_pitch_joint", 88.0}, {"left_hip_roll_joint", 139.0},
      {"left_hip_yaw_joint", 88.0}, {"left_knee_joint", 139.0},
      {"left_ankle_pitch_joint", 35.0}, {"left_ankle_roll_joint", 35.0},
      {"right_hip_pitch_joint", 88.0}, {"right_hip_roll_joint", 139.0},
      {"right_hip_yaw_joint", 88.0}, {"right_knee_joint", 139.0},
      {"right_ankle_pitch_joint", 35.0}, {"right_ankle_roll_joint", 35.0},
      {"waist_yaw_joint", 88.0}, {"waist_roll_joint", 35.0},
      {"waist_pitch_joint", 35.0},
      {"left_shoulder_pitch_joint", 25.0}, {"left_shoulder_roll_joint", 25.0},
      {"left_shoulder_yaw_joint", 25.0}, {"left_elbow_joint", 25.0},
      {"left_wrist_roll_joint", 25.0}, {"left_wrist_pitch_joint", 5.0},
      {"left_wrist_yaw_joint", 5.0},
      {"right_shoulder_pitch_joint", 25.0}, {"right_shoulder_roll_joint", 25.0},
      {"right_shoulder_yaw_joint", 25.0}, {"right_elbow_joint", 25.0},
      {"right_wrist_roll_joint", 25.0}, {"right_wrist_pitch_joint", 5.0},
      {"right_wrist_yaw_joint", 5.0},
  };
  return limits;
}

BridgeConfig load_config(const std::string & path)
{
  const YAML::Node raw = YAML::LoadFile(path);
  BridgeConfig cfg;
  cfg.lowcmd_topic = yaml_required<std::string>(raw["lowcmd_topic"], "lowcmd_topic");
  cfg.lowstate_topic = yaml_required<std::string>(raw["lowstate_topic"], "lowstate_topic");

  const YAML::Node udp = raw["udp"];
  cfg.udp.state_host = yaml_required<std::string>(udp["state_host"], "udp.state_host");
  cfg.udp.state_port = yaml_required<int>(udp["state_port"], "udp.state_port");
  cfg.udp.cmd_bind_host = yaml_required<std::string>(udp["cmd_bind_host"], "udp.cmd_bind_host");
  cfg.udp.cmd_port = yaml_required<int>(udp["cmd_port"], "udp.cmd_port");
  cfg.udp.cmd_allowed_host = yaml_value_or<std::string>(udp["cmd_allowed_host"], "");
  cfg.udp.recvbuf_bytes = yaml_value_or<int>(udp["recvbuf_bytes"], 1 << 20);
  cfg.udp.sndbuf_bytes = yaml_value_or<int>(udp["sndbuf_bytes"], 1 << 20);
  validate_udp_port(cfg.udp.state_port, "udp.state_port");
  validate_udp_port(cfg.udp.cmd_port, "udp.cmd_port");
  if (cfg.udp.recvbuf_bytes < 0 || cfg.udp.sndbuf_bytes < 0) {
    throw std::runtime_error("udp recv/send buffer sizes must be non-negative");
  }
  if (!cfg.udp.cmd_allowed_host.empty()) {
    in_addr allowed_addr{};
    if (::inet_pton(AF_INET, cfg.udp.cmd_allowed_host.c_str(), &allowed_addr) != 1) {
      throw std::runtime_error("udp.cmd_allowed_host must be an IPv4 address or empty");
    }
  }

  const YAML::Node freq = raw["freq"];
  cfg.freq.physical_hz = yaml_required<double>(freq["physical_hz"], "freq.physical_hz");
  cfg.freq.state_decimation = yaml_required<size_t>(freq["state_decimation"], "freq.state_decimation");
  cfg.freq.state_publish_mode =
      parse_state_publish_mode(yaml_value_or<std::string>(freq["state_publish_mode"], "lowstate_tick"));
  if (cfg.freq.physical_hz <= 0.0) {
    throw std::runtime_error("freq.physical_hz must be positive");
  }
  if (cfg.freq.state_decimation == 0) {
    throw std::runtime_error("freq.state_decimation must be positive");
  }
  if (cfg.freq.state_decimation > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("freq.state_decimation must fit in uint32_t LowState.tick arithmetic");
  }

  const YAML::Node low = raw["low_level"];
  cfg.low_level.mode_pr = static_cast<uint8_t>(yaml_value_or<int>(low["mode_pr"], 0));
  cfg.low_level.damping_kd = yaml_value_or<double>(low["damping_kd"], 8.0);
  cfg.low_level.wait_lowstate_timeout_s = yaml_value_or<double>(low["wait_lowstate_timeout_s"], 0.0);
  cfg.low_level.release_motion_service = yaml_value_or<bool>(low["release_motion_service"], true);
  cfg.low_level.release_required = yaml_value_or<bool>(low["release_required"], true);
  cfg.low_level.release_max_attempts = yaml_value_or<int>(low["release_max_attempts"], 5);
  cfg.low_level.release_sleep_s = yaml_value_or<double>(low["release_sleep_s"], 1.0);
  if (cfg.low_level.mode_pr > 1) {
    throw std::runtime_error("low_level.mode_pr must be 0 (PR) or 1 (AB)");
  }
  if (cfg.low_level.damping_kd < 0.0) {
    throw std::runtime_error("low_level.damping_kd must be non-negative");
  }
  if (cfg.low_level.wait_lowstate_timeout_s < 0.0) {
    throw std::runtime_error("low_level.wait_lowstate_timeout_s must be non-negative");
  }
  if (cfg.low_level.release_max_attempts < 1) {
    throw std::runtime_error("low_level.release_max_attempts must be positive");
  }
  if (cfg.low_level.release_sleep_s < 0.0) {
    throw std::runtime_error("low_level.release_sleep_s must be non-negative");
  }

  const YAML::Node safety = raw["safety"];
  if (safety) {
    cfg.safety.enabled = yaml_value_or<bool>(safety["enabled"], false);
    cfg.safety.command_timeout_s = yaml_value_or<double>(safety["command_timeout_s"], 0.0);
    cfg.safety.startup_damping = yaml_value_or<bool>(safety["startup_damping"], true);
    cfg.safety.damping_publish_hz = yaml_value_or<double>(safety["damping_publish_hz"], 50.0);
  }
  if (cfg.safety.enabled) {
    if (cfg.safety.command_timeout_s <= 0.0) {
      throw std::runtime_error("safety.command_timeout_s must be positive when safety is enabled");
    }
    if (cfg.safety.damping_publish_hz <= 0.0) {
      throw std::runtime_error("safety.damping_publish_hz must be positive");
    }
  }

  const YAML::Node diagnostics = raw["diagnostics"];
  if (diagnostics) {
    cfg.diagnostics.enabled = yaml_value_or<bool>(diagnostics["enabled"], false);
    cfg.diagnostics.report_interval_s =
        yaml_value_or<double>(diagnostics["report_interval_s"], 1.0);
    cfg.diagnostics.motor_casing_warn_c =
        yaml_value_or<double>(diagnostics["motor_casing_warn_c"], 75.0);
    cfg.diagnostics.motor_casing_limit_c =
        yaml_value_or<double>(diagnostics["motor_casing_limit_c"], 85.0);
    cfg.diagnostics.motor_winding_warn_c =
        yaml_value_or<double>(diagnostics["motor_winding_warn_c"], 100.0);
    cfg.diagnostics.motor_winding_limit_c =
        yaml_value_or<double>(diagnostics["motor_winding_limit_c"], 120.0);
    cfg.diagnostics.joint_velocity_warn_rad_s =
        yaml_value_or<double>(diagnostics["joint_velocity_warn_rad_s"], 8.0);
    cfg.diagnostics.joint_velocity_limit_rad_s =
        yaml_value_or<double>(diagnostics["joint_velocity_limit_rad_s"], 10.0);
    cfg.diagnostics.imu_angular_velocity_warn_rad_s =
        yaml_value_or<double>(diagnostics["imu_angular_velocity_warn_rad_s"], 5.0);
    cfg.diagnostics.imu_angular_velocity_limit_rad_s =
        yaml_value_or<double>(diagnostics["imu_angular_velocity_limit_rad_s"], 6.0);
    cfg.diagnostics.joint_position_warn_margin_rad =
        yaml_value_or<double>(diagnostics["joint_position_warn_margin_rad"], 0.05);
    cfg.diagnostics.joint_torque_warn_ratio =
        yaml_value_or<double>(diagnostics["joint_torque_warn_ratio"], 0.9);
    cfg.diagnostics.check_joint_position =
        yaml_value_or<bool>(diagnostics["check_joint_position"], true);
    cfg.diagnostics.check_joint_torque =
        yaml_value_or<bool>(diagnostics["check_joint_torque"], true);
  }
  if (cfg.diagnostics.enabled) {
    const auto validate_warning_limit = [](double warning, double limit, const std::string & name) {
      if (!(warning >= 0.0 && warning < limit)) {
        throw std::runtime_error(
            "diagnostics." + name + " warning must be non-negative and below its limit");
      }
    };
    if (cfg.diagnostics.report_interval_s <= 0.0) {
      throw std::runtime_error("diagnostics.report_interval_s must be positive");
    }
    validate_warning_limit(
        cfg.diagnostics.motor_casing_warn_c, cfg.diagnostics.motor_casing_limit_c,
        "motor_casing temperature");
    validate_warning_limit(
        cfg.diagnostics.motor_winding_warn_c, cfg.diagnostics.motor_winding_limit_c,
        "motor_winding temperature");
    validate_warning_limit(
        cfg.diagnostics.joint_velocity_warn_rad_s, cfg.diagnostics.joint_velocity_limit_rad_s,
        "joint_velocity");
    validate_warning_limit(
        cfg.diagnostics.imu_angular_velocity_warn_rad_s,
        cfg.diagnostics.imu_angular_velocity_limit_rad_s, "imu_angular_velocity");
    if (cfg.diagnostics.joint_position_warn_margin_rad < 0.0) {
      throw std::runtime_error("diagnostics.joint_position_warn_margin_rad must be non-negative");
    }
    if (!(cfg.diagnostics.joint_torque_warn_ratio > 0.0 &&
          cfg.diagnostics.joint_torque_warn_ratio < 1.0)) {
      throw std::runtime_error("diagnostics.joint_torque_warn_ratio must be in (0, 1)");
    }
  }

  cfg.policy_joint_names = yaml_string_vector(raw["policy_joint_names"], "policy_joint_names");
  cfg.real_joint_names = yaml_string_vector(raw["real_joint_names"], "real_joint_names");
  if (cfg.policy_joint_names.empty() || cfg.real_joint_names.empty()) {
    throw std::runtime_error("joint name lists must not be empty");
  }
  if (cfg.real_joint_names.size() > static_cast<size_t>(kG1MotorCount)) {
    throw std::runtime_error("real_joint_names has more joints than supported G1 motors");
  }
  cfg.diagnostics.joint_position_lower.reserve(cfg.real_joint_names.size());
  cfg.diagnostics.joint_position_upper.reserve(cfg.real_joint_names.size());
  cfg.diagnostics.joint_torque_limit.reserve(cfg.real_joint_names.size());
  for (const auto & joint_name : cfg.real_joint_names) {
    const auto position_it = g1_joint_position_limits().find(joint_name);
    const auto torque_it = g1_joint_torque_limits().find(joint_name);
    if (position_it == g1_joint_position_limits().end() ||
        torque_it == g1_joint_torque_limits().end()) {
      throw std::runtime_error("No built-in G1 diagnostic limit for joint: " + joint_name);
    }
    cfg.diagnostics.joint_position_lower.push_back(position_it->second.first);
    cfg.diagnostics.joint_position_upper.push_back(position_it->second.second);
    cfg.diagnostics.joint_torque_limit.push_back(torque_it->second);
  }
  return cfg;
}

struct RemoteState {
  bool start = false;
  bool stop = false;
  bool a = false;
  bool up = false;
  bool down = false;
  float lx = 0.0f;
  float ly = 0.0f;
  float rx = 0.0f;
  float ry = 0.0f;
};

struct MotorTelemetry {
  std::vector<float> ddq;
  std::vector<float> casing_temperature;
  std::vector<float> winding_temperature;
  std::vector<float> voltage;
  std::vector<uint32_t> mode;
  std::vector<uint32_t> sensor_0;
  std::vector<uint32_t> sensor_1;
  std::vector<uint32_t> state;
  std::array<std::vector<uint32_t>, 4> reserve;
};

float read_float_le(const std::array<uint8_t, 40> & bytes, size_t offset)
{
  float value = 0.0f;
  std::memcpy(&value, bytes.data() + offset, sizeof(float));
  return value;
}

RemoteState parse_remote(const std::array<uint8_t, 40> & remote)
{
  const uint16_t keys = static_cast<uint16_t>(remote[2]) | (static_cast<uint16_t>(remote[3]) << 8);
  RemoteState out;
  out.start = (keys & (1U << 2)) != 0;
  out.stop = (keys & (1U << 3)) != 0;
  out.a = (keys & (1U << 8)) != 0;
  out.up = (keys & (1U << 12)) != 0;
  out.down = (keys & (1U << 14)) != 0;
  out.lx = read_float_le(remote, 4);
  out.rx = read_float_le(remote, 8);
  out.ry = read_float_le(remote, 12);
  out.ly = read_float_le(remote, 20);
  return out;
}

struct PackedArray {
  size_t offset = 0;
  size_t nbytes = 0;
  size_t count = 0;
};

PackedArray append_float_array(std::vector<uint8_t> & payload, const std::vector<float> & values)
{
  PackedArray ref;
  ref.offset = payload.size();
  ref.count = values.size();
  ref.nbytes = values.size() * sizeof(float);
  payload.resize(ref.offset + ref.nbytes);
  if (ref.nbytes > 0) {
    std::memcpy(payload.data() + ref.offset, values.data(), ref.nbytes);
  }
  return ref;
}

PackedArray append_u32_array(std::vector<uint8_t> & payload, const std::vector<uint32_t> & values)
{
  PackedArray ref;
  ref.offset = payload.size();
  ref.count = values.size();
  ref.nbytes = values.size() * sizeof(uint32_t);
  payload.resize(ref.offset + ref.nbytes);
  if (ref.nbytes > 0) {
    std::memcpy(payload.data() + ref.offset, values.data(), ref.nbytes);
  }
  return ref;
}

std::string ndarray_meta(const PackedArray & ref, const char * dtype = "<f4")
{
  std::ostringstream out;
  out << "{\"" << kTypeKey << "\":\"ndarray\",\"dtype\":\"" << dtype << "\",\"shape\":[" << ref.count
      << "],\"offset\":" << ref.offset << ",\"nbytes\":" << ref.nbytes << "}";
  return out.str();
}

const char * bool_text(bool value)
{
  return value ? "true" : "false";
}

void append_float_json(std::ostringstream & out, float value)
{
  out << std::setprecision(9) << static_cast<double>(value);
}

std::vector<uint8_t> encode_datagram(const std::string & meta_text, const std::vector<uint8_t> & payload, uint64_t seq)
{
  if (meta_text.size() > std::numeric_limits<uint32_t>::max() ||
      payload.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("UDP payload too large");
  }

  std::vector<uint8_t> packet;
  packet.reserve(kHeaderSize + kCrcSize + meta_text.size() + payload.size());
  packet.insert(packet.end(), kMagic.begin(), kMagic.end());
  packet.push_back(kVersion);
  packet.push_back(0);
  append_u16_be(packet, 0);
  append_u64_be(packet, seq);
  append_u64_be(packet, now_ns());
  append_u32_be(packet, static_cast<uint32_t>(meta_text.size()));
  append_u32_be(packet, static_cast<uint32_t>(payload.size()));

  uLong crc = crc32(0, packet.data(), static_cast<uInt>(packet.size()));
  crc = crc32(crc, reinterpret_cast<const Bytef *>(meta_text.data()), static_cast<uInt>(meta_text.size()));
  if (!payload.empty()) {
    crc = crc32(crc, payload.data(), static_cast<uInt>(payload.size()));
  }
  append_u32_be(packet, static_cast<uint32_t>(crc & 0xffffffffU));
  packet.insert(packet.end(), meta_text.begin(), meta_text.end());
  packet.insert(packet.end(), payload.begin(), payload.end());
  return packet;
}

class UdpLatestSender {
 public:
  UdpLatestSender(const std::string & host, int port, int sndbuf_bytes)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error(errno_text("socket(AF_INET, SOCK_DGRAM) failed"));
    }
    if (sndbuf_bytes > 0) {
      ::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf_bytes, sizeof(sndbuf_bytes));
    }
    target_.sin_family = AF_INET;
    target_.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &target_.sin_addr) != 1) {
      throw std::runtime_error("Invalid UDP target host: " + host);
    }
  }

  ~UdpLatestSender()
  {
    close();
  }

  UdpLatestSender(const UdpLatestSender &) = delete;
  UdpLatestSender & operator=(const UdpLatestSender &) = delete;

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void send_state(
      const std::vector<float> & q, const std::vector<float> & dq, const std::vector<float> & tau,
      const std::vector<float> & tau_latest,
      const std::vector<float> & quat,
      const std::vector<float> & gyro, const std::vector<float> & linacc, const RemoteState & remote,
      const MotorTelemetry & motor, const DiagnosticSnapshot & diagnostics, uint8_t mode_machine)
  {
    std::vector<uint8_t> payload;
    payload.reserve(
        (q.size() + dq.size() + tau.size() + tau_latest.size() + quat.size() + gyro.size() + linacc.size() +
         motor.ddq.size() + motor.casing_temperature.size() + motor.winding_temperature.size() +
         motor.voltage.size()) * sizeof(float) +
        (motor.mode.size() + motor.sensor_0.size() + motor.sensor_1.size() + motor.state.size() +
         diagnostics.motor_flags.size() + motor.reserve[0].size() + motor.reserve[1].size() +
         motor.reserve[2].size() + motor.reserve[3].size()) * sizeof(uint32_t));
    const PackedArray q_ref = append_float_array(payload, q);
    const PackedArray dq_ref = append_float_array(payload, dq);
    const PackedArray tau_ref = append_float_array(payload, tau);
    const PackedArray tau_latest_ref = append_float_array(payload, tau_latest);
    const PackedArray quat_ref = append_float_array(payload, quat);
    const PackedArray gyro_ref = append_float_array(payload, gyro);
    const PackedArray linacc_ref = append_float_array(payload, linacc);
    const PackedArray motor_ddq_ref = append_float_array(payload, motor.ddq);
    const PackedArray casing_temperature_ref = append_float_array(payload, motor.casing_temperature);
    const PackedArray winding_temperature_ref = append_float_array(payload, motor.winding_temperature);
    const PackedArray motor_voltage_ref = append_float_array(payload, motor.voltage);
    const PackedArray motor_mode_ref = append_u32_array(payload, motor.mode);
    const PackedArray motor_sensor_0_ref = append_u32_array(payload, motor.sensor_0);
    const PackedArray motor_sensor_1_ref = append_u32_array(payload, motor.sensor_1);
    const PackedArray motor_state_ref = append_u32_array(payload, motor.state);
    const PackedArray motor_reserve_0_ref = append_u32_array(payload, motor.reserve[0]);
    const PackedArray motor_reserve_1_ref = append_u32_array(payload, motor.reserve[1]);
    const PackedArray motor_reserve_2_ref = append_u32_array(payload, motor.reserve[2]);
    const PackedArray motor_reserve_3_ref = append_u32_array(payload, motor.reserve[3]);
    const PackedArray motor_diagnostic_flags_ref = append_u32_array(payload, diagnostics.motor_flags);

    std::ostringstream meta;
    meta << "{\"q\":" << ndarray_meta(q_ref) << ",\"dq\":" << ndarray_meta(dq_ref)
         << ",\"tau\":" << ndarray_meta(tau_ref)
         << ",\"tau_latest\":" << ndarray_meta(tau_latest_ref)
         << ",\"quat_wxyz\":" << ndarray_meta(quat_ref) << ",\"gyro\":" << ndarray_meta(gyro_ref)
         << ",\"linacc\":" << ndarray_meta(linacc_ref)
         << ",\"motor_ddq\":" << ndarray_meta(motor_ddq_ref)
         << ",\"motor_temperature_casing\":" << ndarray_meta(casing_temperature_ref)
         << ",\"motor_temperature_winding\":" << ndarray_meta(winding_temperature_ref)
         << ",\"motor_voltage\":" << ndarray_meta(motor_voltage_ref)
         << ",\"motor_mode\":" << ndarray_meta(motor_mode_ref, "<u4")
         << ",\"motor_sensor_0\":" << ndarray_meta(motor_sensor_0_ref, "<u4")
         << ",\"motor_sensor_1\":" << ndarray_meta(motor_sensor_1_ref, "<u4")
         << ",\"motor_state\":" << ndarray_meta(motor_state_ref, "<u4")
         << ",\"motor_reserve_0\":" << ndarray_meta(motor_reserve_0_ref, "<u4")
         << ",\"motor_reserve_1\":" << ndarray_meta(motor_reserve_1_ref, "<u4")
         << ",\"motor_reserve_2\":" << ndarray_meta(motor_reserve_2_ref, "<u4")
         << ",\"motor_reserve_3\":" << ndarray_meta(motor_reserve_3_ref, "<u4")
         << ",\"motor_diagnostic_flags\":" << ndarray_meta(motor_diagnostic_flags_ref, "<u4")
         << ",\"motor_diagnostic_flag_schema\":1"
         << ",\"diagnostic_warning_count\":" << diagnostics.warning_count
         << ",\"diagnostic_critical_count\":" << diagnostics.critical_count
         << ",\"diagnostic_imu_flags\":" << diagnostics.imu_flags
         << ",\"mode_machine\":" << static_cast<uint32_t>(mode_machine)
         << ",\"buttons\":{"
         << "\"start\":" << bool_text(remote.start) << ",\"stop\":" << bool_text(remote.stop)
         << ",\"A\":" << bool_text(remote.a) << ",\"up\":" << bool_text(remote.up)
         << ",\"down\":" << bool_text(remote.down) << "},\"sticks\":{";
    meta << "\"lx\":";
    append_float_json(meta, remote.lx);
    meta << ",\"ly\":";
    append_float_json(meta, remote.ly);
    meta << ",\"rx\":";
    append_float_json(meta, remote.rx);
    meta << ",\"ry\":";
    append_float_json(meta, remote.ry);
    meta << "},\"state_receive_time_ns\":" << now_ns() << "}";

    uint64_t seq = 0;
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      seq = seq_++;
    }
    const std::vector<uint8_t> packet = encode_datagram(meta.str(), payload, seq);
    const ssize_t sent =
        ::sendto(fd_, packet.data(), packet.size(), 0, reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
    if (sent < 0 || static_cast<size_t>(sent) != packet.size()) {
      throw std::runtime_error(errno_text("sendto() failed"));
    }
  }

 private:
  int fd_ = -1;
  sockaddr_in target_{};
  uint64_t seq_ = 0;
  std::mutex send_mutex_;
};

struct LatestPacket {
  uint64_t seq = 0;
  uint64_t send_time_ns = 0;
  uint64_t recv_time_ns = 0;
  YAML::Node data;
  std::vector<uint8_t> payload;
};

LatestPacket decode_datagram(const std::vector<uint8_t> & packet)
{
  if (packet.size() < kHeaderSize + kCrcSize) {
    throw std::runtime_error("packet too short");
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), packet.begin())) {
    throw std::runtime_error("bad magic");
  }
  if (packet[4] != kVersion) {
    throw std::runtime_error("unsupported version");
  }
  const uint64_t seq = read_u64_be(packet.data() + 8);
  const uint64_t send_time_ns = read_u64_be(packet.data() + 16);
  const uint32_t meta_len = read_u32_be(packet.data() + 24);
  const uint32_t payload_len = read_u32_be(packet.data() + 28);
  const size_t expected = kHeaderSize + kCrcSize + static_cast<size_t>(meta_len) + static_cast<size_t>(payload_len);
  if (packet.size() != expected) {
    throw std::runtime_error("packet length mismatch");
  }

  const uint32_t recv_crc = read_u32_be(packet.data() + kHeaderSize);
  uLong crc = crc32(0, packet.data(), static_cast<uInt>(kHeaderSize));
  crc = crc32(crc, packet.data() + kHeaderSize + kCrcSize, static_cast<uInt>(meta_len + payload_len));
  if (static_cast<uint32_t>(crc & 0xffffffffU) != recv_crc) {
    throw std::runtime_error("crc mismatch");
  }

  const char * meta_begin = reinterpret_cast<const char *>(packet.data() + kHeaderSize + kCrcSize);
  LatestPacket out;
  out.seq = seq;
  out.send_time_ns = send_time_ns;
  out.data = YAML::Load(std::string(meta_begin, meta_begin + meta_len));
  out.payload.assign(packet.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + kCrcSize + meta_len), packet.end());
  return out;
}

std::vector<double> read_array_as_double(const YAML::Node & root, const std::vector<uint8_t> & payload, const std::string & key)
{
  const YAML::Node meta = root[key];
  if (!meta || !meta.IsMap() || !meta[kTypeKey] || meta[kTypeKey].as<std::string>() != "ndarray") {
    throw std::runtime_error("Expected ndarray for key: " + key);
  }
  const std::string dtype = meta["dtype"].as<std::string>();
  const size_t offset = meta["offset"].as<size_t>();
  const size_t nbytes = meta["nbytes"].as<size_t>();
  const YAML::Node shape = meta["shape"];
  if (!shape || !shape.IsSequence()) {
    throw std::runtime_error("Missing ndarray shape for key: " + key);
  }
  size_t count = 1;
  for (const auto & dim : shape) {
    const size_t dim_count = dim.as<size_t>();
    if (dim_count != 0 && count > std::numeric_limits<size_t>::max() / dim_count) {
      throw std::runtime_error("ndarray shape overflows size_t for key: " + key);
    }
    count *= dim_count;
  }
  if (offset > payload.size() || nbytes > payload.size() - offset) {
    throw std::runtime_error("ndarray payload out of range: " + key);
  }
  std::vector<double> out(count, 0.0);
  if (dtype == "<f4" || dtype == "=f4" || dtype == "|f4") {
    if (nbytes != count * sizeof(float)) {
      throw std::runtime_error("Unexpected float32 nbytes for key: " + key);
    }
    for (size_t i = 0; i < count; ++i) {
      float value = 0.0f;
      std::memcpy(&value, payload.data() + offset + i * sizeof(float), sizeof(float));
      out[i] = static_cast<double>(value);
    }
    return out;
  }
  if (dtype == "<f8" || dtype == "=f8" || dtype == "|f8") {
    if (nbytes != count * sizeof(double)) {
      throw std::runtime_error("Unexpected float64 nbytes for key: " + key);
    }
    for (size_t i = 0; i < count; ++i) {
      double value = 0.0;
      std::memcpy(&value, payload.data() + offset + i * sizeof(double), sizeof(double));
      out[i] = value;
    }
    return out;
  }
  throw std::runtime_error("Unsupported ndarray dtype for key " + key + ": " + dtype);
}

std::optional<uint64_t> yaml_u64_optional(const YAML::Node & node)
{
  if (!node) {
    return std::nullopt;
  }
  try {
    const int64_t signed_value = node.as<int64_t>();
    if (signed_value < 0) {
      return std::nullopt;
    }
    return static_cast<uint64_t>(signed_value);
  } catch (const std::exception &) {
  }
  try {
    const double value = node.as<double>();
    if (!std::isfinite(value) || value < 0.0) {
      return std::nullopt;
    }
    return static_cast<uint64_t>(value);
  } catch (const std::exception &) {
  }
  return std::nullopt;
}

class UdpLatestReceiver {
 public:
  using Callback = std::function<void(const LatestPacket &)>;

  UdpLatestReceiver(
      const std::string & host, int port, const std::string & allowed_host,
      int recvbuf_bytes, Callback callback)
      : callback_(std::move(callback))
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error(errno_text("socket(AF_INET, SOCK_DGRAM) failed"));
    }
    const int reuse = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (recvbuf_bytes > 0) {
      ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &recvbuf_bytes, sizeof(recvbuf_bytes));
    }

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<uint16_t>(port));
    if (host.empty() || host == "0.0.0.0") {
      bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, host.c_str(), &bind_addr.sin_addr) != 1) {
      throw std::runtime_error("Invalid UDP bind host: " + host);
    }
    if (::bind(fd_, reinterpret_cast<const sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0) {
      throw std::runtime_error(errno_text("bind() failed"));
    }
    if (!allowed_host.empty()) {
      if (::inet_pton(AF_INET, allowed_host.c_str(), &allowed_addr_) != 1) {
        throw std::runtime_error("Invalid UDP allowed host: " + allowed_host);
      }
      filter_source_ = true;
    }
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw std::runtime_error(errno_text("fcntl(O_NONBLOCK) failed"));
    }
  }

  ~UdpLatestReceiver()
  {
    close();
  }

  UdpLatestReceiver(const UdpLatestReceiver &) = delete;
  UdpLatestReceiver & operator=(const UdpLatestReceiver &) = delete;

  void start()
  {
    thread_ = std::thread([this]() { recv_loop(); });
  }

  void close()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  uint64_t packets_received() const
  {
    return packets_received_.load(std::memory_order_relaxed);
  }

  uint64_t packets_decoded() const
  {
    return packets_decoded_.load(std::memory_order_relaxed);
  }

  uint64_t decode_errors() const
  {
    return decode_errors_.load(std::memory_order_relaxed);
  }

  uint64_t source_rejections() const
  {
    return source_rejections_.load(std::memory_order_relaxed);
  }

 private:
  void recv_loop()
  {
    while (!stop_.load()) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd_, &read_fds);
      timeval tv{};
      tv.tv_usec = 50000;
      const int ready = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
      if (ready <= 0 || !FD_ISSET(fd_, &read_fds)) {
        continue;
      }

      std::vector<uint8_t> last_packet;
      std::array<uint8_t, 65535> buffer{};
      uint64_t recv_time = 0;
      while (true) {
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t n =
            ::recvfrom(fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
        if (n < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
          }
          break;
        }
        packets_received_.fetch_add(1, std::memory_order_relaxed);
        if (filter_source_ && from.sin_addr.s_addr != allowed_addr_.s_addr) {
          source_rejections_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
        last_packet.assign(buffer.begin(), buffer.begin() + n);
        recv_time = now_ns();
      }

      if (last_packet.empty()) {
        continue;
      }
      try {
        LatestPacket decoded = decode_datagram(last_packet);
        decoded.recv_time_ns = recv_time;
        packets_decoded_.fetch_add(1, std::memory_order_relaxed);
        callback_(decoded);
      } catch (const std::exception & exc) {
        decode_errors_.fetch_add(1, std::memory_order_relaxed);
        std::cerr << "[G1Bridge] UDP decode error: " << exc.what() << std::endl;
      }
    }
  }

  int fd_ = -1;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> packets_received_{0};
  std::atomic<uint64_t> packets_decoded_{0};
  std::atomic<uint64_t> decode_errors_{0};
  std::atomic<uint64_t> source_rejections_{0};
  bool filter_source_ = false;
  in_addr allowed_addr_{};
  std::thread thread_;
  Callback callback_;
};

timespec duration_to_timespec(std::chrono::nanoseconds duration)
{
  if (duration.count() <= 0) {
    throw std::runtime_error("timer period must be positive");
  }
  constexpr int64_t kNsPerSecond = 1000000000LL;
  timespec out{};
  out.tv_sec = static_cast<time_t>(duration.count() / kNsPerSecond);
  out.tv_nsec = static_cast<long>(duration.count() % kNsPerSecond);
  return out;
}

int create_periodic_timer_fd(std::chrono::nanoseconds period)
{
  const int fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error(errno_text("timerfd_create(CLOCK_MONOTONIC) failed"));
  }

  itimerspec spec{};
  spec.it_interval = duration_to_timespec(period);
  spec.it_value.tv_nsec = 1;  // Start immediately, then let the kernel keep the periodic cadence.
  if (::timerfd_settime(fd, 0, &spec, nullptr) < 0) {
    const std::string error = errno_text("timerfd_settime() failed");
    ::close(fd);
    throw std::runtime_error(error);
  }
  return fd;
}

class G1UdpBridge {
 public:
  G1UdpBridge(BridgeConfig cfg, std::string network_interface)
      : cfg_(std::move(cfg)),
        network_interface_(std::move(network_interface)),
        policy_index_(build_index_map(cfg_.policy_joint_names)),
        real_index_(build_index_map(cfg_.real_joint_names)),
        real_to_policy_(build_source_indices(cfg_.policy_joint_names, real_index_, "real_to_policy map")),
        policy_to_real_(build_source_indices(cfg_.real_joint_names, policy_index_, "policy_to_real map")),
        state_sender_(cfg_.udp.state_host, cfg_.udp.state_port, cfg_.udp.sndbuf_bytes)
  {
    if (cfg_.real_joint_names.size() != static_cast<size_t>(kG1MotorCount)) {
      std::cerr << "[G1Bridge] Warning: real_joint_names size=" << cfg_.real_joint_names.size()
                << ", G1 examples use 29 motors" << std::endl;
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
    release_motion_service();

    lowcmd_publisher_.reset(new unitree::robot::ChannelPublisher<LowCmd>(cfg_.lowcmd_topic));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new unitree::robot::ChannelSubscriber<LowState>(cfg_.lowstate_topic));
    if (cfg_.freq.state_publish_mode == StatePublishMode::LowStateTick) {
      lowstate_subscriber_->InitChannel(
          std::bind(&G1UdpBridge::on_lowstate, this, std::placeholders::_1), 0);
    } else {
      lowstate_subscriber_->InitChannel();
    }

    wait_for_lowstate();
    if (cfg_.safety.enabled && cfg_.safety.startup_damping) {
      std::cout << "[G1Bridge] Startup damping active while waiting for "
                   "the first valid command"
                << std::endl;
      publish_damping_command(
          /*log=*/false, /*require_startup_pending=*/true);
    }
    start_state_sender_thread();

    command_receiver_.reset(new UdpLatestReceiver(
        cfg_.udp.cmd_bind_host, cfg_.udp.cmd_port, cfg_.udp.cmd_allowed_host,
        cfg_.udp.recvbuf_bytes,
        [this](const LatestPacket & packet) { on_udp_command(packet); }));
    command_receiver_->start();
    start_command_watchdog();

    std::cout << "[G1Bridge] endpoints: state=" << cfg_.udp.state_host << ":" << cfg_.udp.state_port
              << " cmd_bind=" << cfg_.udp.cmd_bind_host << ":" << cfg_.udp.cmd_port
              << " cmd_allowed="
              << (cfg_.udp.cmd_allowed_host.empty() ? "<any>" : cfg_.udp.cmd_allowed_host)
              << std::endl;
    std::cout << "[G1Bridge] freq: physical_hz=" << cfg_.freq.physical_hz
              << " state_decimation=" << cfg_.freq.state_decimation
              << " state_publish_mode=" << state_publish_mode_name(cfg_.freq.state_publish_mode)
              << " state_hz=" << cfg_.freq.physical_hz / static_cast<double>(cfg_.freq.state_decimation)
              << std::endl;
    std::cout << "[G1Bridge] command mode: event-driven UDP callback -> DDS Write, mode_pr="
              << static_cast<int>(cfg_.low_level.mode_pr) << std::endl;
    if (cfg_.safety.enabled) {
      std::cout << "[G1Bridge] task safety: command_timeout_monitor=" << cfg_.safety.command_timeout_s
                << "s timeout_action=warn-only"
                << " startup_damping=" << (cfg_.safety.startup_damping ? "on" : "off")
                << " damping_hz=" << cfg_.safety.damping_publish_hz
                << std::endl;
    }
    if (cfg_.diagnostics.enabled) {
      std::cout << "[MotorDiag] enabled action=report-only"
                << " report_interval=" << cfg_.diagnostics.report_interval_s << "s"
                << " casing_warn/limit=" << cfg_.diagnostics.motor_casing_warn_c << "/"
                << cfg_.diagnostics.motor_casing_limit_c << "C"
                << " winding_warn/limit=" << cfg_.diagnostics.motor_winding_warn_c << "/"
                << cfg_.diagnostics.motor_winding_limit_c << "C"
                << " joint_dq_warn/limit=" << cfg_.diagnostics.joint_velocity_warn_rad_s << "/"
                << cfg_.diagnostics.joint_velocity_limit_rad_s << "rad/s"
                << " imu_gyro_warn/limit=" << cfg_.diagnostics.imu_angular_velocity_warn_rad_s << "/"
                << cfg_.diagnostics.imu_angular_velocity_limit_rad_s << "rad/s"
                << " position_check=" << (cfg_.diagnostics.check_joint_position ? "on" : "off")
                << " torque_check=" << (cfg_.diagnostics.check_joint_torque ? "on" : "off")
                << std::endl;
    } else {
      std::cout << "[MotorDiag] checks disabled; raw motor telemetry is still forwarded" << std::endl;
    }
    if (cfg_.freq.state_publish_mode == StatePublishMode::LowStateTick) {
      std::cout << "[G1Bridge] state mode: LowState.tick target decimation -> dedicated UDP state sender thread"
                << std::endl;
    } else {
      std::cout << "[G1Bridge] state mode: timerfd periodic sender actively reads LowState with no callback"
                << std::endl;
    }
    start_stdin_button_thread();
  }

  ~G1UdpBridge()
  {
    close();
  }

  G1UdpBridge(const G1UdpBridge &) = delete;
  G1UdpBridge & operator=(const G1UdpBridge &) = delete;

  void run()
  {
    auto next_log_time = SteadyClock::now() + std::chrono::seconds(1);
    while (!g_stop_requested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const auto now = SteadyClock::now();
      if (now >= next_log_time) {
        log_rates();
        next_log_time = now + std::chrono::seconds(1);
      }
    }
  }

  void close()
  {
    if (closed_.exchange(true)) {
      return;
    }
    if (command_receiver_) {
      command_receiver_->close();
    }
    stop_command_watchdog();
    stop_stdin_button_thread();
    stop_state_sender_thread();
    lowstate_subscriber_.reset();
    publish_damping_command();
    state_sender_.close();
    lowcmd_publisher_.reset();
    unitree::robot::ChannelFactory::Instance()->Release();
  }

 private:
  void release_motion_service()
  {
    if (!cfg_.low_level.release_motion_service) {
      std::cout << "[G1Bridge] Motion service release disabled by config" << std::endl;
      return;
    }

    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(5.0f);
    msc.Init();

    for (int attempt = 1; attempt <= cfg_.low_level.release_max_attempts; ++attempt) {
      std::string form;
      std::string name;
      const int32_t check_ret = msc.CheckMode(form, name);
      if (check_ret != 0) {
        const std::string msg = "MotionSwitcher CheckMode failed ret=" + std::to_string(check_ret);
        if (cfg_.low_level.release_required) {
          throw std::runtime_error(msg);
        }
        std::cerr << "[G1Bridge] Warning: " << msg << std::endl;
        return;
      }
      if (name.empty()) {
        std::cout << "[G1Bridge] Motion control service is already released" << std::endl;
        return;
      }
      std::cout << "[G1Bridge] Active motion service form=" << form << " name=" << name
                << ", ReleaseMode attempt " << attempt << "/" << cfg_.low_level.release_max_attempts << std::endl;
      const int32_t release_ret = msc.ReleaseMode();
      if (release_ret != 0) {
        std::cerr << "[G1Bridge] ReleaseMode failed ret=" << release_ret << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(cfg_.low_level.release_sleep_s));
    }

    std::string form;
    std::string name;
    const int32_t check_ret = msc.CheckMode(form, name);
    if (check_ret == 0 && name.empty()) {
      std::cout << "[G1Bridge] Motion control service released" << std::endl;
      return;
    }
    const std::string msg = "Motion control service is still active after release attempts";
    if (cfg_.low_level.release_required) {
      throw std::runtime_error(msg);
    }
    std::cerr << "[G1Bridge] Warning: " << msg << std::endl;
  }

  void wait_for_lowstate()
  {
    if (cfg_.freq.state_publish_mode == StatePublishMode::Timer) {
      wait_for_lowstate_reader();
    } else {
      wait_for_lowstate_callback();
    }
    std::cout << "[G1Bridge] Successfully connected to DDS lowstate, mode_machine="
              << static_cast<int>(mode_machine_.load()) << std::endl;
  }

  void wait_for_lowstate_callback()
  {
    std::unique_lock<std::mutex> lock(first_state_mutex_);
    const auto ready = [this]() { return have_lowstate_.load(); };
    if (cfg_.low_level.wait_lowstate_timeout_s <= 0.0) {
      while (!ready()) {
        if (g_stop_requested.load()) {
          throw std::runtime_error("Interrupted while waiting for first valid DDS lowstate");
        }
        first_state_cv_.wait_for(lock, std::chrono::milliseconds(100));
      }
    } else {
      const auto deadline = SteadyClock::now() + std::chrono::duration<double>(cfg_.low_level.wait_lowstate_timeout_s);
      while (!ready()) {
        if (g_stop_requested.load()) {
          throw std::runtime_error("Interrupted while waiting for first valid DDS lowstate");
        }
        if (first_state_cv_.wait_until(lock, deadline, ready)) {
          break;
        }
        if (SteadyClock::now() >= deadline) {
          throw std::runtime_error("Timed out waiting for first valid DDS lowstate");
        }
      }
    }
  }

  void wait_for_lowstate_reader()
  {
    const auto start = SteadyClock::now();
    while (!g_stop_requested.load()) {
      if (poll_lowstate_reader(/*allow_publish=*/false)) {
        return;
      }
      if (cfg_.low_level.wait_lowstate_timeout_s > 0.0) {
        const double elapsed = std::chrono::duration<double>(SteadyClock::now() - start).count();
        if (elapsed >= cfg_.low_level.wait_lowstate_timeout_s) {
          throw std::runtime_error("Timed out waiting for first valid DDS lowstate");
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("Interrupted while waiting for first valid DDS lowstate");
  }

  struct TickDecision {
    bool unique = false;
    bool publish_now = false;
  };

  void on_lowstate(const void * message)
  {
    if (message == nullptr || fatal_shutdown_requested_.load(std::memory_order_relaxed) ||
        g_stop_requested.load(std::memory_order_relaxed)) {
      return;
    }
    const LowState low_state = *(const LowState *)message;
    lowstate_callback_count_.fetch_add(1, std::memory_order_relaxed);
    process_lowstate_sample(low_state, /*allow_tick_publish=*/true);
  }

  bool poll_lowstate_reader(bool allow_publish)
  {
    LowState low_state;
    if (!read_lowstate_from_subscriber(low_state)) {
      return false;
    }
    return process_lowstate_sample(low_state, allow_publish);
  }

  bool read_lowstate_from_subscriber(LowState & low_state)
  {
    if (!lowstate_subscriber_) {
      return false;
    }
    if (!lowstate_subscriber_->ReadLatest(low_state)) {
      return false;
    }
    lowstate_read_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  bool process_lowstate_sample(
      const LowState & low_state, bool allow_tick_publish, DiagnosticSnapshot * diagnostic_out = nullptr)
  {
    if (fatal_shutdown_requested_.load(std::memory_order_relaxed) ||
        g_stop_requested.load(std::memory_order_relaxed)) {
      return false;
    }
    if (low_state.crc() !=
        crc32_core((uint32_t *)&low_state, (static_cast<uint32_t>(sizeof(LowState)) >> 2) - 1)) {
      lowstate_crc_error_count_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    const uint8_t observed_mode_machine = low_state.mode_machine();
    const bool had_mode = have_mode_machine_.exchange(true);
    const uint8_t prev_mode = mode_machine_.exchange(observed_mode_machine);
    if (!had_mode || prev_mode != observed_mode_machine) {
      std::cout << "[G1Bridge] mode_machine=" << static_cast<int>(observed_mode_machine) << std::endl;
    }

    const TickDecision tick_decision = update_lowstate_tick(low_state.tick());
    if (!tick_decision.unique) {
      return false;
    }
    DiagnosticSnapshot diagnostics = evaluate_diagnostics(low_state);
    if (diagnostic_out != nullptr) {
      *diagnostic_out = diagnostics;
    }
    accumulate_joint_torque(low_state);

    if (!have_lowstate_.load(std::memory_order_relaxed)) {
      {
        std::lock_guard<std::mutex> lock(first_state_mutex_);
        have_lowstate_.store(true);
      }
      first_state_cv_.notify_all();
    }

    if (allow_tick_publish && tick_decision.publish_now) {
      enqueue_state_snapshot(low_state, consume_joint_torque_average(), std::move(diagnostics));
    }
    return true;
  }

  static bool tick_reached_or_passed(uint32_t observed_tick, uint32_t target_tick)
  {
    return static_cast<int32_t>(observed_tick - target_tick) >= 0;
  }

  TickDecision update_lowstate_tick(uint32_t observed_tick)
  {
    std::lock_guard<std::mutex> lock(lowstate_tick_mutex_);
    TickDecision decision;
    bool reset_schedule = false;
    if (last_lowstate_tick_) {
      if (*last_lowstate_tick_ == observed_tick) {
        lowstate_duplicate_tick_count_.fetch_add(1, std::memory_order_relaxed);
        return decision;
      }

      const uint32_t delta = observed_tick - *last_lowstate_tick_;
      if (delta > 1) {
        if (delta < 0x80000000U) {
          if (cfg_.freq.state_publish_mode == StatePublishMode::LowStateTick) {
            lowstate_tick_gap_count_.fetch_add(1, std::memory_order_relaxed);
            lowstate_tick_missing_count_.fetch_add(static_cast<uint64_t>(delta - 1), std::memory_order_relaxed);
          }
        } else {
          // Large modulo deltas usually mean a producer reset or backward jump, not billions of missed ticks.
          lowstate_tick_reset_count_.fetch_add(1, std::memory_order_relaxed);
          reset_schedule = true;
        }
      }
    }

    last_lowstate_tick_ = observed_tick;
    lowstate_count_.fetch_add(1, std::memory_order_relaxed);
    lowstate_unique_tick_.fetch_add(1, std::memory_order_relaxed);
    decision.unique = true;

    if (cfg_.freq.state_publish_mode == StatePublishMode::Timer) {
      return decision;
    }

    if (!next_state_tick_ || reset_schedule) {
      next_state_tick_ = observed_tick + static_cast<uint32_t>(cfg_.freq.state_decimation);
      decision.publish_now = true;
      return decision;
    }
    if (!tick_reached_or_passed(observed_tick, *next_state_tick_)) {
      return decision;
    }
    const uint32_t fatal_sync_tick = *next_state_tick_ + static_cast<uint32_t>(cfg_.freq.state_decimation);
    if (tick_reached_or_passed(observed_tick, fatal_sync_tick)) {
      request_fatal_shutdown(observed_tick, *next_state_tick_, fatal_sync_tick);
      return decision;
    }

    *next_state_tick_ += static_cast<uint32_t>(cfg_.freq.state_decimation);
    decision.publish_now = true;
    return decision;
  }

  void request_fatal_shutdown(uint32_t observed_tick, uint32_t next_state_tick, uint32_t fatal_sync_tick)
  {
    if (!fatal_shutdown_requested_.exchange(true, std::memory_order_relaxed)) {
      std::cerr << "[G1Bridge] Fatal lowstate tick sync error: observed_tick=" << observed_tick
                << " next_state_tick=" << next_state_tick << " fatal_threshold_tick=" << fatal_sync_tick
                << " state_decimation=" << cfg_.freq.state_decimation
                << ". Stopping bridge instead of publishing stale/catch-up state." << std::endl;
    }
    g_stop_requested.store(true, std::memory_order_relaxed);
  }

  static std::string motor_diagnostic_flag_names(uint32_t flags)
  {
    const std::array<std::pair<uint32_t, const char *>, 12> names{{
        {kMotorStateFault, "motor_state_fault"},
        {kCasingTemperatureWarning, "casing_temperature_warning"},
        {kCasingTemperatureLimit, "casing_temperature_limit"},
        {kWindingTemperatureWarning, "winding_temperature_warning"},
        {kWindingTemperatureLimit, "winding_temperature_limit"},
        {kJointPositionWarning, "joint_position_warning"},
        {kJointPositionLimit, "joint_position_limit"},
        {kJointVelocityWarning, "joint_velocity_warning"},
        {kJointVelocityLimit, "joint_velocity_limit"},
        {kJointTorqueWarning, "joint_torque_warning"},
        {kJointTorqueLimit, "joint_torque_limit"},
        {kMotorStateNonFinite, "non_finite_motor_feedback"},
    }};
    std::ostringstream out;
    bool first = true;
    for (const auto & item : names) {
      if ((flags & item.first) == 0) {
        continue;
      }
      if (!first) {
        out << ",";
      }
      out << item.second;
      first = false;
    }
    return out.str();
  }

  static std::string imu_diagnostic_flag_names(uint32_t flags)
  {
    std::ostringstream out;
    if ((flags & kImuAngularVelocityWarning) != 0) {
      out << "imu_angular_velocity_warning";
    }
    if ((flags & kImuAngularVelocityLimit) != 0) {
      if (out.tellp() > 0) {
        out << ",";
      }
      out << "imu_angular_velocity_limit";
    }
    if ((flags & kImuStateNonFinite) != 0) {
      if (out.tellp() > 0) {
        out << ",";
      }
      out << "non_finite_imu_feedback";
    }
    return out.str();
  }

  DiagnosticSnapshot evaluate_diagnostics(const LowState & low_state)
  {
    DiagnosticSnapshot snapshot;
    snapshot.motor_flags.assign(cfg_.real_joint_names.size(), 0U);
    if (!cfg_.diagnostics.enabled) {
      return snapshot;
    }

    const auto & motors = low_state.motor_state();
    float max_casing = -std::numeric_limits<float>::infinity();
    float max_winding = -std::numeric_limits<float>::infinity();
    float max_abs_velocity = 0.0f;
    float max_abs_torque = 0.0f;
    size_t max_casing_index = 0;
    size_t max_winding_index = 0;
    size_t max_velocity_index = 0;
    size_t max_torque_index = 0;

    for (size_t i = 0; i < cfg_.real_joint_names.size(); ++i) {
      const auto & motor = motors.at(i);
      uint32_t flags = 0;
      const float q = motor.q();
      const float dq = motor.dq();
      const float ddq = motor.ddq();
      const float tau = motor.tau_est();
      const float voltage = motor.vol();
      const float casing = static_cast<float>(motor.temperature()[0]);
      const float winding = static_cast<float>(motor.temperature()[1]);

      if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(ddq) ||
          !std::isfinite(tau) || !std::isfinite(voltage)) {
        flags |= kMotorStateNonFinite;
      }
      if (motor.motorstate() != 0U) {
        flags |= kMotorStateFault;
      }
      if (casing > cfg_.diagnostics.motor_casing_limit_c) {
        flags |= kCasingTemperatureLimit;
      } else if (casing > cfg_.diagnostics.motor_casing_warn_c) {
        flags |= kCasingTemperatureWarning;
      }
      if (winding > cfg_.diagnostics.motor_winding_limit_c) {
        flags |= kWindingTemperatureLimit;
      } else if (winding > cfg_.diagnostics.motor_winding_warn_c) {
        flags |= kWindingTemperatureWarning;
      }

      if (cfg_.diagnostics.check_joint_position && std::isfinite(q)) {
        const double lower = cfg_.diagnostics.joint_position_lower.at(i);
        const double upper = cfg_.diagnostics.joint_position_upper.at(i);
        if (q < lower || q > upper) {
          flags |= kJointPositionLimit;
        } else if (q < lower + cfg_.diagnostics.joint_position_warn_margin_rad ||
                   q > upper - cfg_.diagnostics.joint_position_warn_margin_rad) {
          flags |= kJointPositionWarning;
        }
      }

      const float abs_velocity = std::fabs(dq);
      if (std::isfinite(abs_velocity)) {
        if (abs_velocity > cfg_.diagnostics.joint_velocity_limit_rad_s) {
          flags |= kJointVelocityLimit;
        } else if (abs_velocity > cfg_.diagnostics.joint_velocity_warn_rad_s) {
          flags |= kJointVelocityWarning;
        }
      }

      const float abs_torque = std::fabs(tau);
      if (cfg_.diagnostics.check_joint_torque && std::isfinite(abs_torque)) {
        const double limit = cfg_.diagnostics.joint_torque_limit.at(i);
        if (abs_torque > limit) {
          flags |= kJointTorqueLimit;
        } else if (abs_torque > limit * cfg_.diagnostics.joint_torque_warn_ratio) {
          flags |= kJointTorqueWarning;
        }
      }

      snapshot.motor_flags[i] = flags;
      if ((flags & kMotorCriticalMask) != 0) {
        ++snapshot.critical_count;
      } else if ((flags & kMotorWarningMask) != 0) {
        ++snapshot.warning_count;
      }
      if (casing > max_casing) {
        max_casing = casing;
        max_casing_index = i;
      }
      if (winding > max_winding) {
        max_winding = winding;
        max_winding_index = i;
      }
      if (abs_velocity > max_abs_velocity) {
        max_abs_velocity = abs_velocity;
        max_velocity_index = i;
      }
      if (abs_torque > max_abs_torque) {
        max_abs_torque = abs_torque;
        max_torque_index = i;
      }
    }

    float max_abs_imu_angular_velocity = 0.0f;
    for (const float value : low_state.imu_state().gyroscope()) {
      if (!std::isfinite(value)) {
        snapshot.imu_flags |= kImuStateNonFinite;
        continue;
      }
      max_abs_imu_angular_velocity = std::max(max_abs_imu_angular_velocity, std::fabs(value));
    }
    if (max_abs_imu_angular_velocity > cfg_.diagnostics.imu_angular_velocity_limit_rad_s) {
      snapshot.imu_flags |= kImuAngularVelocityLimit;
    } else if (max_abs_imu_angular_velocity > cfg_.diagnostics.imu_angular_velocity_warn_rad_s) {
      snapshot.imu_flags |= kImuAngularVelocityWarning;
    }
    if ((snapshot.imu_flags & kImuCriticalMask) != 0) {
      ++snapshot.critical_count;
    } else if ((snapshot.imu_flags & kImuWarningMask) != 0) {
      ++snapshot.warning_count;
    }

    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    if (previous_motor_diagnostic_flags_.size() != snapshot.motor_flags.size()) {
      previous_motor_diagnostic_flags_.assign(snapshot.motor_flags.size(), 0U);
      previous_motor_state_codes_.assign(snapshot.motor_flags.size(), 0U);
    }
    for (size_t i = 0; i < snapshot.motor_flags.size(); ++i) {
      const uint32_t flags = snapshot.motor_flags[i];
      const uint32_t motor_state_code = motors.at(i).motorstate();
      if (flags == previous_motor_diagnostic_flags_[i] &&
          motor_state_code == previous_motor_state_codes_[i]) {
        continue;
      }
      const auto & motor = motors.at(i);
      if (flags == 0U) {
        std::cout << "[MotorDiag][RECOVERED] joint=" << cfg_.real_joint_names[i] << std::endl;
      } else {
        const bool critical = (flags & kMotorCriticalMask) != 0;
        std::ostream & stream = critical ? std::cerr : std::cout;
        if (critical) {
          stream << kAnsiBoldRed;
        }
        stream << "[MotorDiag][" << (critical ? "CRITICAL" : "WARNING") << "] joint="
               << cfg_.real_joint_names[i] << " flags=" << motor_diagnostic_flag_names(flags)
               << " motorstate=0x" << std::hex << motor.motorstate() << std::dec
               << " q=" << motor.q() << "rad dq=" << motor.dq() << "rad/s tau_est="
               << motor.tau_est() << "Nm casing=" << motor.temperature()[0]
               << "C winding=" << motor.temperature()[1] << "C mode="
               << static_cast<uint32_t>(motor.mode());
        if (critical) {
          stream << kAnsiReset;
        }
        stream << std::endl;
      }
    }
    if (snapshot.imu_flags != previous_imu_diagnostic_flags_) {
      if (snapshot.imu_flags == 0U) {
        std::cout << "[MotorDiag][RECOVERED] imu angular velocity" << std::endl;
      } else {
        const bool critical = (snapshot.imu_flags & kImuCriticalMask) != 0;
        std::ostream & stream = critical ? std::cerr : std::cout;
        if (critical) {
          stream << kAnsiBoldRed;
        }
        stream << "[MotorDiag][" << (critical ? "CRITICAL" : "WARNING")
               << "] flags=" << imu_diagnostic_flag_names(snapshot.imu_flags)
               << " max_abs_gyro=" << max_abs_imu_angular_velocity << "rad/s";
        if (critical) {
          stream << kAnsiReset;
        }
        stream << std::endl;
      }
    }

    previous_motor_diagnostic_flags_ = snapshot.motor_flags;
    for (size_t i = 0; i < previous_motor_state_codes_.size(); ++i) {
      previous_motor_state_codes_[i] = motors.at(i).motorstate();
    }
    previous_imu_diagnostic_flags_ = snapshot.imu_flags;
    const auto now = SteadyClock::now();
    if (!next_diagnostic_report_time_ || now >= *next_diagnostic_report_time_) {
      std::cout << "[MotorDiag] status="
                << (snapshot.critical_count > 0 ? "CRITICAL" : (snapshot.warning_count > 0 ? "WARNING" : "OK"))
                << " warning_sources=" << snapshot.warning_count
                << " critical_sources=" << snapshot.critical_count
                << " max_casing=" << max_casing << "C(" << cfg_.real_joint_names[max_casing_index] << ")"
                << " max_winding=" << max_winding << "C(" << cfg_.real_joint_names[max_winding_index] << ")"
                << " max_abs_dq=" << max_abs_velocity << "rad/s(" << cfg_.real_joint_names[max_velocity_index]
                << ") max_abs_tau=" << max_abs_torque << "Nm(" << cfg_.real_joint_names[max_torque_index]
                << ") max_abs_gyro=" << max_abs_imu_angular_velocity << "rad/s"
                << " mode_machine=" << static_cast<uint32_t>(low_state.mode_machine()) << std::endl;
      next_diagnostic_report_time_ = now + std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(cfg_.diagnostics.report_interval_s));
    }
    return snapshot;
  }

  void accumulate_joint_torque(const LowState & low_state)
  {
    std::lock_guard<std::mutex> lock(torque_accumulator_mutex_);
    if (torque_sum_real_.size() != cfg_.real_joint_names.size()) {
      torque_sum_real_.assign(cfg_.real_joint_names.size(), 0.0);
      torque_sample_count_ = 0;
    }
    for (size_t index = 0; index < cfg_.real_joint_names.size(); ++index) {
      torque_sum_real_[index] += static_cast<double>(low_state.motor_state().at(index).tau_est());
    }
    ++torque_sample_count_;
  }

  std::vector<float> consume_joint_torque_average()
  {
    std::lock_guard<std::mutex> lock(torque_accumulator_mutex_);
    std::vector<float> average(cfg_.real_joint_names.size(), 0.0f);
    if (torque_sample_count_ > 0 && torque_sum_real_.size() == average.size()) {
      const double denominator = static_cast<double>(torque_sample_count_);
      for (size_t index = 0; index < average.size(); ++index) {
        average[index] = static_cast<float>(torque_sum_real_[index] / denominator);
      }
    }
    torque_sum_real_.assign(cfg_.real_joint_names.size(), 0.0);
    torque_sample_count_ = 0;
    return average;
  }

  void enqueue_state_snapshot(
      const LowState & low_state, std::vector<float> tau_real_average, DiagnosticSnapshot diagnostics)
  {
    bool should_notify = false;
    {
      std::lock_guard<std::mutex> lock(state_snapshot_mutex_);
      if (state_sender_stop_) {
        return;
      }
      if (pending_state_snapshot_) {
        state_snapshot_overwrite_count_.fetch_add(1, std::memory_order_relaxed);
      }
      pending_state_snapshot_ =
          StateSnapshot{low_state, std::move(tau_real_average), std::move(diagnostics)};
      should_notify = true;
    }
    if (should_notify) {
      state_snapshot_cv_.notify_one();
    }
  }

  void start_stdin_button_thread()
  {
    stdin_button_stop_.store(false, std::memory_order_relaxed);
    stdin_button_thread_ = std::thread([this]() { stdin_button_loop(); });
  }

  void stop_stdin_button_thread()
  {
    stdin_button_stop_.store(true, std::memory_order_relaxed);
    if (stdin_button_thread_.joinable()) {
      stdin_button_thread_.join();
    }
  }

  void stdin_button_loop()
  {
    ScopedTerminalRawMode raw_mode(STDIN_FILENO);
    std::cout << "[G1Bridge] stdin buttons enabled: s=start, a=A, x=stop"
              << (raw_mode.enabled() ? " (single-key tty mode)" : " (line-buffered/pipe mode)") << std::endl;

    while (!stdin_button_stop_.load(std::memory_order_relaxed) &&
           !g_stop_requested.load(std::memory_order_relaxed)) {
      pollfd pfd{};
      pfd.fd = STDIN_FILENO;
      pfd.events = POLLIN;
      const int ready = ::poll(&pfd, 1, 100);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        std::cerr << "[G1Bridge] stdin button poll failed: " << std::strerror(errno) << std::endl;
        return;
      }
      if (ready == 0) {
        continue;
      }
      if ((pfd.revents & POLLIN) == 0) {
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          return;
        }
        continue;
      }

      char buffer[64];
      const ssize_t n = ::read(STDIN_FILENO, buffer, sizeof(buffer));
      if (n == 0) {
        return;
      }
      if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        std::cerr << "[G1Bridge] stdin button read failed: " << std::strerror(errno) << std::endl;
        return;
      }
      for (ssize_t i = 0; i < n; ++i) {
        handle_stdin_button(buffer[i]);
      }
    }
  }

  void handle_stdin_button(char input)
  {
    const char ch = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
    if (ch == 's') {
      pulse_stdin_button(stdin_start_until_ns_, "start");
    } else if (ch == 'a') {
      pulse_stdin_button(stdin_a_until_ns_, "A");
    } else if (ch == 'x') {
      pulse_stdin_button(stdin_stop_until_ns_, "stop");
    }
  }

  void pulse_stdin_button(std::atomic<uint64_t> & until_ns, const char * name)
  {
    until_ns.store(now_ns() + kStdinButtonPulseNs, std::memory_order_relaxed);
    stdin_button_event_count_.fetch_add(1, std::memory_order_relaxed);
    std::cout << "[G1Bridge] stdin button pulse: " << name << std::endl;
  }

  static bool stdin_button_active(const std::atomic<uint64_t> & until_ns, uint64_t now)
  {
    return until_ns.load(std::memory_order_relaxed) > now;
  }

  RemoteState apply_stdin_button_overrides(RemoteState remote) const
  {
    const uint64_t now = now_ns();
    remote.start = remote.start || stdin_button_active(stdin_start_until_ns_, now);
    remote.a = remote.a || stdin_button_active(stdin_a_until_ns_, now);
    remote.stop = remote.stop || stdin_button_active(stdin_stop_until_ns_, now);
    return remote;
  }

  void start_state_sender_thread()
  {
    state_sender_thread_ = std::thread([this]() { state_sender_loop(); });
  }

  void stop_state_sender_thread()
  {
    {
      std::lock_guard<std::mutex> lock(state_snapshot_mutex_);
      state_sender_stop_ = true;
      if (pending_state_snapshot_) {
        pending_state_snapshot_.reset();
        state_snapshot_drop_count_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    state_snapshot_cv_.notify_all();
    if (state_sender_thread_.joinable()) {
      state_sender_thread_.join();
    }
  }

  void state_sender_loop()
  {
    if (cfg_.freq.state_publish_mode == StatePublishMode::Timer) {
      state_sender_timer_loop();
      return;
    }
    state_sender_tick_loop();
  }

  void state_sender_tick_loop()
  {
    while (true) {
      StateSnapshot snapshot;
      {
        std::unique_lock<std::mutex> lock(state_snapshot_mutex_);
        state_snapshot_cv_.wait(lock, [this]() { return state_sender_stop_ || pending_state_snapshot_.has_value(); });
        if (state_sender_stop_ && !pending_state_snapshot_) {
          return;
        }
        snapshot = *pending_state_snapshot_;
        pending_state_snapshot_.reset();
      }
      send_state_snapshot(snapshot);
    }
  }

  bool state_sender_stop_requested()
  {
    std::lock_guard<std::mutex> lock(state_snapshot_mutex_);
    return state_sender_stop_;
  }

  std::chrono::nanoseconds state_publish_period() const
  {
    const double seconds = static_cast<double>(cfg_.freq.state_decimation) / cfg_.freq.physical_hz;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds));
    if (period.count() <= 0) {
      throw std::runtime_error("Computed state publish period is not positive");
    }
    return period;
  }

  static uint64_t read_timer_expirations(int timer_fd)
  {
    uint64_t expirations = 0;
    while (true) {
      const ssize_t n = ::read(timer_fd, &expirations, sizeof(expirations));
      if (n == static_cast<ssize_t>(sizeof(expirations))) {
        return expirations;
      }
      if (n < 0 && errno == EINTR) {
        continue;
      }
      if (n < 0) {
        throw std::runtime_error(errno_text("timerfd read() failed"));
      }
      throw std::runtime_error("timerfd read() returned a short read");
    }
  }

  void state_sender_timer_loop()
  {
    const std::chrono::nanoseconds period = state_publish_period();
    const int timer_fd = create_periodic_timer_fd(period);

    try {
      while (!state_sender_stop_requested()) {
        pollfd pfd{};
        pfd.fd = timer_fd;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, 50);
        if (ready < 0) {
          if (errno == EINTR) {
            continue;
          }
          throw std::runtime_error(errno_text("poll(timerfd) failed"));
        }
        if (ready == 0) {
          continue;
        }
        if ((pfd.revents & POLLIN) == 0) {
          continue;
        }

        const uint64_t expirations = read_timer_expirations(timer_fd);
        if (expirations == 0) {
          continue;
        }
        if (expirations > 1) {
          timer_missed_period_count_.fetch_add(expirations - 1, std::memory_order_relaxed);
        }

        LowState low_state;
        if (!read_lowstate_from_subscriber(low_state)) {
          timer_no_snapshot_count_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
        DiagnosticSnapshot diagnostics;
        if (!process_lowstate_sample(
                low_state, /*allow_tick_publish=*/false, &diagnostics)) {
          timer_skipped_read_count_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
        send_state_snapshot(
            StateSnapshot{low_state, consume_joint_torque_average(), std::move(diagnostics)});
      }
    } catch (const std::exception & exc) {
      state_send_error_count_.fetch_add(1, std::memory_order_relaxed);
      std::cerr << "[G1Bridge] Timer state sender failed: " << exc.what() << std::endl;
      g_stop_requested.store(true, std::memory_order_relaxed);
    }
    ::close(timer_fd);
  }

  void send_state_snapshot(const StateSnapshot & snapshot)
  {
    const LowState & low_state = snapshot.low_state;
    std::vector<float> q_real(cfg_.real_joint_names.size(), 0.0f);
    std::vector<float> dq_real(cfg_.real_joint_names.size(), 0.0f);
    std::vector<float> tau_latest_real(cfg_.real_joint_names.size(), 0.0f);
    std::vector<float> tau_real = snapshot.tau_real_average;
    if (tau_real.size() != cfg_.real_joint_names.size()) {
      tau_real.assign(cfg_.real_joint_names.size(), 0.0f);
      for (size_t i = 0; i < cfg_.real_joint_names.size(); ++i) {
        tau_real[i] = low_state.motor_state().at(i).tau_est();
      }
    }
    for (size_t i = 0; i < cfg_.real_joint_names.size(); ++i) {
      q_real[i] = low_state.motor_state().at(i).q();
      dq_real[i] = low_state.motor_state().at(i).dq();
      tau_latest_real[i] = low_state.motor_state().at(i).tau_est();
    }

    std::vector<float> q_policy(cfg_.policy_joint_names.size(), 0.0f);
    std::vector<float> dq_policy(cfg_.policy_joint_names.size(), 0.0f);
    std::vector<float> tau_policy(cfg_.policy_joint_names.size(), 0.0f);
    std::vector<float> tau_latest_policy(cfg_.policy_joint_names.size(), 0.0f);
    MotorTelemetry motor_policy;
    motor_policy.ddq.resize(cfg_.policy_joint_names.size(), 0.0f);
    motor_policy.casing_temperature.resize(cfg_.policy_joint_names.size(), 0.0f);
    motor_policy.winding_temperature.resize(cfg_.policy_joint_names.size(), 0.0f);
    motor_policy.voltage.resize(cfg_.policy_joint_names.size(), 0.0f);
    motor_policy.mode.resize(cfg_.policy_joint_names.size(), 0U);
    motor_policy.sensor_0.resize(cfg_.policy_joint_names.size(), 0U);
    motor_policy.sensor_1.resize(cfg_.policy_joint_names.size(), 0U);
    motor_policy.state.resize(cfg_.policy_joint_names.size(), 0U);
    for (auto & reserve : motor_policy.reserve) {
      reserve.resize(cfg_.policy_joint_names.size(), 0U);
    }
    DiagnosticSnapshot diagnostics_policy = snapshot.diagnostics;
    diagnostics_policy.motor_flags.assign(cfg_.policy_joint_names.size(), 0U);
    for (size_t i = 0; i < cfg_.policy_joint_names.size(); ++i) {
      const size_t real_index = real_to_policy_[i];
      const auto & motor = low_state.motor_state().at(real_index);
      q_policy[i] = q_real[real_index];
      dq_policy[i] = dq_real[real_index];
      tau_policy[i] = tau_real[real_index];
      tau_latest_policy[i] = tau_latest_real[real_index];
      motor_policy.ddq[i] = motor.ddq();
      motor_policy.casing_temperature[i] = static_cast<float>(motor.temperature()[0]);
      motor_policy.winding_temperature[i] = static_cast<float>(motor.temperature()[1]);
      motor_policy.voltage[i] = motor.vol();
      motor_policy.mode[i] = static_cast<uint32_t>(motor.mode());
      motor_policy.sensor_0[i] = motor.sensor()[0];
      motor_policy.sensor_1[i] = motor.sensor()[1];
      motor_policy.state[i] = motor.motorstate();
      for (size_t reserve_index = 0; reserve_index < motor_policy.reserve.size(); ++reserve_index) {
        motor_policy.reserve[reserve_index][i] = motor.reserve()[reserve_index];
      }
      if (real_index < snapshot.diagnostics.motor_flags.size()) {
        diagnostics_policy.motor_flags[i] = snapshot.diagnostics.motor_flags[real_index];
      }
    }

    const auto & imu = low_state.imu_state();
    const std::vector<float> quat{
        imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2], imu.quaternion()[3]};
    const std::vector<float> gyro{imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2]};
    const std::vector<float> linacc{imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2]};
    const RemoteState remote = apply_stdin_button_overrides(parse_remote(low_state.wireless_remote()));

    try {
      state_sender_.send_state(
          q_policy, dq_policy, tau_policy, tau_latest_policy, quat, gyro, linacc, remote,
          motor_policy, diagnostics_policy, low_state.mode_machine());
      state_forward_count_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception & exc) {
      state_send_error_count_.fetch_add(1, std::memory_order_relaxed);
      std::cerr << "[G1Bridge] UDP state send failed: " << exc.what() << std::endl;
    }
  }

  void on_udp_command(const LatestPacket & packet)
  {
    if (fatal_shutdown_requested_.load(std::memory_order_relaxed) ||
        g_stop_requested.load(std::memory_order_relaxed)) {
      return;
    }
    const int64_t packet_seq = static_cast<int64_t>(packet.seq);
    const int64_t prev_seq = latest_cmd_seq_.exchange(packet_seq);
    if (packet_seq <= prev_seq) {
      return;
    }
    if (!have_mode_machine_.load()) {
      std::cerr << "[G1Bridge] Ignore UDP command before mode_machine sync" << std::endl;
      return;
    }

    try {
      const std::vector<double> q_src = read_array_as_double(packet.data, packet.payload, "q_des");
      const std::vector<double> qd_src = read_array_as_double(packet.data, packet.payload, "qd_des");
      const std::vector<double> kp_src = read_array_as_double(packet.data, packet.payload, "kp");
      const std::vector<double> kd_src = read_array_as_double(packet.data, packet.payload, "kd");
      const size_t policy_dof = cfg_.policy_joint_names.size();
      if (q_src.size() != policy_dof || qd_src.size() != policy_dof || kp_src.size() != policy_dof ||
          kd_src.size() != policy_dof) {
        std::cerr << "[G1Bridge] Ignore UDP command with unexpected DOF size" << std::endl;
        return;
      }
      const int enable = packet.data["enable"] ? packet.data["enable"].as<int>() : 0;

      LowCmd cmd;
      cmd.mode_pr() = cfg_.low_level.mode_pr;
      cmd.mode_machine() = mode_machine_.load();
      for (auto & motor_cmd : cmd.motor_cmd()) {
        motor_cmd.mode() = 1;
        motor_cmd.q() = 0.0f;
        motor_cmd.dq() = 0.0f;
        motor_cmd.kp() = 0.0f;
        motor_cmd.kd() = 0.0f;
        motor_cmd.tau() = 0.0f;
      }
      for (size_t real_idx = 0; real_idx < cfg_.real_joint_names.size(); ++real_idx) {
        const size_t policy_idx = policy_to_real_[real_idx];
        auto & motor_cmd = cmd.motor_cmd().at(real_idx);
        motor_cmd.q() = static_cast<float>(q_src[policy_idx]);
        motor_cmd.dq() = static_cast<float>(qd_src[policy_idx]);
        motor_cmd.kp() = static_cast<float>(kp_src[policy_idx]);
        motor_cmd.kd() = static_cast<float>(kd_src[policy_idx]);
        motor_cmd.tau() = 0.0f;
      }
      cmd.reserve()[0] = static_cast<uint32_t>(enable);
      cmd.crc() = crc32_core((uint32_t *)&cmd, (static_cast<uint32_t>(sizeof(LowCmd)) >> 2) - 1);

      {
        std::lock_guard<std::mutex> lock(cmd_write_mutex_);
        lowcmd_publisher_->Write(cmd);
        if (cfg_.safety.enabled) {
          const bool first_valid_command =
              !have_valid_command_.load(std::memory_order_relaxed);
          last_valid_command_ns_.store(now_ns(), std::memory_order_relaxed);
          have_valid_command_.store(true, std::memory_order_release);
          command_timeout_warning_active_.store(false, std::memory_order_relaxed);
          if (first_valid_command && cfg_.safety.startup_damping) {
            std::cout << "[G1Bridge] First valid command accepted; "
                         "startup damping released and command timeout monitor armed"
                      << std::endl;
          }
        }
      }
      record_policy_delay(yaml_u64_optional(packet.data["state_receive_time_ns"]));
      command_forward_count_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception & exc) {
      std::cerr << "[G1Bridge] Ignore malformed UDP command: " << exc.what() << std::endl;
    }
  }

  void start_command_watchdog()
  {
    if (!cfg_.safety.enabled) {
      return;
    }
    watchdog_stop_.store(false);
    command_watchdog_thread_ = std::thread([this]() {
      const uint64_t timeout_ns = static_cast<uint64_t>(
          cfg_.safety.command_timeout_s * 1.0e9);
      const auto damping_period = std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(1.0 / cfg_.safety.damping_publish_hz));
      auto next_damping = SteadyClock::now();
      while (!watchdog_stop_.load(std::memory_order_relaxed) &&
             !g_stop_requested.load(std::memory_order_relaxed)) {
        const bool have_command =
            have_valid_command_.load(std::memory_order_acquire);
        if (have_command) {
          const uint64_t last = last_valid_command_ns_.load(std::memory_order_relaxed);
          const uint64_t now = now_ns();
          if (now > last && now - last > timeout_ns) {
            if (!command_timeout_warning_active_.exchange(
                    true, std::memory_order_relaxed)) {
              const double elapsed_s =
                  static_cast<double>(now - last) * 1.0e-9;
              std::cerr << kAnsiBoldRed
                        << "[G1Bridge] WARNING: valid UDP command timeout ("
                        << elapsed_s << "s > "
                        << cfg_.safety.command_timeout_s
                        << "s); timeout damping is disabled, waiting for "
                           "commands to resume"
                        << kAnsiReset << std::endl;
            }
          }
        }
        const bool should_publish_damping =
            cfg_.safety.startup_damping && !have_command;
        const auto steady_now = SteadyClock::now();
        if (should_publish_damping && steady_now >= next_damping) {
          publish_damping_command(
              /*log=*/false, /*require_startup_pending=*/true);
          next_damping = steady_now + damping_period;
        } else if (!should_publish_damping) {
          next_damping = steady_now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
  }

  void stop_command_watchdog()
  {
    watchdog_stop_.store(true);
    if (command_watchdog_thread_.joinable()) {
      command_watchdog_thread_.join();
    }
  }

  void record_policy_delay(const std::optional<uint64_t> state_receive_time_ns)
  {
    if (!state_receive_time_ns) {
      return;
    }
    const uint64_t now = now_ns();
    if (*state_receive_time_ns > now) {
      return;
    }
    const double delay_ms = static_cast<double>(now - *state_receive_time_ns) * 1e-6;
    std::lock_guard<std::mutex> lock(policy_delay_mutex_);
    ++policy_delay_count_;
    policy_delay_sum_ms_ += delay_ms;
    policy_delay_min_ms_ = std::min(policy_delay_min_ms_, delay_ms);
    policy_delay_max_ms_ = std::max(policy_delay_max_ms_, delay_ms);
  }

  void publish_damping_command(
      bool log = true, bool require_startup_pending = false)
  {
    if (!lowcmd_publisher_) {
      return;
    }
    LowCmd cmd;
    cmd.mode_pr() = cfg_.low_level.mode_pr;
    cmd.mode_machine() = mode_machine_.load();
    for (auto & motor_cmd : cmd.motor_cmd()) {
      motor_cmd.mode() = 1;
      motor_cmd.q() = 0.0f;
      motor_cmd.dq() = 0.0f;
      motor_cmd.kp() = 0.0f;
      motor_cmd.kd() = static_cast<float>(cfg_.low_level.damping_kd);
      motor_cmd.tau() = 0.0f;
    }
    cmd.crc() = crc32_core((uint32_t *)&cmd, (static_cast<uint32_t>(sizeof(LowCmd)) >> 2) - 1);
    std::lock_guard<std::mutex> lock(cmd_write_mutex_);
    if (require_startup_pending &&
        (!cfg_.safety.startup_damping ||
         have_valid_command_.load(std::memory_order_acquire))) {
      return;
    }
    lowcmd_publisher_->Write(cmd);
    damping_command_count_.fetch_add(1, std::memory_order_relaxed);
    if (log) {
      std::cout << "[G1Bridge] Damping command sent" << std::endl;
    }
  }

  void log_rates()
  {
    const auto now = SteadyClock::now();
    const double elapsed = std::chrono::duration<double>(now - rate_window_start_).count();
    if (elapsed <= 0.0) {
      return;
    }
    rate_window_start_ = now;

    const uint64_t lowstate_callbacks = lowstate_callback_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t lowstate_reads = lowstate_read_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t lowstate_count = lowstate_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t state_count = state_forward_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t command_count = command_forward_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t crc_errors = lowstate_crc_error_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t duplicate_ticks = lowstate_duplicate_tick_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t tick_gap_events = lowstate_tick_gap_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t tick_missing = lowstate_tick_missing_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t tick_resets = lowstate_tick_reset_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t snapshot_overwrites = state_snapshot_overwrite_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t snapshot_drops = state_snapshot_drop_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t state_send_errors = state_send_error_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t timer_missed_periods = timer_missed_period_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t timer_no_snapshot = timer_no_snapshot_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t timer_skipped_reads = timer_skipped_read_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t stdin_button_events = stdin_button_event_count_.exchange(0, std::memory_order_relaxed);
    const uint64_t damping_commands = damping_command_count_.exchange(0, std::memory_order_relaxed);

    uint64_t udp_rx_delta = 0;
    uint64_t udp_decoded_delta = 0;
    uint64_t udp_errors_delta = 0;
    uint64_t udp_source_rejections_delta = 0;
    if (command_receiver_) {
      const uint64_t rx = command_receiver_->packets_received();
      const uint64_t decoded = command_receiver_->packets_decoded();
      const uint64_t errors = command_receiver_->decode_errors();
      const uint64_t source_rejections = command_receiver_->source_rejections();
      udp_rx_delta = rx - last_udp_rx_;
      udp_decoded_delta = decoded - last_udp_decoded_;
      udp_errors_delta = errors - last_udp_errors_;
      udp_source_rejections_delta =
          source_rejections - last_udp_source_rejections_;
      last_udp_rx_ = rx;
      last_udp_decoded_ = decoded;
      last_udp_errors_ = errors;
      last_udp_source_rejections_ = source_rejections;
    }

    uint64_t delay_count = 0;
    double delay_mean_ms = 0.0;
    double delay_min_ms = 0.0;
    double delay_max_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(policy_delay_mutex_);
      delay_count = policy_delay_count_;
      if (delay_count > 0) {
        delay_mean_ms = policy_delay_sum_ms_ / static_cast<double>(delay_count);
        delay_min_ms = policy_delay_min_ms_;
        delay_max_ms = policy_delay_max_ms_;
      }
      policy_delay_count_ = 0;
      policy_delay_sum_ms_ = 0.0;
      policy_delay_min_ms_ = std::numeric_limits<double>::infinity();
      policy_delay_max_ms_ = 0.0;
    }

    std::cout << "[G1Bridge] rates over " << std::fixed << std::setprecision(2) << elapsed
              << "s | lowstate_callbacks=" << std::setprecision(1)
              << (static_cast<double>(lowstate_callbacks) / elapsed) << " Hz (" << lowstate_callbacks << ")"
              << " | lowstate_reads=" << std::setprecision(1)
              << (static_cast<double>(lowstate_reads) / elapsed) << " Hz (" << lowstate_reads << ")"
              << " | lowstate_unique="
              << std::setprecision(1) << (static_cast<double>(lowstate_count) / elapsed) << " Hz (" << lowstate_count
              << ", configured_physical=" << cfg_.freq.physical_hz << ") | state="
              << (static_cast<double>(state_count) / elapsed) << " Hz (" << state_count << ", configured_expected="
              << cfg_.freq.physical_hz / static_cast<double>(cfg_.freq.state_decimation)
              << ", mode=" << state_publish_mode_name(cfg_.freq.state_publish_mode) << ") | command="
              << (static_cast<double>(command_count) / elapsed) << " Hz (" << command_count << ") | cmd_udp_rx="
              << udp_rx_delta << " decoded=" << udp_decoded_delta << " errors=" << udp_errors_delta
              << " source_rejected=" << udp_source_rejections_delta
              << " | lowstate_crc_errors=" << crc_errors << " duplicate_ticks=" << duplicate_ticks
              << " tick_gap_events=" << tick_gap_events << " tick_missing=" << tick_missing
              << " tick_resets=" << tick_resets << " state_snapshot_overwrites=" << snapshot_overwrites
              << " state_snapshot_drops=" << snapshot_drops << " state_send_errors=" << state_send_errors
              << " stdin_button_events=" << stdin_button_events
              << " damping_commands=" << damping_commands;
    if (cfg_.freq.state_publish_mode == StatePublishMode::Timer) {
      std::cout << " timer_missed_periods=" << timer_missed_periods
                << " timer_no_snapshot=" << timer_no_snapshot
                << " timer_skipped_reads=" << timer_skipped_reads;
    }
    if (delay_count > 0) {
      std::cout << " | policy_delay_ms mean=" << std::setprecision(3) << delay_mean_ms << " min=" << delay_min_ms
                << " max=" << delay_max_ms << " n=" << delay_count;
    } else {
      std::cout << " | policy_delay_ms n/a";
    }
    std::cout << std::endl;
  }

  BridgeConfig cfg_;
  std::string network_interface_;
  std::unordered_map<std::string, size_t> policy_index_;
  std::unordered_map<std::string, size_t> real_index_;
  std::vector<size_t> real_to_policy_;
  std::vector<size_t> policy_to_real_;

  unitree::robot::ChannelPublisherPtr<LowCmd> lowcmd_publisher_;
  unitree::robot::ChannelSubscriberPtr<LowState> lowstate_subscriber_;
  UdpLatestSender state_sender_;
  std::unique_ptr<UdpLatestReceiver> command_receiver_;
  std::thread command_watchdog_thread_;
  std::atomic<bool> watchdog_stop_{false};
  std::atomic<bool> have_valid_command_{false};
  std::atomic<bool> command_timeout_warning_active_{false};
  std::atomic<uint64_t> last_valid_command_ns_{0};

  std::atomic<bool> closed_{false};
  std::atomic<bool> fatal_shutdown_requested_{false};
  std::atomic<bool> have_lowstate_{false};
  std::atomic<bool> have_mode_machine_{false};
  std::atomic<uint8_t> mode_machine_{0};
  std::mutex first_state_mutex_;
  std::condition_variable first_state_cv_;
  std::mutex cmd_write_mutex_;
  std::mutex lowstate_tick_mutex_;
  std::optional<uint32_t> last_lowstate_tick_;
  std::optional<uint32_t> next_state_tick_;
  std::mutex torque_accumulator_mutex_;
  std::vector<double> torque_sum_real_;
  size_t torque_sample_count_ = 0;
  std::mutex diagnostics_mutex_;
  std::vector<uint32_t> previous_motor_diagnostic_flags_;
  std::vector<uint32_t> previous_motor_state_codes_;
  uint32_t previous_imu_diagnostic_flags_ = 0;
  std::optional<SteadyClock::time_point> next_diagnostic_report_time_;
  std::mutex state_snapshot_mutex_;
  std::condition_variable state_snapshot_cv_;
  std::optional<StateSnapshot> pending_state_snapshot_;
  std::thread state_sender_thread_;
  bool state_sender_stop_ = false;
  std::thread stdin_button_thread_;
  std::atomic<bool> stdin_button_stop_{false};
  std::atomic<uint64_t> stdin_start_until_ns_{0};
  std::atomic<uint64_t> stdin_a_until_ns_{0};
  std::atomic<uint64_t> stdin_stop_until_ns_{0};

  std::atomic<uint64_t> lowstate_callback_count_{0};
  std::atomic<uint64_t> lowstate_read_count_{0};
  std::atomic<uint64_t> lowstate_count_{0};
  std::atomic<uint64_t> lowstate_unique_tick_{0};
  std::atomic<uint64_t> state_forward_count_{0};
  std::atomic<uint64_t> command_forward_count_{0};
  std::atomic<uint64_t> lowstate_crc_error_count_{0};
  std::atomic<uint64_t> lowstate_duplicate_tick_count_{0};
  std::atomic<uint64_t> lowstate_tick_gap_count_{0};
  std::atomic<uint64_t> lowstate_tick_missing_count_{0};
  std::atomic<uint64_t> lowstate_tick_reset_count_{0};
  std::atomic<uint64_t> state_snapshot_overwrite_count_{0};
  std::atomic<uint64_t> state_snapshot_drop_count_{0};
  std::atomic<uint64_t> state_send_error_count_{0};
  std::atomic<uint64_t> timer_missed_period_count_{0};
  std::atomic<uint64_t> timer_no_snapshot_count_{0};
  std::atomic<uint64_t> timer_skipped_read_count_{0};
  std::atomic<uint64_t> stdin_button_event_count_{0};
  std::atomic<uint64_t> damping_command_count_{0};
  std::atomic<int64_t> latest_cmd_seq_{-1};

  std::mutex policy_delay_mutex_;
  uint64_t policy_delay_count_ = 0;
  double policy_delay_sum_ms_ = 0.0;
  double policy_delay_min_ms_ = std::numeric_limits<double>::infinity();
  double policy_delay_max_ms_ = 0.0;

  uint64_t last_udp_rx_ = 0;
  uint64_t last_udp_decoded_ = 0;
  uint64_t last_udp_errors_ = 0;
  uint64_t last_udp_source_rejections_ = 0;
  SteadyClock::time_point rate_window_start_ = SteadyClock::now();
};

struct ProgramOptions {
  std::string config_path = "config/g1_bridge.yaml";
  std::string network_interface = "lo";
  std::optional<std::string> state_host;
  std::optional<std::string> cmd_bind_host;
  std::optional<std::string> cmd_allowed_host;
};

ProgramOptions parse_options(int argc, char ** argv)
{
  ProgramOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--config" || arg == "--bridge-config") && i + 1 < argc) {
      options.config_path = argv[++i];
      continue;
    }
    if (arg.rfind("--config=", 0) == 0) {
      options.config_path = arg.substr(std::string("--config=").size());
      continue;
    }
    if ((arg == "--net" || arg == "--network-interface") && i + 1 < argc) {
      options.network_interface = argv[++i];
      continue;
    }
    if (arg.rfind("--net=", 0) == 0) {
      options.network_interface = arg.substr(std::string("--net=").size());
      continue;
    }
    if (arg == "--state-host" && i + 1 < argc) {
      options.state_host = argv[++i];
      continue;
    }
    if (arg.rfind("--state-host=", 0) == 0) {
      options.state_host = arg.substr(std::string("--state-host=").size());
      continue;
    }
    if (arg == "--cmd-bind-host" && i + 1 < argc) {
      options.cmd_bind_host = argv[++i];
      continue;
    }
    if (arg.rfind("--cmd-bind-host=", 0) == 0) {
      options.cmd_bind_host =
          arg.substr(std::string("--cmd-bind-host=").size());
      continue;
    }
    if (arg == "--cmd-allowed-host" && i + 1 < argc) {
      options.cmd_allowed_host = argv[++i];
      continue;
    }
    if (arg.rfind("--cmd-allowed-host=", 0) == 0) {
      options.cmd_allowed_host =
          arg.substr(std::string("--cmd-allowed-host=").size());
      continue;
    }
    if (arg == "-h" || arg == "--help") {
      std::cout
          << "Usage: g1_udp_bridge [--net IFACE] [--config PATH] "
             "[--state-host IP] [--cmd-bind-host IP] "
             "[--cmd-allowed-host IP]"
          << std::endl;
      std::exit(0);
    }
    throw std::runtime_error("Unknown or incomplete argument: " + arg);
  }
  return options;
}

}  // namespace
}  // namespace g1_bridge

int main(int argc, char ** argv)
{
  std::signal(SIGINT, g1_bridge::handle_signal);
  std::signal(SIGTERM, g1_bridge::handle_signal);

  try {
    const g1_bridge::ProgramOptions options = g1_bridge::parse_options(argc, argv);
    g1_bridge::BridgeConfig config = g1_bridge::load_config(options.config_path);
    if (options.state_host) {
      config.udp.state_host = *options.state_host;
    }
    if (options.cmd_bind_host) {
      config.udp.cmd_bind_host = *options.cmd_bind_host;
    }
    if (options.cmd_allowed_host) {
      config.udp.cmd_allowed_host = *options.cmd_allowed_host;
    }
    g1_bridge::G1UdpBridge bridge(std::move(config), options.network_interface);
    bridge.run();
    bridge.close();
  } catch (const std::exception & exc) {
    std::cerr << "[G1Bridge] Fatal: " << exc.what() << std::endl;
    return 1;
  }
  return 0;
}
