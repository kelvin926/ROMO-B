#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace romo_b_base
{

constexpr std::size_t kCommandFrameSize = 13;
constexpr std::size_t kFeedbackFrameSize = 25;
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kRadiansToDegrees = 57.29577951308232;

enum class SteerMode : std::uint8_t
{
  k2Wis = 0,
  k4Wis = 1,
  kPivot = 2,
};

struct Command
{
  bool auto_mode{false};
  bool estop{false};
  SteerMode steer_mode{SteerMode::k2Wis};
  double speed_mps{0.0};
  double steer_deg{0.0};
  std::uint8_t alive{0};
};

struct Feedback
{
  bool auto_mode{false};
  bool estop{false};
  SteerMode steer_mode{SteerMode::k2Wis};
  std::array<double, 4> wheel_speed_mps{};  // FL, FR, RL, RR
  std::array<double, 4> wheel_steer_rad{};  // PCU convention: left is negative
  std::uint8_t alive{0};
};

struct ControlLimits
{
  double wheelbase_m{0.323};
  double max_speed_mps{0.2};
  double max_steer_deg{22.0};
  double zero_speed_epsilon{1.0e-3};
  bool allow_reverse{false};
};

struct ControlOutput
{
  bool valid{true};
  bool clamped{false};
  double speed_mps{0.0};
  double pcu_steer_deg{0.0};
};

struct VehicleMotion
{
  double center_speed_mps{0.0};
  double equivalent_steer_rad{0.0};  // ROS convention: left is positive
  double yaw_rate_radps{0.0};
};

std::array<std::uint8_t, kCommandFrameSize> encode_command(const Command & command);
std::optional<Feedback> decode_feedback(
  const std::array<std::uint8_t, kFeedbackFrameSize> & frame);
ControlOutput map_twist(double linear_x_mps, double angular_z_radps, const ControlLimits & limits);
VehicleMotion estimate_vehicle_motion(
  const Feedback & feedback, double wheelbase_m = 0.323, double control_track_m = 0.390);

class FeedbackParser
{
public:
  std::vector<Feedback> push(const std::uint8_t * data, std::size_t size);
  std::size_t buffered_bytes() const noexcept {return buffer_.size();}
  void clear() {buffer_.clear();}

private:
  std::vector<std::uint8_t> buffer_;
};

}  // namespace romo_b_base
