#include "romo_b_base/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace romo_b_base
{
namespace
{

constexpr std::array<std::uint8_t, 3> kHeader{0x53, 0x54, 0x58};
constexpr std::array<std::uint8_t, 2> kTail{0x0d, 0x0a};

void put_be_i16(std::array<std::uint8_t, kCommandFrameSize> & frame, std::size_t offset,
  std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  frame[offset] = static_cast<std::uint8_t>((raw >> 8U) & 0xffU);
  frame[offset + 1] = static_cast<std::uint8_t>(raw & 0xffU);
}

void put_le_i16(std::array<std::uint8_t, kCommandFrameSize> & frame, std::size_t offset,
  std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  frame[offset] = static_cast<std::uint8_t>(raw & 0xffU);
  frame[offset + 1] = static_cast<std::uint8_t>((raw >> 8U) & 0xffU);
}

std::int16_t get_le_i16(
  const std::array<std::uint8_t, kFeedbackFrameSize> & frame, std::size_t offset)
{
  const auto raw = static_cast<std::uint16_t>(frame[offset]) |
    (static_cast<std::uint16_t>(frame[offset + 1]) << 8U);
  return static_cast<std::int16_t>(raw);
}

bool plausible(const std::array<std::uint8_t, kFeedbackFrameSize> & frame)
{
  if (!std::equal(kHeader.begin(), kHeader.end(), frame.begin()) ||
    frame[23] != kTail[0] || frame[24] != kTail[1])
  {
    return false;
  }
  if (frame[3] > 1U || frame[5] > 2U) {
    return false;
  }
  for (std::size_t offset = 6; offset <= 12; offset += 2) {
    if (std::abs(static_cast<int>(get_le_i16(frame, offset))) > 200) {
      return false;
    }
  }
  for (std::size_t offset = 14; offset <= 20; offset += 2) {
    if (std::abs(static_cast<int>(get_le_i16(frame, offset))) > 350) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::array<std::uint8_t, kCommandFrameSize> encode_command(
  const Command & command, bool little_endian)
{
  std::array<std::uint8_t, kCommandFrameSize> frame{};
  std::copy(kHeader.begin(), kHeader.end(), frame.begin());
  frame[3] = command.auto_mode ? 1U : 0U;
  frame[4] = command.estop ? 1U : 0U;
  frame[5] = static_cast<std::uint8_t>(command.steer_mode);

  const auto speed_raw = static_cast<std::int16_t>(std::lround(
      std::clamp(command.speed_mps, -1.5, 1.5) * 100.0));
  const auto steer_raw = static_cast<std::int16_t>(std::lround(
      std::clamp(command.steer_deg, -300.0, 300.0)));
  const auto put_i16 = little_endian ? put_le_i16 : put_be_i16;
  put_i16(frame, 6, speed_raw);
  put_i16(frame, 8, steer_raw);
  frame[10] = command.alive;
  frame[11] = kTail[0];
  frame[12] = kTail[1];
  return frame;
}

std::optional<Feedback> decode_feedback(
  const std::array<std::uint8_t, kFeedbackFrameSize> & frame)
{
  if (!plausible(frame)) {
    return std::nullopt;
  }

  Feedback result;
  result.auto_mode = frame[3] == 1U;
  result.estop_raw = frame[4];
  result.estop = result.estop_raw != 0U;
  result.steer_mode = static_cast<SteerMode>(frame[5]);
  for (std::size_t index = 0; index < 4; ++index) {
    result.wheel_speed_mps[index] = static_cast<double>(get_le_i16(frame, 6 + index * 2)) * 0.01;
    result.wheel_steer_rad[index] =
      static_cast<double>(get_le_i16(frame, 14 + index * 2)) * 0.1 * kDegreesToRadians;
  }
  result.alive = frame[22];
  return result;
}

ControlOutput map_twist(
  double linear_x_mps, double angular_z_radps, const ControlLimits & limits)
{
  ControlOutput output;
  if (!std::isfinite(linear_x_mps) || !std::isfinite(angular_z_radps) ||
    limits.wheelbase_m <= 0.0 || limits.max_speed_mps <= 0.0 || limits.max_steer_deg <= 0.0)
  {
    output.valid = false;
    return output;
  }

  if (!limits.allow_reverse && linear_x_mps < -limits.zero_speed_epsilon) {
    output.valid = false;
    return output;
  }
  if (std::abs(linear_x_mps) <= limits.zero_speed_epsilon) {
    if (std::abs(angular_z_radps) > limits.zero_speed_epsilon) {
      output.valid = false;
    }
    return output;
  }

  output.speed_mps = std::clamp(
    linear_x_mps, limits.allow_reverse ? -limits.max_speed_mps : 0.0, limits.max_speed_mps);
  output.clamped = output.speed_mps != linear_x_mps;

  const double ros_steer_deg =
    std::atan(limits.wheelbase_m * angular_z_radps / linear_x_mps) * kRadiansToDegrees;
  const double limited_ros_steer_deg =
    std::clamp(ros_steer_deg, -limits.max_steer_deg, limits.max_steer_deg);
  output.clamped = output.clamped || limited_ros_steer_deg != ros_steer_deg;

  // ROS positive angular.z means left; the PCU manual defines left as negative.
  output.pcu_steer_deg = -limited_ros_steer_deg;
  return output;
}

VehicleMotion estimate_vehicle_motion(
  const Feedback & feedback, double wheelbase_m, double control_track_m)
{
  VehicleMotion motion;
  motion.center_speed_mps =
    0.5 * (feedback.wheel_speed_mps[2] + feedback.wheel_speed_mps[3]);
  if (wheelbase_m <= 0.0 || control_track_m <= 0.0) {
    return motion;
  }

  // Convert PCU negative-left convention to ROS positive-left convention.
  const double fl = -feedback.wheel_steer_rad[0];
  const double fr = -feedback.wheel_steer_rad[1];
  std::array<double, 2> radii{};
  std::size_t count = 0;
  if (std::abs(std::tan(fl)) > 1.0e-5) {
    radii[count++] = wheelbase_m / std::tan(fl) + control_track_m * 0.5;
  }
  if (std::abs(std::tan(fr)) > 1.0e-5) {
    radii[count++] = wheelbase_m / std::tan(fr) - control_track_m * 0.5;
  }
  if (count == 0) {
    return motion;
  }

  double radius = radii[0];
  if (count == 2) {
    if (radii[0] * radii[1] <= 0.0) {
      return motion;
    }
    radius = 0.5 * (radii[0] + radii[1]);
  }
  if (!std::isfinite(radius) || std::abs(radius) < control_track_m * 0.5) {
    return motion;
  }
  const double curvature = 1.0 / radius;
  motion.equivalent_steer_rad = std::atan(wheelbase_m * curvature);
  motion.yaw_rate_radps = motion.center_speed_mps * curvature;
  return motion;
}

std::vector<Feedback> FeedbackParser::push(const std::uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  std::vector<Feedback> decoded;

  while (buffer_.size() >= kHeader.size()) {
    auto header = std::search(buffer_.begin(), buffer_.end(), kHeader.begin(), kHeader.end());
    if (header == buffer_.end()) {
      const std::size_t keep = std::min(buffer_.size(), kHeader.size() - 1);
      buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      break;
    }
    buffer_.erase(buffer_.begin(), header);
    if (buffer_.size() < kFeedbackFrameSize) {
      break;
    }

    std::array<std::uint8_t, kFeedbackFrameSize> frame{};
    std::copy_n(buffer_.begin(), kFeedbackFrameSize, frame.begin());
    auto feedback = decode_feedback(frame);
    if (feedback.has_value()) {
      decoded.push_back(*feedback);
      buffer_.erase(buffer_.begin(), buffer_.begin() + kFeedbackFrameSize);
    } else {
      buffer_.erase(buffer_.begin());
    }
  }
  return decoded;
}

}  // namespace romo_b_base
