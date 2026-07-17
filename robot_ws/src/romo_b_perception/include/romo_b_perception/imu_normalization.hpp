#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/imu.hpp"

namespace romo_b_perception
{

inline sensor_msgs::msg::Imu normalize_livox_imu(
  const sensor_msgs::msg::Imu & input, const double acceleration_scale,
  const std::string & frame_id)
{
  if (!std::isfinite(acceleration_scale) || acceleration_scale <= 0.0) {
    throw std::invalid_argument("acceleration_scale must be finite and positive");
  }

  auto output = input;
  if (!frame_id.empty()) {
    output.header.frame_id = frame_id;
  }

  output.linear_acceleration.x *= acceleration_scale;
  output.linear_acceleration.y *= acceleration_scale;
  output.linear_acceleration.z *= acceleration_scale;
  const double covariance_scale = acceleration_scale * acceleration_scale;
  for (auto & value : output.linear_acceleration_covariance) {
    value *= covariance_scale;
  }

  // Mid-360 supplies angular velocity and acceleration, not an orientation
  // estimate. Mark orientation unavailable per sensor_msgs/Imu semantics.
  output.orientation.x = 0.0;
  output.orientation.y = 0.0;
  output.orientation.z = 0.0;
  output.orientation.w = 1.0;
  std::fill(
    output.orientation_covariance.begin(), output.orientation_covariance.end(), 0.0);
  output.orientation_covariance[0] = -1.0;
  return output;
}

}  // namespace romo_b_perception
