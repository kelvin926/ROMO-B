#include <cmath>
#include <iostream>

#include "romo_b_perception/imu_normalization.hpp"

int main()
{
  sensor_msgs::msg::Imu input;
  input.header.frame_id = "source";
  input.orientation.w = 0.25;
  input.orientation_covariance[0] = 4.0;
  input.angular_velocity.z = 0.5;
  input.linear_acceleration.x = 1.0;
  input.linear_acceleration.y = -0.5;
  input.linear_acceleration.z = 0.25;
  input.linear_acceleration_covariance[0] = 2.0;

  constexpr double gravity = 9.80665;
  const auto output = romo_b_perception::normalize_livox_imu(
    input, gravity, "livox_frame");
  const auto close = [](double lhs, double rhs) {
      return std::abs(lhs - rhs) < 1.0e-9;
    };

  if (output.header.frame_id != "livox_frame" ||
    !close(output.angular_velocity.z, 0.5) ||
    !close(output.linear_acceleration.x, gravity) ||
    !close(output.linear_acceleration.y, -0.5 * gravity) ||
    !close(output.linear_acceleration.z, 0.25 * gravity) ||
    !close(output.linear_acceleration_covariance[0], 2.0 * gravity * gravity) ||
    !close(output.orientation.w, 1.0) || output.orientation_covariance[0] != -1.0)
  {
    std::cerr << "Livox IMU normalization test failed\n";
    return 1;
  }
  return 0;
}
