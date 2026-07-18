#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SafeCommandHeartbeat final : public rclcpp::Node
{
public:
  SafeCommandHeartbeat()
  : Node("safe_command_heartbeat")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/cmd_vel_collision_checked");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_safe");
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.12);
    publish_frequency_ = declare_parameter<double>("publish_frequency", 20.0);
    max_forward_speed_ = declare_parameter<double>("max_forward_speed", 0.20);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.25);
    if (
      input_timeout_sec_ <= 0.0 || input_timeout_sec_ >= 0.15 ||
      publish_frequency_ < 10.0 || max_forward_speed_ <= 0.0 || max_angular_speed_ <= 0.0)
    {
      throw std::invalid_argument("Invalid safe command heartbeat parameters");
    }

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        on_command(*message);
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_frequency_),
      [this]() {publish();});
  }

private:
  static bool finite(const geometry_msgs::msg::Twist & message)
  {
    return
      std::isfinite(message.linear.x) && std::isfinite(message.linear.y) &&
      std::isfinite(message.linear.z) && std::isfinite(message.angular.x) &&
      std::isfinite(message.angular.y) && std::isfinite(message.angular.z);
  }

  void on_command(const geometry_msgs::msg::Twist & message)
  {
    geometry_msgs::msg::Twist safe;
    if (
      finite(message) && message.linear.x >= 0.0 &&
      std::abs(message.linear.y) < 1.0e-6 && std::abs(message.linear.z) < 1.0e-6 &&
      std::abs(message.angular.x) < 1.0e-6 && std::abs(message.angular.y) < 1.0e-6)
    {
      safe.linear.x = std::min(message.linear.x, max_forward_speed_);
      safe.angular.z = std::clamp(
        message.angular.z, -max_angular_speed_, max_angular_speed_);
    }
    last_command_ = safe;
    last_command_time_ = now();
    command_received_ = true;
  }

  void publish()
  {
    geometry_msgs::msg::Twist output;
    if (
      command_received_ &&
      (now() - last_command_time_).seconds() <= input_timeout_sec_)
    {
      output = last_command_;
    }
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double input_timeout_sec_{0.12};
  double publish_frequency_{20.0};
  double max_forward_speed_{0.20};
  double max_angular_speed_{0.25};
  bool command_received_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist last_command_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeCommandHeartbeat>());
  rclcpp::shutdown();
  return 0;
}
