#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "romo_b_perception/imu_normalization.hpp"

namespace romo_b_perception
{

class ImuNormalizer final : public rclcpp::Node
{
public:
  ImuNormalizer()
  : Node("romo_b_imu_normalizer")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/sensing/imu/livox_raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/sensing/imu/imu_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    acceleration_scale_ = declare_parameter<double>("acceleration_scale", 9.80665);
    (void)normalize_livox_imu(sensor_msgs::msg::Imu{}, acceleration_scale_, frame_id_);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(200)).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuNormalizer::on_imu, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Normalizing Livox acceleration by %.5f: %s -> %s",
      acceleration_scale_, input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr message)
  {
    publisher_->publish(normalize_livox_imu(*message, acceleration_scale_, frame_id_));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double acceleration_scale_{9.80665};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

}  // namespace romo_b_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<romo_b_perception::ImuNormalizer>());
  rclcpp::shutdown();
  return 0;
}
