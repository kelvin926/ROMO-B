#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace romo_b_perception
{

class PointCloudFilter final : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("romo_b_pointcloud_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/sensing/lidar/top/pointcloud_raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/sensing/lidar/top/pointcloud_filtered");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
    min_z_ = declare_parameter<double>("min_z", 0.10);
    max_z_ = declare_parameter<double>("max_z", 1.80);
    self_half_x_ = declare_parameter<double>("self_half_x", 0.398);
    self_half_y_ = declare_parameter<double>("self_half_y", 0.319);
    self_min_z_ = declare_parameter<double>("self_min_z", 0.0);
    self_max_z_ = declare_parameter<double>("self_max_z", 0.45);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    if (voxel_size_ <= 0.0 || min_z_ >= max_z_ || self_half_x_ <= 0.0 || self_half_y_ <= 0.0) {
      throw std::invalid_argument("Invalid point cloud filter parameters");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudFilter::on_cloud, this, std::placeholders::_1));
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    sensor_msgs::msg::PointCloud2 transformed;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id, rclcpp::Time(message->header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      tf2::doTransform(*message, transformed, transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Point cloud transform unavailable: %s", error.what());
      return;
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(transformed, *cloud);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);

    auto height_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZ> height;
    height.setInputCloud(cloud);
    height.setFilterFieldName("z");
    height.setFilterLimits(static_cast<float>(min_z_), static_cast<float>(max_z_));
    height.filter(*height_filtered);

    auto without_robot = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::CropBox<pcl::PointXYZ> self_filter;
    self_filter.setInputCloud(height_filtered);
    self_filter.setMin(Eigen::Vector4f(
        static_cast<float>(-self_half_x_), static_cast<float>(-self_half_y_),
        static_cast<float>(self_min_z_), 1.0F));
    self_filter.setMax(Eigen::Vector4f(
        static_cast<float>(self_half_x_), static_cast<float>(self_half_y_),
        static_cast<float>(self_max_z_), 1.0F));
    self_filter.setNegative(true);
    self_filter.filter(*without_robot);

    pcl::PointCloud<pcl::PointXYZ> filtered;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(without_robot);
    const auto leaf = static_cast<float>(voxel_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(filtered, output);
    output.header.stamp = message->header.stamp;
    output.header.frame_id = target_frame_;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double voxel_size_{0.05};
  double min_z_{0.10};
  double max_z_{1.80};
  double self_half_x_{0.398};
  double self_half_y_{0.319};
  double self_min_z_{0.0};
  double self_max_z_{0.45};
  double transform_timeout_sec_{0.05};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace romo_b_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<romo_b_perception::PointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
