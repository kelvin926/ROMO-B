#include "romo_b_base/protocol.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "romo_b_msgs/msg/platform_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace romo_b_base
{

class SerialBridge final : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SerialBridge()
  : rclcpp_lifecycle::LifecycleNode("romo_b_serial_bridge"), serial_(io_context_)
  {
    declare_parameter<std::string>("device", "/dev/romo_b_pcu");
    declare_parameter<int>("baud", 115200);
    declare_parameter<int>("data_bits", 8);
    declare_parameter<std::string>("parity", "none");
    declare_parameter<int>("stop_bits", 1);
    declare_parameter<std::string>("command_endian", "unverified");
    declare_parameter<double>("tx_rate_hz", 20.0);
    declare_parameter<double>("command_timeout_sec", 0.15);
    declare_parameter<double>("feedback_timeout_sec", 0.20);
    declare_parameter<double>("auto_transition_timeout_sec", 0.50);
    declare_parameter<double>("wheelbase_m", 0.323);
    declare_parameter<double>("control_track_m", 0.390);
    declare_parameter<double>("wheel_radius_m", 0.103);
    declare_parameter<std::string>("safety_profile", "bench");
    declare_parameter<bool>("receive_only", true);
    declare_parameter<bool>("sensor_calibrated", false);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
  }

  ~SerialBridge() override {close_serial();}

private:
  enum class BridgeState : std::uint8_t
  {
    kDisconnected = romo_b_msgs::msg::PlatformStatus::STATE_DISCONNECTED,
    kConnectedSafe = romo_b_msgs::msg::PlatformStatus::STATE_CONNECTED_SAFE,
    kArmedAuto = romo_b_msgs::msg::PlatformStatus::STATE_ARMED_AUTO,
    kEstop = romo_b_msgs::msg::PlatformStatus::STATE_ESTOP,
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    device_ = get_parameter("device").as_string();
    baud_ = static_cast<int>(get_parameter("baud").as_int());
    data_bits_ = static_cast<int>(get_parameter("data_bits").as_int());
    parity_ = get_parameter("parity").as_string();
    stop_bits_ = static_cast<int>(get_parameter("stop_bits").as_int());
    command_endian_ = get_parameter("command_endian").as_string();
    command_little_endian_ = command_endian_ == "little";
    command_timeout_ = std::chrono::duration<double>(
      get_parameter("command_timeout_sec").as_double());
    feedback_timeout_ = std::chrono::duration<double>(
      get_parameter("feedback_timeout_sec").as_double());
    auto_transition_timeout_ = std::chrono::duration<double>(
      get_parameter("auto_transition_timeout_sec").as_double());
    wheelbase_m_ = get_parameter("wheelbase_m").as_double();
    control_track_m_ = get_parameter("control_track_m").as_double();
    wheel_radius_m_ = get_parameter("wheel_radius_m").as_double();
    receive_only_ = get_parameter("receive_only").as_bool();
    sensor_calibrated_ = get_parameter("sensor_calibrated").as_bool();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    const auto profile = get_parameter("safety_profile").as_string();
    limits_.wheelbase_m = wheelbase_m_;
    limits_.allow_reverse = false;
    if (profile == "bench") {
      navigation_profile_ = false;
      limits_.max_speed_mps = 0.1;
      limits_.max_steer_deg = 5.0;
    } else if (profile == "navigation") {
      navigation_profile_ = true;
      limits_.max_speed_mps = 0.2;
      limits_.max_steer_deg = 22.0;
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown safety_profile '%s'", profile.c_str());
      return CallbackReturn::FAILURE;
    }

    const double tx_rate = get_parameter("tx_rate_hz").as_double();
    if (baud_ <= 0 || data_bits_ < 5 || data_bits_ > 8 || parity_ != "none" ||
      stop_bits_ != 1 ||
      (command_endian_ != "big" && command_endian_ != "little" &&
      command_endian_ != "unverified") ||
      (!receive_only_ && command_endian_ == "unverified") ||
      tx_rate <= 0.0 || command_timeout_.count() <= 0.0 ||
      feedback_timeout_.count() <= 0.0 || auto_transition_timeout_.count() <= 0.0 ||
      wheel_radius_m_ <= 0.0)
    {
      RCLCPP_ERROR(get_logger(), "Invalid serial, timeout, or vehicle parameter");
      return CallbackReturn::FAILURE;
    }

    status_pub_ = create_publisher<romo_b_msgs::msg::PlatformStatus>(
      "/romo_b/platform_status", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/wheel/odometry_raw", rclcpp::SensorDataQoS());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", rclcpp::QoS(10),
      std::bind(&SerialBridge::on_command, this, std::placeholders::_1));
    arm_service_ = create_service<std_srvs::srv::SetBool>(
      "/romo_b/arm",
      std::bind(&SerialBridge::on_arm, this, std::placeholders::_1, std::placeholders::_2));
    estop_service_ = create_service<std_srvs::srv::SetBool>(
      "/romo_b/software_estop",
      std::bind(&SerialBridge::on_estop, this, std::placeholders::_1, std::placeholders::_2));

    tx_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tx_rate)),
      std::bind(&SerialBridge::on_tx_timer, this));

    {
      std::scoped_lock lock(state_mutex_);
      bridge_state_ = BridgeState::kDisconnected;
      software_estop_ = false;
      have_feedback_ = false;
      command_timed_out_ = false;
      feedback_timed_out_ = false;
      auto_confirmed_ = false;
      auto_request_sent_ = false;
      manual_zero_sent_ = false;
      desired_control_ = {};
      hlv_alive_ = 0;
      x_ = 0.0;
      y_ = 0.0;
      yaw_ = 0.0;
      wheel_positions_.fill(0.0);
      const auto steady_now = std::chrono::steady_clock::now();
      last_command_time_ = steady_now;
      last_feedback_time_ = steady_now;
      last_alive_change_time_ = steady_now;
      auto_request_time_ = steady_now;
      last_odom_time_ = steady_now;
    }

    if (!open_serial()) {
      return CallbackReturn::FAILURE;
    }
    {
      std::scoped_lock lock(state_mutex_);
      bridge_state_ = BridgeState::kConnectedSafe;
    }
    RCLCPP_INFO(
      get_logger(), "Configured %s at %d baud (%s, command %s-endian)", device_.c_str(), baud_,
      receive_only_ ? "receive-only" : profile.c_str(), command_endian_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    status_pub_->on_activate();
    odom_pub_->on_activate();
    joint_pub_->on_activate();
    diagnostics_pub_->on_activate();
    {
      std::scoped_lock lock(state_mutex_);
      // A lifecycle deactivate/activate cycle must never clear a latched
      // software E-stop. Only the explicit reset service may do that.
      bridge_state_ = software_estop_ ? BridgeState::kEstop : BridgeState::kConnectedSafe;
      auto_confirmed_ = false;
      auto_request_sent_ = false;
      manual_zero_sent_ = false;
      last_command_time_ = std::chrono::steady_clock::now();
    }
    RCLCPP_INFO(
      get_logger(), "Bridge active, %s",
      software_estop_ ? "software E-stop remains latched" : "but disarmed");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (!receive_only_ && serial_open_.load()) {
      Command command;
      command.auto_mode = true;
      command.estop = true;
      command.alive = hlv_alive_++;
      send_command(command);
    }
    {
      std::scoped_lock lock(state_mutex_);
      bridge_state_ = BridgeState::kEstop;
      software_estop_ = true;
    }
    diagnostics_pub_->on_deactivate();
    joint_pub_->on_deactivate();
    odom_pub_->on_deactivate();
    status_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    close_serial();
    tx_timer_.reset();
    cmd_sub_.reset();
    arm_service_.reset();
    estop_service_.reset();
    status_pub_.reset();
    odom_pub_.reset();
    joint_pub_.reset();
    diagnostics_pub_.reset();
    parser_.clear();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    close_serial();
    return CallbackReturn::SUCCESS;
  }

  bool open_serial()
  {
    try {
      io_context_.restart();
      serial_.open(device_);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_));
      serial_.set_option(boost::asio::serial_port_base::character_size(data_bits_));
      serial_.set_option(boost::asio::serial_port_base::parity(
          boost::asio::serial_port_base::parity::none));
      serial_.set_option(boost::asio::serial_port_base::stop_bits(
          boost::asio::serial_port_base::stop_bits::one));
      serial_.set_option(boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none));
      serial_open_.store(true);
      start_async_read();
      io_thread_ = std::thread([this]() {io_context_.run();});
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot open serial device %s: %s", device_.c_str(), error.what());
      serial_open_.store(false);
      return false;
    }
  }

  void close_serial()
  {
    serial_open_.store(false);
    boost::system::error_code ignored;
    if (serial_.is_open()) {
      serial_.cancel(ignored);
      serial_.close(ignored);
    }
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    std::scoped_lock lock(state_mutex_);
    bridge_state_ = BridgeState::kDisconnected;
  }

  void start_async_read()
  {
    serial_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code & error, std::size_t bytes) {
        if (error) {
          if (error != boost::asio::error::operation_aborted && rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Serial read failed: %s", error.message().c_str());
            serial_open_.store(false);
            std::scoped_lock lock(state_mutex_);
            bridge_state_ = BridgeState::kDisconnected;
            software_estop_ = true;
          }
          return;
        }
        for (const auto & feedback : parser_.push(read_buffer_.data(), bytes)) {
          handle_feedback(feedback);
        }
        if (serial_open_.load()) {
          start_async_read();
        }
      });
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const auto mapped = map_twist(message->linear.x, message->angular.z, limits_);
    std::scoped_lock lock(state_mutex_);
    last_command_time_ = std::chrono::steady_clock::now();
    desired_control_ = mapped.valid ? mapped : ControlOutput{};
    if (!mapped.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected reverse, pure-rotation, non-finite, or otherwise invalid Twist");
    }
  }

  void on_arm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::scoped_lock lock(state_mutex_);
    if (!request->data) {
      desired_control_ = {};
      auto_confirmed_ = false;
      auto_request_sent_ = false;
      manual_zero_sent_ = false;
      bridge_state_ = software_estop_ ? BridgeState::kEstop : BridgeState::kConnectedSafe;
      response->success = true;
      response->message = software_estop_ ?
        "Disarmed; software E-stop remains latched until explicit reset" :
        "Disarmed; zero manual command will be sent";
      return;
    }
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      response->message = "Lifecycle node is not active";
      return;
    }
    if (receive_only_) {
      response->message = "receive_only is enabled";
      return;
    }
    if (navigation_profile_ && !sensor_calibrated_) {
      response->message = "Navigation profile requires an approved LiDAR transform";
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    const auto age = steady_now - last_feedback_time_;
    const auto alive_age = steady_now - last_alive_change_time_;
    if (!serial_open_.load() || !have_feedback_ || age > feedback_timeout_ ||
      alive_age > feedback_timeout_)
    {
      response->message = "Fresh PCU feedback is required";
      return;
    }
    if (latest_feedback_.estop || software_estop_ ||
      latest_feedback_.steer_mode != SteerMode::k2Wis)
    {
      response->message = "PCU must report E-stop off and 2WIS";
      return;
    }
    const bool stopped = std::all_of(
      latest_feedback_.wheel_speed_mps.begin(), latest_feedback_.wheel_speed_mps.end(),
      [](double speed) {return std::abs(speed) < 0.02;});
    if (!stopped) {
      response->message = "PCU must report all wheels stopped before Auto transition";
      return;
    }
    if (!manual_zero_sent_) {
      response->message = "A Manual zero frame must be transmitted before the Auto rising edge";
      return;
    }
    bridge_state_ = BridgeState::kArmedAuto;
    auto_confirmed_ = false;
    auto_request_sent_ = false;
    manual_zero_sent_ = false;
    desired_control_ = {};
    last_command_time_ = steady_now;
    command_timed_out_ = false;
    feedback_timed_out_ = false;
    response->success = true;
    response->message = "Auto rising edge requested; motion stays zero until PCU confirms Auto";
  }

  void on_estop(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::scoped_lock lock(state_mutex_);
    if (request->data) {
      software_estop_ = true;
      bridge_state_ = BridgeState::kEstop;
      auto_confirmed_ = false;
      auto_request_sent_ = false;
      desired_control_ = {};
      response->success = true;
      response->message = "Software E-stop latched";
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    const auto age = steady_now - last_feedback_time_;
    const auto alive_age = steady_now - last_alive_change_time_;
    const bool stopped = std::all_of(
      latest_feedback_.wheel_speed_mps.begin(), latest_feedback_.wheel_speed_mps.end(),
      [](double speed) {return std::abs(speed) < 0.02;});
    if (!serial_open_.load() || !have_feedback_ || age > feedback_timeout_ ||
      alive_age > feedback_timeout_ || !stopped)
    {
      response->message = "Cannot reset: fresh stopped feedback is required";
      return;
    }
    software_estop_ = false;
    bridge_state_ = BridgeState::kConnectedSafe;
    auto_confirmed_ = false;
    auto_request_sent_ = false;
    manual_zero_sent_ = false;
    command_timed_out_ = false;
    feedback_timed_out_ = false;
    response->success = true;
    response->message = "Software E-stop reset; bridge remains disarmed";
  }

  void on_tx_timer()
  {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    Command command;
    {
      std::scoped_lock lock(state_mutex_);
      const auto steady_now = std::chrono::steady_clock::now();
      feedback_timed_out_ = !have_feedback_ ||
        steady_now - last_feedback_time_ > feedback_timeout_ ||
        steady_now - last_alive_change_time_ > feedback_timeout_;
      if (bridge_state_ == BridgeState::kArmedAuto) {
        command_timed_out_ = auto_confirmed_ &&
          steady_now - last_command_time_ > command_timeout_;
        const bool auto_transition_timed_out = auto_request_sent_ && !auto_confirmed_ &&
          steady_now - auto_request_time_ > auto_transition_timeout_;
        if (command_timed_out_ || feedback_timed_out_ || auto_transition_timed_out) {
          if (command_timed_out_) {
            RCLCPP_ERROR(
              get_logger(), "Command watchdog expired after %.3f s; latching HLV E-stop",
              std::chrono::duration<double>(steady_now - last_command_time_).count());
          } else if (feedback_timed_out_) {
            RCLCPP_ERROR(get_logger(), "PCU feedback watchdog expired; latching HLV E-stop");
          } else if (auto_transition_timed_out) {
            RCLCPP_ERROR(get_logger(), "PCU did not confirm the Auto transition in time");
          }
          software_estop_ = true;
          bridge_state_ = BridgeState::kEstop;
          auto_confirmed_ = false;
          auto_request_sent_ = false;
          desired_control_ = {};
        }
      } else if (bridge_state_ == BridgeState::kConnectedSafe) {
        if (software_estop_) {
          bridge_state_ = BridgeState::kEstop;
        }
        command_timed_out_ = false;
      }

      command.steer_mode = SteerMode::k2Wis;
      command.alive = hlv_alive_++;
      if (bridge_state_ == BridgeState::kArmedAuto) {
        command.auto_mode = true;
        if (auto_confirmed_) {
          command.speed_mps = desired_control_.speed_mps;
          command.steer_deg = desired_control_.pcu_steer_deg;
        }
      } else if (bridge_state_ == BridgeState::kEstop) {
        command.auto_mode = true;
        command.estop = true;
      }
    }

    bool sent = false;
    if (!receive_only_) {
      sent = send_command(command);
    }
    if (sent && !command.auto_mode && !command.estop) {
      std::scoped_lock lock(state_mutex_);
      manual_zero_sent_ = true;
    } else if (sent && command.auto_mode && !command.estop) {
      std::scoped_lock lock(state_mutex_);
      if (bridge_state_ == BridgeState::kArmedAuto && !auto_request_sent_) {
        auto_request_sent_ = true;
        auto_request_time_ = std::chrono::steady_clock::now();
      }
    }
    publish_status_and_diagnostics();
  }

  bool send_command(const Command & command)
  {
    if (!serial_open_.load()) {
      return false;
    }
    const auto frame = encode_command(command, command_little_endian_);
    try {
      std::scoped_lock lock(write_mutex_);
      boost::asio::write(serial_, boost::asio::buffer(frame));
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Serial write failed: %s", error.what());
      serial_open_.store(false);
      std::scoped_lock state_lock(state_mutex_);
      bridge_state_ = BridgeState::kDisconnected;
      software_estop_ = true;
      return false;
    }
  }

  void handle_feedback(const Feedback & feedback)
  {
    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::JointState joints;
    const auto stamp = now();
    const auto steady_now = std::chrono::steady_clock::now();

    {
      std::scoped_lock lock(state_mutex_);
      if (!have_feedback_ || feedback.alive != latest_feedback_.alive) {
        last_alive_change_time_ = steady_now;
      }
      latest_feedback_ = feedback;
      have_feedback_ = true;
      last_feedback_time_ = steady_now;
      if (bridge_state_ != BridgeState::kEstop &&
        steady_now - last_alive_change_time_ <= feedback_timeout_)
      {
        feedback_timed_out_ = false;
      }
      if (bridge_state_ == BridgeState::kArmedAuto) {
        if (feedback.estop || feedback.steer_mode != SteerMode::k2Wis ||
          (auto_confirmed_ && !feedback.auto_mode))
        {
          software_estop_ = true;
          auto_confirmed_ = false;
          auto_request_sent_ = false;
          bridge_state_ = BridgeState::kEstop;
        } else if (!auto_confirmed_ && auto_request_sent_ && feedback.auto_mode) {
          auto_confirmed_ = true;
          desired_control_ = {};
          last_command_time_ = steady_now;
          RCLCPP_INFO(get_logger(), "PCU confirmed Auto; forward commands are now enabled");
        }
      }

      const auto motion = estimate_vehicle_motion(feedback, wheelbase_m_, control_track_m_);
      const double dt = std::chrono::duration<double>(steady_now - last_odom_time_).count();
      last_odom_time_ = steady_now;
      if (dt > 0.0 && dt < 0.5) {
        yaw_ += motion.yaw_rate_radps * dt;
        x_ += motion.center_speed_mps * std::cos(yaw_) * dt;
        y_ += motion.center_speed_mps * std::sin(yaw_) * dt;
        for (std::size_t index = 0; index < wheel_positions_.size(); ++index) {
          wheel_positions_[index] += feedback.wheel_speed_mps[index] / wheel_radius_m_ * dt;
        }
      }

      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
      odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
      odom.twist.twist.linear.x = motion.center_speed_mps;
      odom.twist.twist.angular.z = motion.yaw_rate_radps;
      odom.pose.covariance[0] = 0.05;
      odom.pose.covariance[7] = 0.05;
      odom.pose.covariance[35] = 0.10;
      odom.twist.covariance[0] = 0.02;
      odom.twist.covariance[35] = 0.05;

      joints.header.stamp = stamp;
      joints.name = {
        "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint",
        "fl_steer_joint", "fr_steer_joint", "rl_steer_joint", "rr_steer_joint"};
      joints.position.resize(8, 0.0);
      joints.velocity.resize(8, 0.0);
      for (std::size_t index = 0; index < 4; ++index) {
        joints.position[index] = wheel_positions_[index];
        joints.velocity[index] = feedback.wheel_speed_mps[index] / wheel_radius_m_;
        joints.position[index + 4] = -feedback.wheel_steer_rad[index];
      }
    }

    if (odom_pub_ && odom_pub_->is_activated()) {
      odom_pub_->publish(odom);
      joint_pub_->publish(joints);
    }
  }

  void publish_status_and_diagnostics()
  {
    romo_b_msgs::msg::PlatformStatus status;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostic_msgs::msg::DiagnosticStatus serial_status;
    bool auto_confirmed = false;
    bool manual_zero_sent = false;
    double desired_speed_mps = 0.0;
    double desired_steer_deg = 0.0;
    double command_age_sec = 0.0;
    status.header.stamp = now();
    status.header.frame_id = base_frame_;
    diagnostics.header = status.header;
    serial_status.name = "ROMO-B/PCU serial bridge";
    serial_status.hardware_id = device_;

    {
      std::scoped_lock lock(state_mutex_);
      status.state = static_cast<std::uint8_t>(bridge_state_);
      status.connected = serial_open_.load();
      status.auto_mode = have_feedback_ && latest_feedback_.auto_mode;
      status.estop = software_estop_ || (have_feedback_ && latest_feedback_.estop);
      status.steer_mode = have_feedback_ ?
        static_cast<std::uint8_t>(latest_feedback_.steer_mode) : 0U;
      for (std::size_t index = 0; index < 4; ++index) {
        status.wheel_speed_mps[index] = static_cast<float>(latest_feedback_.wheel_speed_mps[index]);
        status.wheel_steer_rad[index] = static_cast<float>(-latest_feedback_.wheel_steer_rad[index]);
      }
      status.pcu_alive = latest_feedback_.alive;
      status.hlv_alive = hlv_alive_;
      status.command_timed_out = command_timed_out_;
      status.feedback_timed_out = feedback_timed_out_;
      auto_confirmed = auto_confirmed_;
      manual_zero_sent = manual_zero_sent_;
      desired_speed_mps = desired_control_.speed_mps;
      desired_steer_deg = desired_control_.pcu_steer_deg;
      command_age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_command_time_).count();
    }

    if (!status.connected || status.estop || status.command_timed_out || status.feedback_timed_out) {
      serial_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (status.command_timed_out) {
        serial_status.message = "Command timeout E-stop";
      } else if (status.feedback_timed_out) {
        serial_status.message = "PCU feedback timeout E-stop";
      } else {
        serial_status.message = status.estop ? "E-stop active" : "Serial fault";
      }
    } else if (receive_only_) {
      serial_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      serial_status.message = "Receive-only validation mode";
    } else {
      serial_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      serial_status.message = status.state == romo_b_msgs::msg::PlatformStatus::STATE_ARMED_AUTO ?
        "Armed 2WIS" : "Connected and disarmed";
    }
    const auto add_value = [&serial_status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        serial_status.values.push_back(item);
      };
    add_value("device", device_);
    add_value("receive_only", receive_only_ ? "true" : "false");
    add_value("command_endian", command_endian_);
    add_value("sensor_calibrated", sensor_calibrated_ ? "true" : "false");
    add_value("auto_confirmed", auto_confirmed ? "true" : "false");
    add_value("manual_zero_sent", manual_zero_sent ? "true" : "false");
    add_value("commanded_speed_mps", std::to_string(desired_speed_mps));
    add_value(
      "commanded_speed_raw",
      std::to_string(static_cast<int>(std::lround(desired_speed_mps * 100.0))));
    add_value("commanded_steer_deg", std::to_string(desired_steer_deg));
    add_value("command_age_sec", std::to_string(command_age_sec));
    add_value("command_timeout_sec", std::to_string(command_timeout_.count()));
    add_value("pcu_alive", std::to_string(status.pcu_alive));
    add_value("hlv_alive", std::to_string(status.hlv_alive));
    diagnostics.status.push_back(serial_status);

    if (status_pub_ && status_pub_->is_activated()) {
      status_pub_->publish(status);
      diagnostics_pub_->publish(diagnostics);
    }
  }

  std::string device_;
  int baud_{115200};
  int data_bits_{8};
  std::string parity_{"none"};
  int stop_bits_{1};
  std::string command_endian_{"unverified"};
  bool command_little_endian_{false};
  bool receive_only_{true};
  bool navigation_profile_{false};
  bool sensor_calibrated_{false};
  double wheelbase_m_{0.323};
  double control_track_m_{0.390};
  double wheel_radius_m_{0.103};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::chrono::duration<double> command_timeout_{0.15};
  std::chrono::duration<double> feedback_timeout_{0.20};
  std::chrono::duration<double> auto_transition_timeout_{0.50};
  ControlLimits limits_;

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_;
  std::thread io_thread_;
  std::array<std::uint8_t, 512> read_buffer_{};
  FeedbackParser parser_;
  std::atomic_bool serial_open_{false};
  std::mutex write_mutex_;

  std::mutex state_mutex_;
  BridgeState bridge_state_{BridgeState::kDisconnected};
  bool software_estop_{false};
  bool have_feedback_{false};
  bool command_timed_out_{false};
  bool feedback_timed_out_{false};
  bool auto_confirmed_{false};
  bool auto_request_sent_{false};
  bool manual_zero_sent_{false};
  Feedback latest_feedback_;
  ControlOutput desired_control_;
  std::uint8_t hlv_alive_{0};
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point last_feedback_time_;
  std::chrono::steady_clock::time_point last_alive_change_time_;
  std::chrono::steady_clock::time_point auto_request_time_;
  std::chrono::steady_clock::time_point last_odom_time_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  std::array<double, 4> wheel_positions_{};

  rclcpp_lifecycle::LifecyclePublisher<romo_b_msgs::msg::PlatformStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_service_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
};

}  // namespace romo_b_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<romo_b_base::SerialBridge>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
