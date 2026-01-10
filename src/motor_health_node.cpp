// Copyright 2025 Samuel Ogunniyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "motor_health_monitor/motor_health_node.hpp"

#include <chrono>

#include <lifecycle_msgs/msg/state.hpp>

#include "motor_health_monitor/robot_drive_motor_controller.hpp"

using std::placeholders::_1;

namespace motor_health_monitor
{

MotorHealthNode::MotorHealthNode(const rclcpp::NodeOptions & options)
: LifecycleNode("motor_health_monitor", options),
  cmd_vel_timestamp_(std::chrono::steady_clock::now()),
  odom_timestamp_(std::chrono::steady_clock::now())
{
  declare_parameter("sync_tolerance", 0.05);
  declare_parameter("update_rate_ms", 100);
  declare_parameter("fault_threshold", 0.95);
  declare_parameter("fault_duration", 0.2);
  declare_parameter("fault_threshold_type", "absolute_value");
  declare_parameter("pwm_topic", "motor_pwm");
  declare_parameter("motor_controller_type", "robot_drive");
  declare_parameter("motor_message_type", "");
  declare_parameter("motor_pwm_field", "");
  declare_parameter("stale_data_timeout", 1.0);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Motor Health Monitor");

  std::string controller_type = get_parameter("motor_controller_type").as_string();
  std::string pwm_topic = get_parameter("pwm_topic").as_string();
  std::string msg_type_override = get_parameter("motor_message_type").as_string();
  std::string field_path_override = get_parameter("motor_pwm_field").as_string();

  auto& factory = MotorControllerFactory::getInstance();
  motor_controller_ = factory.create(controller_type);

  if (!motor_controller_) {
    auto registered_types = factory.getRegisteredTypes();
    std::string types_list;
    for (size_t i = 0; i < registered_types.size(); ++i) {
      if (i > 0) types_list += ", ";
      types_list += "'" + registered_types[i] + "'";
    }

    RCLCPP_ERROR(get_logger(),
                 "Unknown motor controller type: '%s'. Registered types: %s",
                 controller_type.c_str(), types_list.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Using motor controller type: '%s'", controller_type.c_str());

  if (!motor_controller_->configure(this, pwm_topic, msg_type_override, field_path_override)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure motor controller");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  FaultDetectionConfig fault_config;
  fault_config.threshold = get_parameter("fault_threshold").as_double();
  fault_config.duration = get_parameter("fault_duration").as_double();

  std::string threshold_type_str = get_parameter("fault_threshold_type").as_string();
  if (threshold_type_str == "upper_limit") {
    fault_config.type = ThresholdType::UPPER_LIMIT;
  } else if (threshold_type_str == "lower_limit") {
    fault_config.type = ThresholdType::LOWER_LIMIT;
  } else {
    fault_config.type = ThresholdType::ABSOLUTE_VALUE;
  }

  fault_detector_ = std::make_unique<MotorFaultDetector>(fault_config, *motor_controller_);

  sync_ = CmdOdomSync(get_parameter("sync_tolerance").as_double());
  stale_data_timeout_ = get_parameter("stale_data_timeout").as_double();

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MotorHealthNode::cmdCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&MotorHealthNode::odomCallback, this, _1));

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(get_parameter("update_rate_ms").as_int()),
    std::bind(&MotorHealthNode::update, this));
  timer_->cancel();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating Motor Health Monitor");

  diag_pub_->on_activate();
  timer_->reset();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating Motor Health Monitor");

  timer_->cancel();
  diag_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Motor Health Monitor");

  cmd_sub_.reset();
  odom_sub_.reset();
  if (motor_controller_) {
    motor_controller_->cleanup();
  }
  diag_pub_.reset();
  timer_.reset();
  motor_controller_.reset();
  fault_detector_.reset();

  cmd_vel_msg_.reset();
  odom_msg_.reset();
  cmd_vel_timestamp_ = std::chrono::steady_clock::now();
  odom_timestamp_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "Motor Health Monitor cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorHealthNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down Motor Health Monitor (state: %s)",
      state.label().c_str());

  cmd_sub_.reset();
  odom_sub_.reset();
  if (motor_controller_) {
    motor_controller_->cleanup();
  }
  diag_pub_.reset();
  timer_.reset();
  motor_controller_.reset();
  fault_detector_.reset();

  cmd_vel_msg_.reset();
  odom_msg_.reset();
  cmd_vel_timestamp_ = std::chrono::steady_clock::now();
  odom_timestamp_ = std::chrono::steady_clock::now();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MotorHealthNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  cmd_vel_msg_ = msg;
  cmd_vel_timestamp_ = std::chrono::steady_clock::now();
}

void MotorHealthNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  odom_msg_ = msg;
  odom_timestamp_ = std::chrono::steady_clock::now();
}

void MotorHealthNode::update()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  auto current_time_monotonic = std::chrono::steady_clock::now();
  auto current_time_ros = now();

  geometry_msgs::msg::Twist::SharedPtr cmd_msg = cmd_vel_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg = odom_msg_;

  bool cmd_stale = false;
  bool odom_stale = false;
  if (cmd_msg) {
    auto cmd_age = std::chrono::duration<double>(
      current_time_monotonic - cmd_vel_timestamp_).count();
    cmd_stale = cmd_age > stale_data_timeout_;
  }
  if (odom_msg) {
    auto odom_age = std::chrono::duration<double>(
      current_time_monotonic - odom_timestamp_).count();
    odom_stale = odom_age > stale_data_timeout_;
  }

  bool fault = false;
  std::string fault_reason;
  double feedback_value = 0.0;
  if (fault_detector_) {
    fault_detector_->update(current_time_monotonic);
    fault = fault_detector_->getState() == MotorFaultState::FAULT;
    feedback_value = fault_detector_->getCurrentFeedbackValue();
    const auto& config = fault_detector_->getConfig();

    if (fault) {
      std::string threshold_type_str;
      switch (config.type) {
        case ThresholdType::ABSOLUTE_VALUE:
          threshold_type_str = "absolute";
          fault_reason = "Feedback value |" + std::to_string(feedback_value) +
            "| >= threshold " + std::to_string(config.threshold) +
            " for " + std::to_string(config.duration) + "s";
          break;
        case ThresholdType::UPPER_LIMIT:
          threshold_type_str = "upper_limit";
          fault_reason = "Feedback value " + std::to_string(feedback_value) +
            " >= upper limit " + std::to_string(config.threshold) +
            " for " + std::to_string(config.duration) + "s";
          break;
        case ThresholdType::LOWER_LIMIT:
          threshold_type_str = "lower_limit";
          fault_reason = "Feedback value " + std::to_string(feedback_value) +
            " <= lower limit " + std::to_string(config.threshold) +
            " for " + std::to_string(config.duration) + "s";
          break;
      }
    }
  }

  double cmd_vel = (cmd_msg && !cmd_stale) ? cmd_msg->linear.x : 0.0;
  double odom_vel = (odom_msg && !odom_stale) ? odom_msg->twist.twist.linear.x : 0.0;
  sync_.update(cmd_vel, odom_vel);

  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "Motor healthy";
  if (fault) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "Motor fault detected: " + fault_reason;
  } else if (cmd_stale || odom_stale) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    std::string stale_sources;
    if (cmd_stale) stale_sources += "cmd_vel";
    if (odom_stale) {
      if (!stale_sources.empty()) stale_sources += ", ";
      stale_sources += "odom";
    }
    diag_message = "Stale data detected (" + stale_sources + "), using defaults";
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "drive_motor";
  status.hardware_id = "drive_motor_0";
  status.level = diag_level;
  status.message = diag_message;

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "feedback_value";
  kv.value = std::to_string(feedback_value);
  status.values.push_back(kv);

  if (fault_detector_) {
    const auto& config = fault_detector_->getConfig();
    kv.key = "fault_threshold";
    kv.value = std::to_string(config.threshold);
    status.values.push_back(kv);

    kv.key = "fault_duration";
    kv.value = std::to_string(config.duration);
    status.values.push_back(kv);

    std::string threshold_type_str;
    switch (config.type) {
      case ThresholdType::ABSOLUTE_VALUE:
        threshold_type_str = "absolute_value";
        break;
      case ThresholdType::UPPER_LIMIT:
        threshold_type_str = "upper_limit";
        break;
      case ThresholdType::LOWER_LIMIT:
        threshold_type_str = "lower_limit";
        break;
    }
    kv.key = "threshold_type";
    kv.value = threshold_type_str;
    status.values.push_back(kv);
  }

  kv.key = "cmd_vel_age";
  kv.value = cmd_msg ? std::to_string(std::chrono::duration<double>(
    current_time_monotonic - cmd_vel_timestamp_).count()) : "N/A";
  status.values.push_back(kv);

  kv.key = "odom_age";
  kv.value = odom_msg ? std::to_string(std::chrono::duration<double>(
    current_time_monotonic - odom_timestamp_).count()) : "N/A";
  status.values.push_back(kv);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = current_time_ros;
  array.status.push_back(status);

  if (diag_pub_->is_activated()) {
    diag_pub_->publish(array);
  }
}

}  // namespace motor_health_monitor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motor_health_monitor::MotorHealthNode)
