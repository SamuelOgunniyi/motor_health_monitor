#include "motor_health_monitor/motor_health_node.hpp"

#include <chrono>

using std::placeholders::_1;

namespace motor_health_monitor
{

MotorHealthNode::MotorHealthNode(const rclcpp::NodeOptions & options)
: Node("motor_health_monitor", options),
  fault_detector_(FaultDetectionConfig{}, motor_controller_)
{
  declare_parameter("sync_tolerance", 0.05);
  declare_parameter("update_rate_ms", 100);
  declare_parameter("fault_limit", 0.95);
  declare_parameter("fault_duration", 0.2);

  sync_ = CmdOdomSync(get_parameter("sync_tolerance").as_double());
  
  FaultDetectionConfig fault_config;
  fault_config.limit = get_parameter("fault_limit").as_double();
  fault_config.duration = get_parameter("fault_duration").as_double();
  fault_detector_.setConfig(fault_config);

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MotorHealthNode::cmdCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&MotorHealthNode::odomCallback, this, _1));

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(get_parameter("update_rate_ms").as_int()),
    std::bind(&MotorHealthNode::update, this));

  RCLCPP_INFO(get_logger(), "Motor Health Monitor started");
}

void MotorHealthNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  cmd_vel_msg_ = msg;
  motor_controller_.setCommand(msg->linear.x);
}

void MotorHealthNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  odom_msg_ = msg;
  motor_controller_.setOdomVelocity(msg->twist.twist.linear.x);
}

void MotorHealthNode::update()
{
  geometry_msgs::msg::Twist::SharedPtr cmd_msg;
  nav_msgs::msg::Odometry::SharedPtr odom_msg;
  bool fault;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd_msg = cmd_vel_msg_;
    odom_msg = odom_msg_;
    
    // Update fault detector (reads from motor_controller_ which holds current velocities)
    auto current_time = now().seconds();
    fault_detector_.update(current_time);
    fault = fault_detector_.getState() == MotorFaultState::FAULT;
  }

  // Extract velocities from messages for sync check
  double cmd_vel = cmd_msg ? cmd_msg->linear.x : 0.0;
  double odom_vel = odom_msg ? odom_msg->twist.twist.linear.x : 0.0;
  auto sync_state = sync_.update(cmd_vel, odom_vel);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "drive_motor";
  status.hardware_id = "drive_motor_0";

  status.level = fault
    ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
    : diagnostic_msgs::msg::DiagnosticStatus::OK;

  status.message = fault ? "Motor fault detected" : "Motor healthy";

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(status);

  diag_pub_->publish(array);
}

} 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motor_health_monitor::MotorHealthNode)
