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

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "motor_health_monitor/motor_health_node.hpp"

using motor_health_monitor::MotorHealthNode;

class TestableMotorHealthNode : public MotorHealthNode {
public:
  explicit TestableMotorHealthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MotorHealthNode(options) {}
  using MotorHealthNode::update;
};

class IntegrationControllerTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestableMotorHealthNode>();
    publisher_node_ = std::make_shared<rclcpp::Node>("test_publisher");
    motor_pub_ = publisher_node_->create_publisher<std_msgs::msg::Float64>("/motor_pwm", 10);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(publisher_node_);
    executor_->add_node(node_->get_node_base_interface());
  }

  void TearDown() override
  {
    executor_.reset();
    motor_pub_.reset();
    publisher_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void publishMotorPWM(double value)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = value;
    motor_pub_->publish(msg);
    executor_->spin_some(std::chrono::milliseconds(100));
  }

  std::shared_ptr<TestableMotorHealthNode> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(IntegrationControllerTest, ConfigureWithRobotDriveController)
{
  auto result = node_->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS,
    result);
}

TEST_F(IntegrationControllerTest, ActivateAndUpdate)
{
  node_->on_configure(rclcpp_lifecycle::State());
  auto activate_result = node_->on_activate(rclcpp_lifecycle::State());
  EXPECT_EQ(
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS,
    activate_result);

  ASSERT_NO_THROW(node_->update());
}

TEST_F(IntegrationControllerTest, MotorPWMSubscription)
{
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());

  publishMotorPWM(0.5);
  ASSERT_NO_THROW(node_->update());

  publishMotorPWM(0.95);
  ASSERT_NO_THROW(node_->update());
}

TEST_F(IntegrationControllerTest, FaultDetectionWithHighPWM)
{
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());

  publishMotorPWM(0.5);
  node_->update();

  publishMotorPWM(0.98);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  ASSERT_NO_THROW(node_->update());
}

TEST_F(IntegrationControllerTest, MultipleMotorUpdates)
{
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());

  for (int i = 0; i < 10; ++i) {
    publishMotorPWM(0.3 + (i * 0.05));
    node_->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
