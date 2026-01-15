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
#include <rclcpp/rclcpp.hpp>

#include "motor_health_monitor/motor_health_node.hpp"

using motor_health_monitor::MotorHealthNode;

class TestableMotorHealthNode : public MotorHealthNode {
public:
  explicit TestableMotorHealthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MotorHealthNode(options) {}
  using MotorHealthNode::update;
};

class MotorHealthNodeTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestableMotorHealthNode>();
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TestableMotorHealthNode> node_;
};

TEST_F(MotorHealthNodeTest, UpdateWhenInactiveDoesNothing)
{
  // Node starts unconfigured, update should return early
  // This tests the early return in update()
  ASSERT_NO_THROW(node_->update());
}

TEST_F(MotorHealthNodeTest, UpdateWhenConfigured)
{
  // Configure node - update should still return early (not activated)
  auto result = node_->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS,
    result);
  ASSERT_NO_THROW(node_->update());
}

TEST_F(MotorHealthNodeTest, UpdateWhenActivated)
{
  // Configure and activate node
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());

  // Update should process without errors
  // This exercises all helper functions: checkStaleData(), updateFaultDetection(),
  // determineDiagnosticLevel(), and buildDiagnosticStatus()
  ASSERT_NO_THROW(node_->update());
}

TEST_F(MotorHealthNodeTest, UpdateHandlesNoMessages)
{
  // Configure and activate
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());

  // Update with no messages should handle gracefully
  // Tests checkStaleData() with null messages
  ASSERT_NO_THROW(node_->update());
}

TEST_F(MotorHealthNodeTest, MultipleUpdates)
{
  // Configure and activate
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());

  // Multiple updates should work correctly
  ASSERT_NO_THROW(node_->update());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_NO_THROW(node_->update());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_NO_THROW(node_->update());
}

TEST_F(MotorHealthNodeTest, UpdateAfterDeactivate)
{
  // Configure, activate, then deactivate
  node_->on_configure(rclcpp_lifecycle::State());
  node_->on_activate(rclcpp_lifecycle::State());
  node_->on_deactivate(rclcpp_lifecycle::State());

  // Update should return early after deactivation
  ASSERT_NO_THROW(node_->update());
}
