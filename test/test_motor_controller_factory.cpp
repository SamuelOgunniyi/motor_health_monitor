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

#include <algorithm>
#include <memory>

#include <gtest/gtest.h>

#include "mock_motor_controller.hpp"
#include "motor_health_monitor/motor_controller.hpp"
#include "motor_health_monitor/motor_controller_factory.hpp"

using namespace motor_health_monitor;
using namespace motor_health_monitor::test;

class MotorControllerFactoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        factory_ = &MotorControllerFactory::getInstance();
    }

    void TearDown() override {
    }

    MotorControllerFactory* factory_;
};

TEST_F(MotorControllerFactoryTest, SingletonPattern) {
    MotorControllerFactory& instance1 = MotorControllerFactory::getInstance();
    MotorControllerFactory& instance2 = MotorControllerFactory::getInstance();
    
    EXPECT_EQ(&instance1, &instance2);
}

TEST_F(MotorControllerFactoryTest, RegisterAndCreate) {
    factory_->registerController("test_controller", []() {
        return std::make_unique<MockMotorController>();
    });
    
    auto controller = factory_->create("test_controller");
    EXPECT_NE(controller, nullptr);
    EXPECT_EQ(controller->getFeedbackValue(), 0.0);
}

TEST_F(MotorControllerFactoryTest, CreateUnknownType) {
    auto controller = factory_->create("unknown_controller");
    EXPECT_EQ(controller, nullptr);
}

TEST_F(MotorControllerFactoryTest, GetRegisteredTypes) {
    factory_->registerController("type1", []() {
        return std::make_unique<MockMotorController>();
    });
    
    factory_->registerController("type2", []() {
        return std::make_unique<MockMotorController>();
    });
    
    auto types = factory_->getRegisteredTypes();
    
    EXPECT_GE(types.size(), 2);
    EXPECT_TRUE(std::find(types.begin(), types.end(), "type1") != types.end());
    EXPECT_TRUE(std::find(types.begin(), types.end(), "type2") != types.end());
}

TEST_F(MotorControllerFactoryTest, OverwriteRegistration) {
    factory_->registerController("overwrite_test", []() {
        auto controller = std::make_unique<MockMotorController>();
        controller->setFeedbackValue(1.0);
        return controller;
    });
    
    auto controller1 = factory_->create("overwrite_test");
    EXPECT_NE(controller1, nullptr);
    EXPECT_EQ(controller1->getFeedbackValue(), 1.0);
    
    factory_->registerController("overwrite_test", []() {
        auto controller = std::make_unique<MockMotorController>();
        controller->setFeedbackValue(2.0);
        return controller;
    });
    
    auto controller2 = factory_->create("overwrite_test");
    EXPECT_NE(controller2, nullptr);
    EXPECT_EQ(controller2->getFeedbackValue(), 2.0);
}

TEST_F(MotorControllerFactoryTest, MultipleInstancesSameType) {
    factory_->registerController("multi_test", []() {
        return std::make_unique<MockMotorController>();
    });
    
    auto controller1 = factory_->create("multi_test");
    auto controller2 = factory_->create("multi_test");
    
    EXPECT_NE(controller1, nullptr);
    EXPECT_NE(controller2, nullptr);
    EXPECT_NE(controller1.get(), controller2.get());
}

TEST_F(MotorControllerFactoryTest, EmptyTypeName) {
    auto controller = factory_->create("");
    EXPECT_EQ(controller, nullptr);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
