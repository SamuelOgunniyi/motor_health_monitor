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
#include <cmath>

#include <gtest/gtest.h>

#include "mock_motor_controller.hpp"
#include "motor_health_monitor/motor_fault_detector.hpp"

using namespace motor_health_monitor;
using namespace motor_health_monitor::test;

class MotorFaultDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_controller_ = std::make_unique<MockMotorController>();
        config_.threshold = 0.95;
        config_.duration = 0.2;
        config_.type = ThresholdType::ABSOLUTE_VALUE;
        detector_ = std::make_unique<MotorFaultDetector>(
            config_, *mock_controller_);
    }

    void TearDown() override {
        detector_.reset();
        mock_controller_.reset();
    }

    FaultDetectionConfig config_;
    std::unique_ptr<MockMotorController> mock_controller_;
    std::unique_ptr<MotorFaultDetector> detector_;
};

TEST_F(MotorFaultDetectorTest, InitialState) {
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, NoFaultBelowLimit) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(0.5);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(1000);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, NoFaultAtLimit) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(0.95);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, FaultAfterDuration) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(1.0);
    
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(100);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t2 = t0 + std::chrono::milliseconds(200);
    detector_->update(t2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
    
    auto t3 = t0 + std::chrono::milliseconds(300);
    detector_->update(t3);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, TimerResetOnDrop) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(1.0);
    
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(100);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(0.5);
    auto t2 = t0 + std::chrono::milliseconds(150);
    detector_->update(t2);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(1.0);
    auto t3 = t0 + std::chrono::milliseconds(200);
    detector_->update(t3);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t4 = t0 + std::chrono::milliseconds(400);
    detector_->update(t4);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, NegativeFeedback) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(-1.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, SetConfig) {
    FaultDetectionConfig new_config;
    new_config.threshold = 0.5;
    new_config.duration = 0.1;
    new_config.type = ThresholdType::ABSOLUTE_VALUE;
    
    detector_->setConfig(new_config);
    
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(0.6);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(100);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LargeFeedbackValues) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(100.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, RapidStateChanges) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(1.0);
    detector_->update(t0);
    
    mock_controller_->setFeedbackValue(0.5);
    auto t1 = t0 + std::chrono::milliseconds(50);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(1.0);
    auto t2 = t0 + std::chrono::milliseconds(100);
    detector_->update(t2);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(0.5);
    auto t3 = t0 + std::chrono::milliseconds(150);
    detector_->update(t3);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, BoundaryAtLimit) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(0.95);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, BoundaryJustAboveLimit) {
    auto t0 = std::chrono::steady_clock::now();
    mock_controller_->setFeedbackValue(0.9501);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, UpperLimitThreshold) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = 5.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(4.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(5.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(5.1);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, UpperLimitBelowThreshold) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = 5.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(4.9);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(1000);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, UpperLimitNegativeValues) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = -1.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(-2.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(-0.5);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LowerLimitThreshold) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = 2.0;
    config_.type = ThresholdType::LOWER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(3.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(2.0);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(1.9);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LowerLimitAboveThreshold) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = 2.0;
    config_.type = ThresholdType::LOWER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(2.1);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(1000);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LowerLimitNegativeValues) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = -1.0;
    config_.type = ThresholdType::LOWER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(-0.5);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(-1.5);
    detector_->update(t0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, ThresholdTypeSwitching) {
    auto t0 = std::chrono::steady_clock::now();
    config_.threshold = 5.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(6.0);
    detector_->update(t0);
    auto t1 = t0 + std::chrono::milliseconds(200);
    detector_->update(t1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
    
    FaultDetectionConfig new_config;
    new_config.threshold = 5.0;
    new_config.duration = 0.2;
    new_config.type = ThresholdType::LOWER_LIMIT;
    detector_->setConfig(new_config);
    
    auto t2 = t0 + std::chrono::milliseconds(300);
    mock_controller_->setFeedbackValue(6.0);
    detector_->update(t2);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(4.0);
    detector_->update(t2);
    auto t3 = t0 + std::chrono::milliseconds(500);
    detector_->update(t3);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
