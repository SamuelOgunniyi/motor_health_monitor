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
    mock_controller_->setFeedbackValue(0.5);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(1.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, NoFaultAtLimit) {
    mock_controller_->setFeedbackValue(0.95);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, FaultAfterDuration) {
    mock_controller_->setFeedbackValue(1.0);
    
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
    
    detector_->update(0.3);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, TimerResetOnDrop) {
    mock_controller_->setFeedbackValue(1.0);
    
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(0.5);
    detector_->update(0.15);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(1.0);
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.4);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, NegativeFeedback) {
    mock_controller_->setFeedbackValue(-1.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, SetConfig) {
    FaultDetectionConfig new_config;
    new_config.threshold = 0.5;
    new_config.duration = 0.1;
    new_config.type = ThresholdType::ABSOLUTE_VALUE;
    
    detector_->setConfig(new_config);
    
    mock_controller_->setFeedbackValue(0.6);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.1);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LargeFeedbackValues) {
    mock_controller_->setFeedbackValue(100.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, RapidStateChanges) {
    mock_controller_->setFeedbackValue(1.0);
    detector_->update(0.0);
    
    mock_controller_->setFeedbackValue(0.5);
    detector_->update(0.05);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(1.0);
    detector_->update(0.1);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(0.5);
    detector_->update(0.15);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, BoundaryAtLimit) {
    mock_controller_->setFeedbackValue(0.95);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, BoundaryJustAboveLimit) {
    mock_controller_->setFeedbackValue(0.9501);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, UpperLimitThreshold) {
    config_.threshold = 5.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(4.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(5.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(5.1);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, UpperLimitBelowThreshold) {
    config_.threshold = 5.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(4.9);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(1.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, UpperLimitNegativeValues) {
    config_.threshold = -1.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(-2.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(-0.5);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LowerLimitThreshold) {
    config_.threshold = 2.0;
    config_.type = ThresholdType::LOWER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(3.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(2.0);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(1.9);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LowerLimitAboveThreshold) {
    config_.threshold = 2.0;
    config_.type = ThresholdType::LOWER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(2.1);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(1.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, LowerLimitNegativeValues) {
    config_.threshold = -1.0;
    config_.type = ThresholdType::LOWER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(-0.5);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(-1.5);
    detector_->update(0.0);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

TEST_F(MotorFaultDetectorTest, ThresholdTypeSwitching) {
    config_.threshold = 5.0;
    config_.type = ThresholdType::UPPER_LIMIT;
    detector_ = std::make_unique<MotorFaultDetector>(config_, *mock_controller_);
    
    mock_controller_->setFeedbackValue(6.0);
    detector_->update(0.0);
    detector_->update(0.2);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
    
    FaultDetectionConfig new_config;
    new_config.threshold = 5.0;
    new_config.duration = 0.2;
    new_config.type = ThresholdType::LOWER_LIMIT;
    detector_->setConfig(new_config);
    
    mock_controller_->setFeedbackValue(6.0);
    detector_->update(0.3);
    EXPECT_EQ(MotorFaultState::NO_FAULT, detector_->getState());
    
    mock_controller_->setFeedbackValue(4.0);
    detector_->update(0.3);
    detector_->update(0.5);
    EXPECT_EQ(MotorFaultState::FAULT, detector_->getState());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
