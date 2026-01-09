#include <gtest/gtest.h>
#include "motor_health_monitor/motor_fault_detector.hpp"
#include "mock_motor_controller.hpp"
#include <cmath>

using namespace motor_health_monitor;
using namespace motor_health_monitor::test;

class MotorFaultDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_controller_ = std::make_unique<MockMotorController>();
        config_.limit = 0.95;
        config_.duration = 0.2;
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
    new_config.limit = 0.5;
    new_config.duration = 0.1;
    
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

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
