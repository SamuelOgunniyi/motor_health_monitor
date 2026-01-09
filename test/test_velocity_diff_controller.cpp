#include <gtest/gtest.h>
#include "motor_health_monitor/velocity_diff_controller.hpp"
#include <cmath>

using namespace motor_health_monitor;

class VelocityDiffControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<VelocityDiffController>();
    }

    void TearDown() override {
        controller_.reset();
    }

    std::unique_ptr<VelocityDiffController> controller_;
};

TEST(VelocityDiffControllerConstructorTest, DefaultConstructor) {
    VelocityDiffController controller;
    EXPECT_DOUBLE_EQ(0.0, controller.getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, SetCommand) {
    controller_->setCommand(1.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, SetOdomVelocity) {
    controller_->setOdomVelocity(1.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, MatchingVelocities) {
    controller_->setCommand(1.0);
    controller_->setOdomVelocity(1.0);
    EXPECT_DOUBLE_EQ(0.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, DifferentVelocities) {
    controller_->setCommand(2.0);
    controller_->setOdomVelocity(1.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, AbsoluteValue) {
    controller_->setCommand(1.0);
    controller_->setOdomVelocity(2.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
    
    controller_->setCommand(2.0);
    controller_->setOdomVelocity(1.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, NegativeVelocities) {
    controller_->setCommand(-1.0);
    controller_->setOdomVelocity(-1.0);
    EXPECT_DOUBLE_EQ(0.0, controller_->getFeedbackValue());
    
    controller_->setCommand(-2.0);
    controller_->setOdomVelocity(-1.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, OppositeSignVelocities) {
    controller_->setCommand(1.0);
    controller_->setOdomVelocity(-1.0);
    EXPECT_DOUBLE_EQ(2.0, controller_->getFeedbackValue());
    
    controller_->setCommand(-1.0);
    controller_->setOdomVelocity(1.0);
    EXPECT_DOUBLE_EQ(2.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, ZeroValues) {
    controller_->setCommand(0.0);
    controller_->setOdomVelocity(0.0);
    EXPECT_DOUBLE_EQ(0.0, controller_->getFeedbackValue());
    
    controller_->setCommand(0.0);
    controller_->setOdomVelocity(1.0);
    EXPECT_DOUBLE_EQ(1.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, LargeValues) {
    controller_->setCommand(100.0);
    controller_->setOdomVelocity(105.0);
    EXPECT_DOUBLE_EQ(5.0, controller_->getFeedbackValue());
}

TEST_F(VelocityDiffControllerTest, SmallValues) {
    controller_->setCommand(0.001);
    controller_->setOdomVelocity(0.002);
    EXPECT_NEAR(0.001, controller_->getFeedbackValue(), 1e-9);
}

TEST_F(VelocityDiffControllerTest, ConstCorrectness) {
    controller_->setCommand(1.0);
    controller_->setOdomVelocity(2.0);
    
    const VelocityDiffController& const_controller = *controller_;
    EXPECT_DOUBLE_EQ(1.0, const_controller.getFeedbackValue());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
