#include <gtest/gtest.h>
#include "motor_health_monitor/cmd_odom_sync.hpp"

using namespace motor_health_monitor;

class CmdOdomSyncTest : public ::testing::Test {
protected:
    void SetUp() override {
        sync_ = std::make_unique<CmdOdomSync>(0.1);
    }

    void TearDown() override {
        sync_.reset();
    }

    std::unique_ptr<CmdOdomSync> sync_;
};

TEST(CmdOdomSyncConstructorTest, DefaultTolerance) {
    CmdOdomSync sync;
    EXPECT_EQ(CmdOdomSync::SYNCED, sync.update(0.0, 0.0));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync.update(0.0, 0.04));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync.update(0.0, 0.06));
}

TEST(CmdOdomSyncConstructorTest, CustomTolerance) {
    CmdOdomSync sync(0.2);
    EXPECT_EQ(CmdOdomSync::SYNCED, sync.update(1.0, 1.15));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync.update(1.0, 1.25));
}

TEST_F(CmdOdomSyncTest, SyncedState) {
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(1.0, 1.0));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(1.0, 1.05));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(1.0, 0.95));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(-1.0, -1.05));
}

TEST_F(CmdOdomSyncTest, UnsyncedState) {
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(1.0, 1.2));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(1.0, 0.8));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(0.0, 0.15));
}

TEST_F(CmdOdomSyncTest, IndeterminateNaN) {
    double nan_val = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(CmdOdomSync::INDETERMINATE, sync_->update(nan_val, 1.0));
    EXPECT_EQ(CmdOdomSync::INDETERMINATE, sync_->update(1.0, nan_val));
    EXPECT_EQ(CmdOdomSync::INDETERMINATE, sync_->update(nan_val, nan_val));
}

TEST_F(CmdOdomSyncTest, IndeterminateInf) {
    double inf_val = std::numeric_limits<double>::infinity();
    EXPECT_EQ(CmdOdomSync::INDETERMINATE, sync_->update(inf_val, 1.0));
    EXPECT_EQ(CmdOdomSync::INDETERMINATE, sync_->update(1.0, inf_val));
    EXPECT_EQ(CmdOdomSync::INDETERMINATE, sync_->update(-inf_val, inf_val));
}

TEST_F(CmdOdomSyncTest, BoundaryConditions) {
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(0.0, 0.05));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(1.0, 1.05));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(0.0, 0.15));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(1.0, 1.15));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(0.0, 0.1));
}

TEST_F(CmdOdomSyncTest, ZeroValues) {
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(0.0, 0.0));
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(0.0, 0.05));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(0.0, 0.15));
}

TEST_F(CmdOdomSyncTest, LargeValues) {
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(100.0, 100.05));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(100.0, 100.15));
}

TEST_F(CmdOdomSyncTest, NegativeValues) {
    EXPECT_EQ(CmdOdomSync::SYNCED, sync_->update(-1.0, -1.05));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(-1.0, -1.15));
    EXPECT_EQ(CmdOdomSync::UNSYNCED, sync_->update(-1.0, 1.0));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
