#ifndef CMD_ODOM_SYNC_H
#define CMD_ODOM_SYNC_H

/// CmdOdomSync detects synchronization state between a setpoint and odometry stream.
/// Can be used in real-time or simulation environments.

#include <deque>
#include <cmath>
#include <limits>

enum class OdomSyncState {
    SYNCED,
    UNSYNCED,
    INDETERMINATE
};

struct MotionSignal  {
    double value = 0.0;
    double timestamp = 0.0;  // Unix time (seconds)
};

using SetpointReading = MotionSignal;
using OdometryReading = MotionSignal;

struct SyncConfig {
    bool use_averaging = true;
    double error_threshold = 0.1;
    double spike_threshold = 0.8;
    double response_timeout = 0.5;  // seconds
    size_t sample_window = 5;
};

class CmdOdomSync {
public:
    explicit CmdOdomSync(const SyncConfig& config);

    void update(double setpoint, double setpoint_time,
                double odometry, double odometry_time,
                double current_time);

    OdomSyncState getState() const;

private:
    SyncConfig config_;
    OdomSyncState state_;

    SetpointReading  latest_setpoint_, prev_setpoint_;
    OdometryReading  latest_odom_;
    std::deque<MotionSignal > setpoint_history_, odom_history_;

    double spike_start_time_;

    bool isSpikeDetected() const;
    bool hasSettled(double current_time) const;
    void trimHistory(std::deque<MotionSignal >& history);
    double computeError() const;
    static double average(const std::deque<MotionSignal >& data);
};

#endif  // CMD_ODOM_SYNC_H
