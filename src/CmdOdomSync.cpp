#include "motor_health_monitor/CmdOdomSync.h"
#include <iostream>

CmdOdomSync::CmdOdomSync(const SyncConfig& config)
    : config_(config),
      state_(OdomSyncState::SYNCED),
      spike_start_time_(-1.0) {}

void CmdOdomSync::update(double setpoint, double setpoint_time,
                        double odometry, double odometry_time,
                        double current_time) {
    prev_setpoint_ = latest_setpoint_;
    latest_setpoint_ = {setpoint, setpoint_time};
    latest_odom_ = {odometry, odometry_time};
    bool dontComputeError = false;

    if (config_.use_averaging) {
        setpoint_history_.push_back(latest_setpoint_);
        odom_history_.push_back(latest_odom_);
        trimHistory(setpoint_history_);
        trimHistory(odom_history_);
    }

    if (isSpikeDetected()) {
        spike_start_time_ = current_time;
        state_ = OdomSyncState::INDETERMINATE;
        return;
    }

    if (!hasSettled(current_time)) {
        state_ = OdomSyncState::INDETERMINATE;
        return;
    }

    spike_start_time_ = -1.0; // reset spike start time when settled
       
    double error = computeError();
    
    if (std::isnan(error)) {
        state_ = OdomSyncState::INDETERMINATE;
        return;
    }

    state_ = std::abs(error) <= config_.error_threshold
               ? OdomSyncState::SYNCED
               : OdomSyncState::UNSYNCED;
}

OdomSyncState CmdOdomSync::getState() const {
    return state_;
}

bool CmdOdomSync::isSpikeDetected() const {
    return std::abs(latest_setpoint_.value - prev_setpoint_.value) >
           config_.spike_threshold;
}

bool CmdOdomSync::hasSettled(double current_time) const {
    return spike_start_time_ < 0.0 ||
           (current_time - spike_start_time_) > config_.response_timeout;
}

void CmdOdomSync::trimHistory(std::deque<MotionSignal >& history) {
    while (history.size() > config_.sample_window) {
        history.pop_front();
    }
}

double CmdOdomSync::computeError() const {
    if (config_.use_averaging) {
        return average(odom_history_) - average(setpoint_history_);
    } else {
        return latest_odom_.value - latest_setpoint_.value;
    }
}

double CmdOdomSync::average(const std::deque<MotionSignal >& data) {
    if (data.empty()) return std::numeric_limits<double>::quiet_NaN();
    double sum = 0.0;
    for (const auto& s : data) sum += s.value;
    return sum / data.size();
}

