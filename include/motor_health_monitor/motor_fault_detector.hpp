#pragma once

#include "motor_controller.hpp"
#include <cmath>

namespace motor_health_monitor
{

enum class MotorFaultState {
    FAULT,
    NO_FAULT
};

struct FaultDetectionConfig  {
    double limit = 0.95;
    double duration = 0.2;
};

class MotorFaultDetector {
public:
    MotorFaultDetector(const FaultDetectionConfig& config, MotorController& controller)
        : config_(config),
          motor_controller_(controller),
          state_(MotorFaultState::NO_FAULT),
          timer_started_(false),
          start_time_(-1.0) {}

    MotorFaultDetector(const MotorFaultDetector&) = delete;
    MotorFaultDetector& operator=(const MotorFaultDetector&) = delete;
    MotorFaultDetector(MotorFaultDetector&&) = delete;
    MotorFaultDetector& operator=(MotorFaultDetector&&) = delete;

    ~MotorFaultDetector() = default;

    void update(double current_time) {
        double value = motor_controller_.getFeedbackValue();

        if (std::fabs(value) < config_.limit) {
            state_ = MotorFaultState::NO_FAULT;
            timer_started_ = false;
            return;
        }

        if (!timer_started_) {
            start_time_ = current_time;
            timer_started_ = true;
        }

        if ((current_time - start_time_) >= config_.duration) {
            state_ = MotorFaultState::FAULT;
        }
    }

    MotorFaultState getState() const noexcept {
        return state_;
    }

    void setConfig(const FaultDetectionConfig& config) noexcept {
        config_ = config;
    }

private:
    FaultDetectionConfig config_;
    MotorController& motor_controller_;
    MotorFaultState state_;
    bool timer_started_;
    double start_time_;
};

} // namespace motor_health_monitor
