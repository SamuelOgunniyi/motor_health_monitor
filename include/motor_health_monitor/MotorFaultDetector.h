// MotorFaultDetector.h
#ifndef MOTOR_FAULT_DETECTOR_H
#define MOTOR_FAULT_DETECTOR_H

#include "MotorController.h"
#include <cmath>

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

    MotorFaultState getState() const {
        return state_;
    }

private:
    FaultDetectionConfig config_;
    MotorController& motor_controller_;
    MotorFaultState state_;
    bool timer_started_;
    double start_time_;
};

#endif // MOTOR_FAULT_DETECTOR_H
