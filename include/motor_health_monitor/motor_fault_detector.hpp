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

#pragma once

#include <cmath>

#include "motor_controller.hpp"

namespace motor_health_monitor
{

enum class MotorFaultState {
    FAULT,
    NO_FAULT
};

enum class ThresholdType {
    ABSOLUTE_VALUE,
    UPPER_LIMIT,
    LOWER_LIMIT
};

struct FaultDetectionConfig {
    double threshold = 0.95;
    double duration = 0.2;
    ThresholdType type = ThresholdType::ABSOLUTE_VALUE;
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
        double feedback_value = motor_controller_.getFeedbackValue();
        bool threshold_exceeded = false;

        switch (config_.type) {
            case ThresholdType::ABSOLUTE_VALUE:
                threshold_exceeded = std::fabs(feedback_value) >= config_.threshold;
                break;
            case ThresholdType::UPPER_LIMIT:
                threshold_exceeded = feedback_value >= config_.threshold;
                break;
            case ThresholdType::LOWER_LIMIT:
                threshold_exceeded = feedback_value <= config_.threshold;
                break;
        }

        if (!threshold_exceeded) {
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

}  // namespace motor_health_monitor
