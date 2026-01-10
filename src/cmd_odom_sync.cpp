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

#include "motor_health_monitor/cmd_odom_sync.hpp"

#include <cmath>

namespace motor_health_monitor
{

CmdOdomSync::CmdOdomSync(double tolerance) noexcept
    : tolerance_(tolerance)
{
}

CmdOdomSync::State CmdOdomSync::update(double cmd, double odom) noexcept
{
    const double error = std::abs(cmd - odom);

    if (std::isnan(error) || std::isinf(error)) {
        return INDETERMINATE;
    }

    if (error <= tolerance_) {
        return SYNCED;
    }

    return UNSYNCED;
}

}  // namespace motor_health_monitor
