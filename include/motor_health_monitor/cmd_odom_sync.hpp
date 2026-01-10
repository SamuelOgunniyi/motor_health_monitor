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

namespace motor_health_monitor
{

class CmdOdomSync
{
public:
  enum State
  {
    SYNCED,
    UNSYNCED,
    INDETERMINATE
  };

  explicit CmdOdomSync(double tolerance = 0.05) noexcept;

  CmdOdomSync(const CmdOdomSync&) = default;
  CmdOdomSync& operator=(const CmdOdomSync&) = default;
  CmdOdomSync(CmdOdomSync&&) noexcept = default;
  CmdOdomSync& operator=(CmdOdomSync&&) noexcept = default;
  ~CmdOdomSync() = default;

  State update(double cmd, double odom) noexcept;

private:
  double tolerance_;
};

}  // namespace motor_health_monitor