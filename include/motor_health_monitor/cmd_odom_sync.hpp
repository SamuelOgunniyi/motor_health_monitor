#pragma once

#include <cmath>

/// CmdOdomSync detects synchronization state between a setpoint and odometry stream.
/// Can be used in real-time or simulation environments.

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

