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
