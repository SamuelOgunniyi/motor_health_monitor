# Motor Health Monitor

A ROS 2 C++17 package for monitoring motor health and detecting faults through configurable motor controller feedback analysis and command/odometry synchronization detection.

**Author:** Samuel Ogunniyi  
**Maintainer:** Samuel Ogunniyi (samogunniyi@gmail.com)

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![C++](https://img.shields.io/badge/C++-17-blue)

## Features

- ✅ **Command/Odometry Synchronization Detection** - Monitors sync state between commanded and actual velocities
- 🔍 **Generic Motor Fault Detection** - Configurable threshold-based fault detection for any motor feedback type (PWM, current, temperature, etc.)
- 🔌 **Pluggable Motor Controllers** - Factory pattern allows custom motor controller implementations via configuration
- 🔄 **Auto-Subscription with Topic Introspection** - Controllers automatically detect and subscribe to their message types
- ⏱️ **Stale Data Detection** - Detects and handles stale messages with configurable timeout
- 🚫 **Bounded Queue with Drop-Oldest Policy** - Queue depth of 1 ensures old messages are dropped immediately, prioritizing fresh data for deterministic control
- 🧩 **ROS 2 Component Architecture** - Built as a composable lifecycle node using `rclcpp_components`
- 🧪 **Comprehensive Unit Tests** - Full test coverage with Google Test
- 🔒 **Memory Safe** - Follows modern C++17 best practices with proper ownership semantics (single-threaded executor)
- 📊 **Enhanced Diagnostics** - Rich diagnostic output with threshold values, feedback values, and message ages

## Architecture

![System Architecture](docs/architecture.png)

The package consists of several key components:

- **MotorHealthNode** - Main ROS 2 lifecycle node that orchestrates monitoring systems
- **CmdOdomSync** - Detects synchronization state between command and odometry streams
- **MotorFaultDetector** - Generic threshold-based fault detector with debouncing (works with any motor feedback type)
- **MotorController** - Abstract interface for motor controller implementations
- **MotorControllerFactory** - Factory pattern for pluggable motor controller types
- **RobotDriveMotorController** - Example implementation with auto-subscription and topic introspection

## How It Works

### Overview

The Motor Health Monitor continuously compares **commanded velocities** (what you want the robot to do) with **actual velocities** (what the robot is actually doing based on odometry) to detect motor faults and synchronization issues.

### Data Flow

```
/cmd_vel (Twist) ──────────────┐
                               ├──> MotorHealthNode ──> /diagnostics (DiagnosticArray)
/odom (Odometry) ──────────────┤
                               │
/motor_pwm (Float64) ──────────┘
  (or custom topic)
```

1. **Data Collection**: The node subscribes to:
   - `/cmd_vel` - Commanded velocity (what the control system wants)
   - `/odom` - Actual velocity from odometry (what the robot is actually doing)
   - Motor feedback topic (configurable, e.g., `/motor_pwm`) - Hardware-level motor feedback

2. **Processing Loop** (runs at configurable rate, default 100ms):
   - Stores latest messages (queue depth = 1, drops old messages immediately)
   - Checks for stale data (messages older than timeout)
   - **Process 1**: Command/Odometry synchronization check
   - **Process 2**: Motor fault detection from controller feedback
   - Publishes enhanced diagnostic status with detailed information

### Two Monitoring Systems

#### 1. Command/Odometry Synchronization (`CmdOdomSync`)

Detects if commanded and actual velocities are synchronized:

- **SYNCED**: Velocities match within tolerance (default: ±0.05 m/s)
- **UNSYNCED**: Velocities differ beyond tolerance
- **INDETERMINATE**: Invalid data (NaN or Inf values)

**Use case**: Detects control system issues, communication delays, or sensor problems.

#### 2. Motor Fault Detection (`MotorFaultDetector`)

Generic threshold-based fault detection with debouncing. Works with **any motor controller feedback type**:

- Reads feedback value from `MotorController` (PWM, current, temperature, etc.)
- Checks if threshold is exceeded based on threshold type:
  - **Absolute Value**: `|value| >= threshold` (for PWM, bidirectional signals)
  - **Upper Limit**: `value >= threshold` (for current, temperature max limits)
  - **Lower Limit**: `value <= threshold` (for voltage minimums)
- If threshold exceeded for duration (default: 0.2s) → **FAULT**
- Timer resets if value returns to normal
- Prevents false positives from brief spikes

**Use cases**:
- PWM saturation detection (motor at max output)
- Current overload detection
- Temperature monitoring
- Voltage monitoring
- Any numeric motor feedback signal

### Lifecycle States

As a lifecycle node, the MotorHealthNode follows ROS 2 lifecycle state machine:

- **unconfigured** → `configure` → **inactive** → `activate` → **active**
- **active** → `deactivate` → **inactive** → `cleanup` → **unconfigured**

**Behavior by state:**
- **unconfigured**: Node created but not initialized
- **inactive**: Configured, subscriptions active, but not processing
- **active**: Fully operational, processing data and publishing diagnostics
- Processing only occurs when in **active** state

### Diagnostic Output

The node publishes to `/diagnostics` with:

- **Status Level**:
  - `OK` - Motor healthy (no fault detected)
  - `WARN` - Stale data detected (messages older than timeout)
  - `ERROR` - Motor fault detected (threshold exceeded for duration)
- **Status Message**: Human-readable description with fault reasoning
- **Hardware ID**: Identifies which motor is being monitored
- **Timestamp**: When the status was computed
- **Diagnostic Values**:
  - `feedback_value` - Current motor feedback value
  - `fault_threshold` - Configured threshold value
  - `fault_duration` - Duration required for fault detection
  - `threshold_type` - Type of threshold comparison
  - `cmd_vel_age` - Age of last cmd_vel message (seconds)
  - `odom_age` - Age of last odom message (seconds)

### Example Scenarios

**Normal Operation:**
- Command: 1.0 m/s, Odometry: 1.0 m/s → Difference: 0.0 → Status: `OK`

**Sync Issue:**
- Command: 1.0 m/s, Odometry: 0.3 m/s → Difference: 0.7 → Status: `OK` (if brief) or `ERROR` (if sustained)

**Motor Fault:**
- Command: 1.0 m/s, Odometry: 0.0 m/s → Difference: 1.0 → After 0.2s → Status: `ERROR`

## Requirements

- ROS 2 (tested with Jazzy)
- C++17 compiler
- CMake ≥ 3.8
- ament_cmake build system

## Build Instructions

### Standard Build

```bash
cd /path/to/your/ros2_ws/src
git clone https://github.com/SamuelOgunniyi/motor_health_monitor.git
cd /path/to/your/ros2_ws
colcon build --packages-select motor_health_monitor
source install/setup.bash
```

### Build with Tests

```bash
colcon build --cmake-args -DBUILD_TESTING=ON --packages-select motor_health_monitor
```

## Running

### Launch the Node

```bash
ros2 launch motor_health_monitor motor_health.launch.py
```

**Note:** This is a lifecycle node. After launching, you need to configure and activate it:

```bash
# Option 1: Use the helper script
./scripts/manage_lifecycle.sh start

# Option 2: Manual lifecycle commands
ros2 lifecycle set /motor_health_monitor configure
ros2 lifecycle set /motor_health_monitor activate

# Check state
ros2 lifecycle get /motor_health_monitor
```

### Lifecycle Management

The node follows the ROS 2 lifecycle state machine:

- **unconfigured** → `configure` → **inactive** → `activate` → **active**
- **active** → `deactivate` → **inactive** → `cleanup` → **unconfigured**

Use the helper script for easy management:
```bash
./scripts/manage_lifecycle.sh {configure|activate|deactivate|cleanup|shutdown|state|start}
```

### View Diagnostics

```bash
ros2 topic echo /diagnostics
```

## Configuration

The node supports the following ROS 2 parameters:

- `sync_tolerance` (double, default: 0.05) - Tolerance for command/odometry synchronization
- `update_rate_ms` (int, default: 100) - Update rate in milliseconds
- `fault_threshold` (double, default: 0.95) - Threshold value for fault detection (units match feedback_value, typically normalized PWM, current, or other motor feedback signal)
- `fault_duration` (double, default: 0.2) - Duration in seconds before fault is triggered
- `fault_threshold_type` (string, default: "absolute_value") - Threshold comparison type: `"absolute_value"` (|feedback| >= threshold), `"upper_limit"` (feedback >= threshold), or `"lower_limit"` (feedback <= threshold)
- `stale_data_timeout` (double, default: 1.0) - Timeout in seconds for detecting stale messages
- `pwm_topic` (string, default: "motor_pwm") - Topic name for motor feedback
- `motor_controller_type` (string, default: "robot_drive") - Type of motor controller to use
- `motor_message_type` (string, default: "") - Message type override (auto-detected from topic if empty)
- `motor_pwm_field` (string, default: "") - Field path to extract value from message (uses controller default if empty)

**Note:** Parameter descriptions are available via `ros2 param describe /motor_health_monitor <parameter_name>`

Example configuration file (`config/motor_health.yaml`):

```yaml
motor_health_monitor:
  ros__parameters:
    sync_tolerance: 0.05
    update_rate_ms: 100
    fault_threshold: 0.95  # Threshold for fault detection (units match feedback_value)
    fault_duration: 0.2
    fault_threshold_type: "absolute_value"  # Options: "absolute_value", "upper_limit", "lower_limit"
    stale_data_timeout: 1.0  # Timeout for stale data detection (seconds)
    pwm_topic: "motor_pwm"
    motor_controller_type: "robot_drive"
    motor_message_type: ""  # Auto-detected if empty
    motor_pwm_field: ""      # Uses controller default if empty
```

### Built-in Motor Controllers

- **`robot_drive`** - PWM-based controller with auto-subscription
  - Handles `std_msgs/msg/Float64` messages
  - Auto-detects message type from topic
  - Extracts PWM value from `data` field

### Threshold Types

- **`absolute_value`**: Detects when `|feedback_value| >= threshold`
  - Use for: PWM saturation, bidirectional signals
  - Example: `fault_threshold: 0.95` detects PWM magnitude ≥ 95%

- **`upper_limit`**: Detects when `feedback_value >= threshold`
  - Use for: Current overload, temperature limits
  - Example: `fault_threshold: 10.0` detects current ≥ 10A

- **`lower_limit`**: Detects when `feedback_value <= threshold`
  - Use for: Voltage minimums, low battery
  - Example: `fault_threshold: 10.5` detects voltage ≤ 10.5V

### Custom Motor Controllers

Users can create their own `MotorController` implementations with auto-subscription support:

1. **Create your controller class** (inherit from `MotorController`):

```cpp
#include "motor_health_monitor/motor_controller.hpp"
#include "motor_health_monitor/motor_controller_factory.hpp"
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialization.hpp>

class MyCustomController : public motor_health_monitor::MotorController {
public:
    bool configure(
        rclcpp_lifecycle::LifecycleNode* node,
        const std::string& topic_name,
        const std::string& message_type_override = "",
        const std::string& field_path_override = "") override {
        // Auto-detect message type, create subscription, extract feedback value
        // See robot_drive_motor_controller.hpp for example
    }
    
    void cleanup() override {
        // Cleanup subscription
    }
    
    double getFeedbackValue() const override {
        // Return value for fault detection
    }
    
    rclcpp::SubscriptionBase::SharedPtr getSubscription() const override {
        return subscription_;
    }
};

// Register it
REGISTER_MOTOR_CONTROLLER("my_custom", MyCustomController)
```

2. **Include your header** in your code (so registration happens)

3. **Configure via YAML**:
```yaml
motor_controller_type: "my_custom"
motor_message_type: "sensor_msgs/msg/JointState"  # Optional override
motor_pwm_field: "effort[0]"  # Optional field path
```

**Key Features:**
- Auto-subscription: Controllers manage their own subscriptions
- Topic introspection: Auto-detect message types from topics
- Generic message support: Works with any ROS 2 message type
- Field extraction: Extract values from complex message structures

See `docs/custom_motor_controller_example.hpp` for a complete example.

## Testing

The package includes comprehensive unit tests:

```bash
# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON --packages-select motor_health_monitor

# Run tests
colcon test --packages-select motor_health_monitor

# View test results
colcon test-result --verbose
```

### Test Coverage

- **CmdOdomSync** (10 tests) - Synchronization state detection, boundary conditions, NaN/Inf handling
- **MotorFaultDetector** (18 tests) - Fault detection logic, timer behavior, configuration updates, threshold types (absolute_value, upper_limit, lower_limit)
- **MotorControllerFactory** (7 tests) - Factory pattern, registration, creation, error handling

Total: **35 tests** covering normal operation, edge cases, and error conditions.

## Project Structure

```
motor_health_monitor/
├── include/
│   └── motor_health_monitor/
│       ├── cmd_odom_sync.hpp              # Command/odometry sync detection
│       ├── motor_controller.hpp            # Motor controller interface
│       ├── motor_controller_factory.hpp    # Factory for pluggable controllers
│       ├── motor_fault_detector.hpp        # Generic fault detection logic
│       ├── motor_health_node.hpp           # Main ROS 2 lifecycle node
│       └── robot_drive_motor_controller.hpp # Example controller with auto-subscription
├── src/
│   ├── cmd_odom_sync.cpp
│   └── motor_health_node.cpp
├── test/
│   ├── mock_motor_controller.hpp          # Mock for testing
│   ├── test_cmd_odom_sync.cpp
│   └── test_motor_fault_detector.cpp
├── docs/
│   └── custom_motor_controller_example.hpp # Example custom controller
├── launch/
│   └── motor_health.launch.py
├── config/
│   └── motor_health.yaml
├── scripts/
│   ├── manage_lifecycle.sh                # Lifecycle management helper
│   └── test_motor_health.py               # Integration test script
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Topics

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Commanded velocity
- `/odom` (`nav_msgs/msg/Odometry`) - Odometry feedback
- `/motor_pwm` (configurable, default: `std_msgs/msg/Float64`) - Motor feedback (auto-detected message type)

### Published Topics

- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) - Motor health diagnostics

## Code Quality

The codebase follows modern C++17 best practices and robotics control system principles:

- **Rule of 5/0** - Explicit copy/move semantics
- **Single-Threaded Executor** - Uses ROS 2 single-threaded executor (no mutexes needed, deterministic execution)
- **Bounded Queue with Drop-Oldest Policy** - Queue depth of 1 ensures old messages are dropped immediately, prioritizing fresh data for deterministic control
- **Time Is a Contract** - Uses monotonic time for fault detection duration calculations
- **Exception Safety** - RAII patterns throughout
- **Const Correctness** - Proper use of `const` and `noexcept`
- **Memory Safety** - Smart pointers, no raw pointers, clear ownership

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## Author & Maintainer

**Samuel Ogunniyi**  
Email: samogunniyi@gmail.com

## Acknowledgments

This package implements a flexible, extensible motor health monitoring system with:
- Factory pattern for pluggable motor controllers
- Generic fault detection supporting multiple threshold types
- Auto-subscription with topic introspection
- ROS 2 lifecycle node architecture

Designed for autonomous mobile robots and industrial applications requiring robust motor health monitoring.

---

<sub>This project uses AI-assisted development tools. See [DEVELOPMENT.md](docs/DEVELOPMENT.md) for details.</sub>