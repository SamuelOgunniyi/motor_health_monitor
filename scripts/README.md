# Test Scripts

This directory contains scripts to test the motor_health_monitor node.

## test_motor_health.py

A Python script that provides comprehensive testing with multiple scenarios.

### Usage

```bash
# Make sure ROS 2 is sourced and the package is built
source install/setup.bash

# Run the test script
python3 scripts/test_motor_health.py
```

Or if installed:

```bash
ros2 run motor_health_monitor test_motor_health.py
```

### Features

- Publishes test `cmd_vel` and `odom` messages
- Subscribes to `/diagnostics` and displays messages
- Runs through 4 test scenarios:
  1. **Normal operation** - Matching velocities
  2. **Sync issue** - Odometry lagging behind command
  3. **Fault condition** - Motor not responding (sustained error)
  4. **Recovery** - Back to normal operation

### Output

The script displays diagnostic messages with:
- Status level (OK, WARN, ERROR)
- Status message
- Timestamps
- Hardware ID

## test_motor_health_simple.sh

A bash script using ROS 2 command-line tools for quick testing.

### Usage

```bash
# Make sure ROS 2 is sourced
source install/setup.bash

# Run the script
./scripts/test_motor_health_simple.sh
```

### Features

- Uses `ros2 topic pub` to publish messages
- Uses `ros2 topic echo` to display diagnostics
- Simpler but less flexible than the Python script

## Manual Testing

You can also test manually using ROS 2 commands:

### Terminal 1: Launch the node
```bash
ros2 launch motor_health_monitor motor_health.launch.py
```

### Terminal 2: Echo diagnostics
```bash
ros2 topic echo /diagnostics
```

### Terminal 3: Publish test messages
```bash
# Publish cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Publish odom
ros2 topic pub /odom nav_msgs/msg/Odometry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, child_frame_id: 'base_link', twist: {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

## Expected Behavior

- **Normal operation**: Diagnostic status should show `OK` with message "Motor healthy"
- **Sync issue**: Status may show `WARN` or `ERROR` depending on tolerance settings
- **Fault condition**: After the fault duration threshold, status should show `ERROR` with message "Motor fault detected"
- **Recovery**: Status should return to `OK` when velocities match again
