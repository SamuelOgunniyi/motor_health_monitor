#!/bin/bash

# Simple test script for motor_health_monitor node
# Uses ros2 topic commands for quick manual testing without Python dependencies
# Useful for verifying basic functionality after code changes

set +e  # Don't exit on error, we'll handle cleanup

echo "=========================================="
echo "Motor Health Monitor Simple Test Script"
echo "=========================================="
echo ""
echo "This script will:"
echo "  1. Launch the motor_health_monitor node"
echo "  2. Publish test cmd_vel messages"
echo "  3. Publish test odom messages"
echo "  4. Echo diagnostic messages"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not sourced. Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Function to cleanup on exit
cleanup() {
    if [ "$CLEANUP_DONE" = "1" ]; then
        return
    fi
    CLEANUP_DONE=1
    echo ""
    echo "Cleaning up..."
    kill $LAUNCH_PID 2>/dev/null || true
    kill $DIAGNOSTICS_PID 2>/dev/null || true
    pkill -f "motor_health" 2>/dev/null || true
    pkill -f "component_container" 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT
CLEANUP_DONE=0

# Launch the node using launch file
echo "Launching motor_health_monitor node..."
echo "Checking if package is available..."
if ! ros2 pkg list | grep -q motor_health_monitor; then
    echo "ERROR: Package not found. Make sure you've sourced install/setup.bash"
    echo "Running: source install/setup.bash"
    exit 1
fi

ros2 launch motor_health_monitor motor_health.launch.py > /tmp/motor_health_launch.log 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"
sleep 4

# Start diagnostics echo in background
echo "Starting diagnostics monitor..."
ros2 topic echo /diagnostics &
DIAGNOSTICS_PID=$!

# Give it a moment to start
sleep 1

echo ""
echo "Publishing test messages..."
echo ""

# Scenario 1: Normal operation
echo "=== Scenario 1: Normal operation (matching velocities) ==="
for i in {1..10}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ros2 topic pub --once /odom nav_msgs/msg/Odometry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, child_frame_id: 'base_link', twist: {twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
    sleep 0.2
done

sleep 2

# Scenario 2: Sync issue
echo ""
echo "=== Scenario 2: Sync issue (odometry lagging) ==="
for i in {1..10}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ros2 topic pub --once /odom nav_msgs/msg/Odometry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, child_frame_id: 'base_link', twist: {twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    sleep 0.2
done

sleep 2

# Scenario 3: Fault condition
echo ""
echo "=== Scenario 3: Fault condition (motor not responding) ==="
for i in {1..15}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ros2 topic pub --once /odom nav_msgs/msg/Odometry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, child_frame_id: 'base_link', twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    sleep 0.2
done

sleep 2

# Scenario 4: Recovery
echo ""
echo "=== Scenario 4: Recovery (back to normal) ==="
for i in {1..10}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ros2 topic pub --once /odom nav_msgs/msg/Odometry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, child_frame_id: 'base_link', twist: {twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    sleep 0.2
done

echo ""
echo "Test complete!"
sleep 2

cleanup
