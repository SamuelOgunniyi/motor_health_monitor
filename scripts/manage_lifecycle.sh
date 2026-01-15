#!/bin/bash

# Helper script to manage motor_health_monitor lifecycle node
# Created to avoid typing long ros2 lifecycle commands during development and testing

NODE_NAME="motor_health_monitor"

case "$1" in
    configure)
        echo "Configuring $NODE_NAME..."
        ros2 lifecycle set /motor_health_monitor configure
        ;;
    activate)
        echo "Activating $NODE_NAME..."
        ros2 lifecycle set /motor_health_monitor activate
        ;;
    deactivate)
        echo "Deactivating $NODE_NAME..."
        ros2 lifecycle set /motor_health_monitor deactivate
        ;;
    cleanup)
        echo "Cleaning up $NODE_NAME..."
        ros2 lifecycle set /motor_health_monitor cleanup
        ;;
    shutdown)
        echo "Shutting down $NODE_NAME..."
        ros2 lifecycle set /motor_health_monitor shutdown
        ;;
    state)
        echo "Current state of $NODE_NAME:"
        ros2 lifecycle get /motor_health_monitor
        ;;
    start)
        echo "Starting lifecycle node (configure + activate)..."
        ros2 lifecycle set /motor_health_monitor configure
        sleep 1
        ros2 lifecycle set /motor_health_monitor activate
        ;;
    *)
        echo "Usage: $0 {configure|activate|deactivate|cleanup|shutdown|state|start}"
        echo ""
        echo "Lifecycle states:"
        echo "  unconfigured -> configuring -> inactive -> activating -> active"
        echo "  active -> deactivating -> inactive -> cleaningup -> unconfigured"
        echo ""
        echo "Commands:"
        echo "  configure   - Configure the node"
        echo "  activate    - Activate the node (starts processing)"
        echo "  deactivate  - Deactivate the node (stops processing)"
        echo "  cleanup     - Cleanup resources"
        echo "  shutdown    - Shutdown the node"
        echo "  state       - Show current state"
        echo "  start       - Configure and activate (quick start)"
        exit 1
        ;;
esac
