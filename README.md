# Motor Health Monitor

A modular C++ tool for:

- **Command/Odometry synchronization** (`CmdOdomSync`)
- **Motor fault detection** with swappable feedback sources (`MotorFaultDetector`)

## Features

- Encoder fault detection using thresholds and durations
- Swappable motor controller implementations via a common interface
- C++17 and CMake-ready
- Suitable for robotics, motor diagnostics, or simulation

## Simulation Scenarios

This project includes simulated behavior of:

- CmdOdomSync states (`SYNCED`, `UNSYNCED`, `INDETERMINATE`) over time
- Encoder-based motor fault detection (e.g., prolonged saturation)

The simulations run in `main.cpp` and provide a basic system check without real hardware.

## Build Instructions

```bash
mkdir build && cd build
cmake ..
make
./motor_health_monitor

## Usage

- Modify simulation parameters in main.cpp as needed
- Integrate MotorFaultDetector and CmdOdomSync into your robotics software
