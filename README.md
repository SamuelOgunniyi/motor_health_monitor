# Motor Health Monitor

> A modular, C++17-based toolkit for motor diagnostics and command/odometry compliance monitoring. Hardware-agnostic and simulation-ready.

![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)

---

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Simulation Scenarios](#simulation-scenarios)
- [Getting Started](#getting-started)
- [Build Instructions](#build-instructions)
- [Run the Simulation](#run-the-simulation)
- [Project Structure](#project-structure)
- [License](#license)
- [Contributing](#contributing)
- [Contact](#contact)

---

## Features

- ✅ **Command/Odometry Synchronization** with `CmdOdomSync`
- ⚙️ **Motor Fault Detection** using `MotorFaultDetector`
- 🔌 Easily extendable to hardware drivers or simulated environments
- 🧩 ROS-independent, pure C++17
- 🧪 Ready for CI testing and modular integration

---

## Architecture

![System Architecture](docs/architecture.png)

---

## Simulation Scenarios

This project includes built-in test simulations that verify:

* **CmdOdomSync** state transitions:
  `SYNCED`, `UNSYNCED`, and `INDETERMINATE`
* **Motor fault detection** logic with duration tracking

These are implemented in `main.cpp`, and require **no hardware** to run.

---

## Getting Started

### 🔧 Requirements

* C++17
* CMake ≥ 3.10
* Linux / WSL (tested)

---

## Build Instructions

```bash
git clone https://github.com/SamuelOgunniyi/motor_health_monitor.git
cd motor_health_monitor
mkdir build && cd build
cmake ..
make
```

---

## Run the Simulation

```bash
./motor_health_monitor
```

This will:

* Generate synthetic command and odometry signals
* Simulate a mismatch (non-compliance)
* Trigger fault conditions and state transitions

The simulation runs entirely from `main.cpp`.

---

## Project Structure

```
motor_health_monitor/
├── include/
│   └── motor_health_monitor/
│       ├── CmdOdomSync.h
│       ├── MotorController.h
│       ├── MotorFaultDetector.h
│       ├── RobotDriveMotorController.h
│       └── motor_controller_factory.hpp
├── src/
│   ├── CmdOdomSync.cpp
│   └── main.cpp
├── build/
├── CMakeLists.txt
├── LICENSE
└── README.md
```

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Contributing

Contributions are welcome! Feel free to:

* Open issues for bug reports or feature ideas
* Submit pull requests with enhancements or test cases
* Fork and adapt the project for your own applications

---

## Contact

Have questions or ideas? Open an issue or reach out via GitHub.
