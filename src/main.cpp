// This program simulates the behavior of CmdOdomSync and MotorFaultDetector
// using predefined inputs to trigger SYNCED, UNSYNCED, and INDETERMINATE states,
// as well as motor fault detection based on PWM values.
//
// These are not mocks, but runtime simulations of system behavior.




#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <random>

#include "motor_health_monitor/RobotDriveMotorController.h"
#include "motor_health_monitor/MotorFaultDetector.h"
#include "motor_health_monitor/CmdOdomSync.h"

int main() {
    FaultDetectionConfig config{0.9, 0.3};

    RobotDriveMotorController motor_controller;  // default ctor
    MotorFaultDetector detector(config, motor_controller);

    SyncConfig sync_config;
    CmdOdomSync odom_sync(sync_config);

    double current_time = 0.0;
    double timestep = 0.1;

    std::cout << "=== Motor Fault Simulation ===" << std::endl;
    for (int i = 0; i < 20; ++i) {
        double simulated_pwm = (i < 5) ? 0.5 : 0.95;  // Causes FAULT after a while
        motor_controller.setCommand(simulated_pwm);     
        detector.update(current_time);

        std::cout << "Time: " << current_time
                  << " | PWM: " << motor_controller.getFeedbackValue()
                  << " | Fault: "
                  << (detector.getState() == MotorFaultState::FAULT ? "FAULT" : "NO_FAULT")
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_time += timestep;
    }

    std::cout << "\n=== Simulating CmdOdomSync Behavior (SYNCED / UNSYNCED / INDETERMINATE) ===" << std::endl;

    // Simulate CmdOdomSync behavior under different conditions:
    // - SYNCED: matching setpoint and odometry
    // - UNSYNCED: large tracking error
    // - INDETERMINATE: sudden setpoint spikes

    // current_time = 0.0;

    double spike_peak = 3.0;

    double setpoint = 1.0;
    double odometry = 1.0;

    for (int i = 0; i < 30; ++i) {


        if (i < 10) {
            // Perfectly synced phase
            setpoint = 1.0;
            odometry = 1.0;
        } 
        else if (i >= 10 && i < 11) {
            //setpoint = 3.0;

            setpoint = spike_peak;

            // Smooth nonlinear catch-up: use a quadratic easing function
            double progress = ((i - 10) / 4.0) * 0.2; // progresses from 0.0 → 0.4
            odometry = 1.0 + (spike_peak - 1.0) * (progress * progress);  // easing in
        }
        else if (i >= 12 && i < 20) {
            // Back to synced steady motion
            setpoint = 1.0;
            odometry = 1.0;
        } 
        else if (i >= 20 && i < 25) {
            // Add increasing noise to odometry → gradual desync
            setpoint = 1.0;
            odometry = 1.0 + ((rand() % 10) / 1000.0 - 0.005); // ±0.005 
        } 
        else {
            // Final phase: consistently offset odometry (sensor drift)
            setpoint = 1.5;
            odometry = 1.22;  // 0.2 offset — looks like sensor drift or bias
        }

        odom_sync.update(setpoint, current_time, odometry, current_time, current_time);

        OdomSyncState state = odom_sync.getState();
        std::string state_str = 
            (state == OdomSyncState::SYNCED) ? "SYNCED" :
            (state == OdomSyncState::UNSYNCED) ? "UNSYNCED" : "INDETERMINATE";

        std::cout << "" << current_time
                    << " | Setpoint: " << setpoint
                    << " | Odometry: " << odometry
                    << " | SyncState: " << state_str
                    << std::endl;


        //std::cout << current_time << "," << setpoint << "," << odometry << "," << state_str << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_time += timestep;
    }
    
    return 0;
}