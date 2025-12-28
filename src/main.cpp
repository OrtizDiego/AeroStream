#include <iostream>
#include <thread> // For sleep
#include <chrono> // For time
#include "PID.hpp"

int main() {
    std::cout << "[SYSTEM STARTUP] Flight Controller Initialized..." << std::endl;

    // PID Parameters
    double dt = 0.1; // 100ms loop time
    double max_val = 100.0;
    double min_val = -100.0;
    double Kp = 0.6;
    double Ki = 0.05;
    double Kd = 0.03;

    // Initialize PID Controller
    PID pid(Kp, Ki, Kd, dt, max_val, min_val);

    // Simulation variables
    double target_altitude = 100.0; // We want to fly to 100m
    double actual_altitude = 0.0;   // We start on the ground

    // Simulate 50 time steps
    for (int i = 0; i < 250; i++) {
        // 1. Calculate the motor output needed
        double motor_power = pid.calculate(target_altitude, actual_altitude);

        // 2. SIMULATE the physics (In the real world, motors would spin and lift the drone)
        // This is a simplified physics model: Altitude += Power * dt
        actual_altitude += motor_power * dt; 

        // 3. Log data to console
        std::cout << "Time: " << i * dt << "s | "
                  << "Target: " << target_altitude << "m | "
                  << "Actual: " << actual_altitude << "m | "
                  << "Motor Power: " << motor_power << std::endl;

        // 4. Wait for dt seconds (simulate real-time)
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }

    return 0;
}
