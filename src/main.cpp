#include <iostream>
#include <thread>
#include <chrono>
#include "PID.hpp"
#include "MockSensor.hpp" // Include our new sensor

int main()
{
    std::cout << "[SYSTEM STARTUP] Flight Controller Initialized..." << std::endl;

    double dt = 0.1;
    // Tuned PID values (Lowered Ki to reduce overshoot)
    PID pid(0.6, 0.01, 0.05, dt, 100.0, -100.0);

    // Initialize the Sensor (Polymorphism in action)
    MockSensor altimeter(0.0);
    altimeter.init();

    double target_altitude = 100.0;

    for (int i = 0; i < 250; i++)
    {
        // 1. READ SENSOR (Now with noise!)
        double current_alt = altimeter.readValue();

        // 2. COMPUTE PID
        double motor_power = pid.calculate(target_altitude, current_alt);

        // 3. UPDATE PHYSICS (Tell the mock sensor to move)
        // In real life, the motor moves the drone, which the sensor then detects.
        altimeter.update(motor_power * dt);

        std::cout << "Time: " << i * dt << "s | "
                  << "Target: " << target_altitude << " | "
                  << "Sensor Read: " << current_alt << " | " // Notice the noise
                  << "Output: " << motor_power << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }

    return 0;
}