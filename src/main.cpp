#include <iostream>
#include <fstream> // New include
#include <thread>
#include <chrono>
#include "PID.hpp"
#include "MockSensor.hpp"

int main()
{
    // 1. Open a file for writing
    std::ofstream logFile("telemetry.csv");
    logFile << "Time,Target,Actual,Output\n"; // CSV Header

    double dt = 0.1;
    PID pid(0.6, 0.01, 0.05, dt, 100.0, -100.0);
    MockSensor altimeter(0.0);
    altimeter.init();

    double target_altitude = 100.0;

    std::cout << "Starting Flight... Logging to telemetry.csv" << std::endl;

    for (int i = 0; i < 200; i++)
    { // Increased to 200 steps
        double current_alt = altimeter.readValue();
        double motor_power = pid.calculate(target_altitude, current_alt);
        altimeter.update(motor_power * dt);

        // 2. Write data to CSV
        logFile << i * dt << "," << target_altitude << "," << current_alt << "," << motor_power << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Run faster for the demo
    }

    logFile.close();
    std::cout << "Flight Complete." << std::endl;
    return 0;
}