#include <iostream>
#include <fstream>
#include <string> // Required for std::stod
#include "PID.hpp"
#include "MockSensor.hpp"

int main(int argc, char *argv[])
{
    // 1. Defaults
    double Kp = 0.6;
    double Ki = 0.01;
    double Kd = 0.05;
    int steps = 200;
    double target_altitude = 100.0; // Default Target

    // 2. Parse Arguments (Now expecting up to 5 args)
    if (argc >= 6)
    {
        try
        {
            Kp = std::stod(argv[1]);
            Ki = std::stod(argv[2]);
            Kd = std::stod(argv[3]);
            steps = std::stoi(argv[4]);
            target_altitude = std::stod(argv[5]); // <--- New Argument
        }
        catch (...)
        {
            std::cerr << "Invalid arguments. Using defaults." << std::endl;
        }
    }

    std::cout << "[SYSTEM STARTUP] Simulating " << steps << " steps to " << target_altitude << "m..." << std::endl;

    // 3. Setup
    std::ofstream logFile("telemetry.csv");
    logFile << "Time,Target,Actual,Output\n";

    double dt = 0.1;
    PID pid(Kp, Ki, Kd, dt, 500.0, -500.0); // Increased motor limits for higher flights

    MockSensor altimeter(0.0);
    altimeter.init();

    // 4. Run Loop
    for (int i = 0; i < steps; i++)
    {
        double current_alt = altimeter.readValue();
        double motor_power = pid.calculate(target_altitude, current_alt);
        altimeter.update(motor_power * dt);

        // 5. Write data to CSV
        logFile << i * dt << "," << target_altitude << "," << current_alt << "," << motor_power << "\n";
    }

    logFile.close();
    return 0;
}