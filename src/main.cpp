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
    int steps = 200; // Default steps

    // 2. Parse Arguments (Expect 4 args now: Kp, Ki, Kd, Steps)
    if (argc >= 5)
    {
        try
        {
            Kp = std::stod(argv[1]);
            Ki = std::stod(argv[2]);
            Kd = std::stod(argv[3]);
            steps = std::stoi(argv[4]); // Parse integer steps
        }
        catch (...)
        {
            std::cerr << "Invalid arguments. Using defaults." << std::endl;
        }
    }

    std::cout << "[SYSTEM STARTUP] Simulating " << steps << " steps..." << std::endl;

    // 3. Setup
    std::ofstream logFile("telemetry.csv");
    logFile << "Time,Target,Actual,Output\n";

    double dt = 0.1;
    PID pid(Kp, Ki, Kd, dt, 100.0, -100.0);

    MockSensor altimeter(0.0);
    altimeter.init();

    double target_altitude = 100.0;

    // 4. Run Loop using 'steps' variable
    for (int i = 0; i < steps; i++)
    {
        double current_alt = altimeter.readValue();
        double motor_power = pid.calculate(target_altitude, current_alt);
        altimeter.update(motor_power * dt);

        // 2. Write data to CSV
        logFile << i * dt << "," << target_altitude << "," << current_alt << "," << motor_power << "\n";
    }

    logFile.close();
    return 0;
}