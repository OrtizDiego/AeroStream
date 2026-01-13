#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <string> // Required for std::stod
#include "PID.hpp"
#include "MockSensor.hpp"

int main(int argc, char *argv[])
{
    // 1. Default Defaults
    double Kp = 0.6;
    double Ki = 0.01;
    double Kd = 0.05;

    // 2. Override if arguments are provided
    if (argc == 4)
    {
        try
        {
            Kp = std::stod(argv[1]);
            Ki = std::stod(argv[2]);
            Kd = std::stod(argv[3]);
        }
        catch (...)
        {
            std::cerr << "Invalid arguments. Using defaults." << std::endl;
        }
    }

    // 3. DEBUG PRINT: Verify we received the values
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "[SYSTEM STARTUP] Configuring Flight Controller..." << std::endl;
    std::cout << "   Kp: " << Kp << " | Ki: " << Ki << " | Kd: " << Kd << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    // 4. Setup Simulation
    std::ofstream logFile("telemetry.csv");
    logFile << "Time,Target,Actual,Output\n";

    double dt = 0.1;

    // CRITICAL: Ensure we pass the variables Kp, Ki, Kd here!
    PID pid(Kp, Ki, Kd, dt, 100.0, -100.0);

    MockSensor altimeter(0.0);
    altimeter.init();

    double target_altitude = 100.0;

    // 5. Run Loop
    for (int i = 0; i < 2000; i++)
    {
        double current_alt = altimeter.readValue();
        double motor_power = pid.calculate(target_altitude, current_alt);
        altimeter.update(motor_power * dt);

        // 2. Write data to CSV
        logFile << i * dt << "," << target_altitude << "," << current_alt << "," << motor_power << "\n";

        // Removed sleep for performance when called from Python
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    logFile.close();
    return 0;
}