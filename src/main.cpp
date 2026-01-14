#include <iostream>
#include <fstream>
#include <string>
#include "PID.hpp"
#include "MockSensor.hpp"

int main(int argc, char *argv[])
{
    // 1. Defaults
    double Kp = 0.6, Ki = 0.01, Kd = 0.05;
    int steps = 1000;
    double target1 = 50.0;  // Start here
    double target2 = 100.0; // Move here later
    int switch_step = 500;  // When to switch

    // 2. Parse Arguments (Now expecting up to 7 args)
    if (argc >= 8)
    {
        try
        {
            Kp = std::stod(argv[1]);
            Ki = std::stod(argv[2]);
            Kd = std::stod(argv[3]);
            steps = std::stoi(argv[4]);
            target1 = std::stod(argv[5]);
            target2 = std::stod(argv[6]);
            switch_step = std::stoi(argv[7]);
        }
        catch (...)
        {
            std::cerr << "Invalid arguments. Using defaults." << std::endl;
        }
    }

    // 3. Setup
    std::ofstream logFile("telemetry.csv");
    logFile << "Time,Target,Actual,Output\n";

    double dt = 0.1;
    PID pid(Kp, Ki, Kd, dt, 500.0, -500.0);

    MockSensor altimeter(0.0);
    altimeter.init();

    // 4. Run Loop
    for (int i = 0; i < steps; i++)
    {
        // DYNAMIC TARGET LOGIC
        double current_target = (i < switch_step) ? target1 : target2;

        double current_alt = altimeter.readValue();
        double motor_power = pid.calculate(current_target, current_alt);
        altimeter.update(motor_power * dt);

        logFile << i * dt << "," << current_target << "," << current_alt << "," << motor_power << "\n";
    }

    logFile.close();
    return 0;
}