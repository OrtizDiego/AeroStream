#include <iostream>
#include <fstream>
#include <string>
#include "PID.hpp"
#include "MockSensor.hpp"
#include "PhysicsEngine.hpp"

using namespace aerostream;

int main(int argc, char *argv[])
{
    double Kp = 0.6, Ki = 0.01, Kd = 0.05;
    int steps = 1000;
    double target1 = 50.0;
    double target2 = 100.0;
    int switch_step = 500;
    double noise_sigma = 0.5;

    if (argc >= 9)
    {
        try
        {
            Kp          = std::stod(argv[1]);
            Ki          = std::stod(argv[2]);
            Kd          = std::stod(argv[3]);
            steps       = std::stoi(argv[4]);
            target1     = std::stod(argv[5]);
            target2     = std::stod(argv[6]);
            switch_step = std::stoi(argv[7]);
            noise_sigma = std::stod(argv[8]);
        }
        catch (...)
        {
            std::cerr << "Invalid arguments. Using defaults.\n";
        }
    }
    else if (argc >= 8)
    {
        try
        {
            Kp          = std::stod(argv[1]);
            Ki          = std::stod(argv[2]);
            Kd          = std::stod(argv[3]);
            steps       = std::stoi(argv[4]);
            target1     = std::stod(argv[5]);
            target2     = std::stod(argv[6]);
            switch_step = std::stoi(argv[7]);
        }
        catch (...)
        {
            std::cerr << "Invalid arguments. Using defaults.\n";
        }
    }

    std::ofstream logFile("telemetry.csv");
    logFile << "Time,Target,Actual,Velocity,Output\n";

    const double dt = 0.1;

    PID pid(Kp, Ki, Kd, dt, 50.0, 0.0);
    MockSensor altimeter(0.0, noise_sigma);
    altimeter.init();

    PhysicsEngine physics(1.0, 0.5, 0.1);

    for (int i = 0; i < steps; i++)
    {
        double current_target = (i < switch_step) ? target1 : target2;

        double true_alt = physics.getPosition();
        double true_vel = physics.getVelocity();

        altimeter.setValue(true_alt);
        double measured_alt = altimeter.readValue();

        double motor_force = pid.calculate(current_target, measured_alt);
        physics.update(motor_force, dt);

        logFile << i * dt << "," << current_target << "," << measured_alt << ","
                << true_vel << "," << motor_force << "\n";
    }

    logFile.close();
    return 0;
}
