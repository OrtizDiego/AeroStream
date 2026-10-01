#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include "PID.hpp"
#include "MockSensor.hpp"
#include "PhysicsEngine.hpp"

using namespace aerostream;

// Usage:
//   flight_controller <Kp> <Ki> <Kd> <steps> <target1> <target2> <switch_step>
//                     [noise_sigma=0.5] [filter_N=5] [seed=-1] [hover_ff=1]
//                     [i_zone=5]
//
//   seed < 0      -> non-deterministic noise (std::random_device)
//   hover_ff != 0 -> add gravity-compensating feed-forward thrust (m*g)
//   i_zone        -> integrate only while |error| < i_zone metres (0 = always)
int main(int argc, char *argv[])
{
    double Kp = 3.0, Ki = 0.2, Kd = 2.0;
    int steps = 1000;
    double target1 = 50.0;
    double target2 = 100.0;
    int switch_step = 500;
    double noise_sigma = 0.5;
    double filter_N = 5.0;
    long seed = -1;
    bool hover_ff = true;
    double i_zone = 5.0;

    if (argc >= 8)
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
            if (argc >= 9)  { noise_sigma = std::stod(argv[8]); }
            if (argc >= 10) { filter_N    = std::stod(argv[9]); }
            if (argc >= 11) { seed        = std::stol(argv[10]); }
            if (argc >= 12) { hover_ff    = std::stoi(argv[11]) != 0; }
            if (argc >= 13) { i_zone      = std::stod(argv[12]); }
        }
        catch (...)
        {
            std::cerr << "Invalid arguments.\n";
            return 1;
        }
    }

    std::ofstream logFile("telemetry.csv");
    // Actual   = true altitude from the physics engine (what the drone really does)
    // Measured = noisy altimeter reading (what the controller sees)
    logFile << "Time,Target,Actual,Measured,Velocity,Output\n";

    const double dt = 0.1;
    const double mass = 1.0;
    const double gravity = 9.81;
    const double max_thrust = 50.0;
    const double min_thrust = 0.0;

    // With feed-forward the PID only has to produce the *correction* around
    // hover thrust, so its limits are shifted to keep total thrust in range.
    const double hover_thrust = hover_ff ? mass * gravity : 0.0;
    PID pid(Kp, Ki, Kd, dt, max_thrust - hover_thrust, min_thrust - hover_thrust,
            filter_N, i_zone);

    MockSensor altimeter = (seed >= 0)
        ? MockSensor(0.0, noise_sigma, static_cast<unsigned int>(seed))
        : MockSensor(0.0, noise_sigma, std::random_device{}());
    altimeter.init();

    PhysicsEngine physics(mass, 0.5, 0.1);

    for (int i = 0; i < steps; i++)
    {
        double current_target = (i < switch_step) ? target1 : target2;

        double true_alt = physics.getPosition();
        double true_vel = physics.getVelocity();

        altimeter.setValue(true_alt);
        double measured_alt = altimeter.readValue();

        double motor_force = hover_thrust + pid.calculate(current_target, measured_alt);
        physics.update(motor_force, dt);

        logFile << i * dt << "," << current_target << "," << true_alt << ","
                << measured_alt << "," << true_vel << "," << motor_force << "\n";
    }

    logFile.close();
    return 0;
}
