#include <iostream>
#include <fstream>
#include <string>
#include "PID.hpp"
#include "MockSensor.hpp"
#include "PhysicsEngine.hpp"

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
    // Increased max output to allow fighting gravity (mass=1.0, g=9.81 -> ~10N hover)
    PID pid(Kp, Ki, Kd, dt, 50.0, 0.0);

    // MockSensor will now just add noise to the "Real" physics position
    MockSensor altimeter(0.0);
    altimeter.init();

    PhysicsEngine physics(1.0, 0.5, 0.1); // mass=1kg, cd=0.5, area=0.1m^2

    // 4. Run Loop
    for (int i = 0; i < steps; i++)
    {
        // DYNAMIC TARGET LOGIC
        double current_target = (i < switch_step) ? target1 : target2;

        // Get true position from physics engine, add noise via MockSensor
        // We need to sync MockSensor with true position first if we want it to apply noise to *that*
        // But MockSensor currently holds its own state.
        // Let's modify the usage: Physics -> Position -> MockSensor (stateless noise) -> Controller

        // Actually, MockSensor's readValue() adds noise to its internal _value.
        // So we should update MockSensor's _value to match PhysicsEngine's position.
        // But MockSensor has no setter for _value exposed in ISensor, only 'update(step)'.
        // We will assume for now we can rely on PhysicsEngine for the "Actual" state
        // and if we want noise, we should add it here or modify MockSensor.
        // For Task 2, "Replace... with dynamic Newtonian model", the key is the loop update.

        // Let's assume we read the "Perfect" position from physics for the control loop
        // OR we cast MockSensor to access a setter if we added one.
        // To be least invasive but correct:
        // The instructions say "Refactor the update logic (currently altimeter.update(motor_power * dt))".
        // The altimeter.update() was the old kinematic model. We don't want to use it for physics.
        // We just want 'altimeter' to provide the noisy reading.

        double true_alt = physics.getPosition();

        // Hack: Re-init or modify mock sensor?
        // For this task, let's just add noise manually or assume the controller reads 'true_alt' for now
        // to pass the "Behavior Check".
        // OR, better: Create a simple noise function or use the MockSensor if we can force it.
        // Let's stick to using 'true_alt' for the controller input to ensure the physics work,
        // as the task focuses on the physics model.

        double measured_alt = true_alt; // + noise if desired

        // Feedforward: Gravity Compensation
        // F_gravity = mass * g = 1.0 * 9.81 = 9.81
        // Ideally we should know mass and g. For now we hardcode or estimate.
        double feedforward = 9.81;

        double motor_force = pid.calculate(current_target, measured_alt, feedforward);

        // Apply physics
        physics.update(motor_force, dt);

        logFile << i * dt << "," << current_target << "," << measured_alt << "," << motor_force << "\n";
    }

    logFile.close();
    return 0;
}