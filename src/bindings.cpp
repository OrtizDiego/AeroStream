#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <map>
#include <string>

#include "PID.hpp"
#include "PhysicsEngine.hpp"

namespace py = pybind11;

// Function to run the simulation and return data directly to Python
// Returns a map of string -> vector<double> (Time, Target, Actual, Output)
std::map<std::string, std::vector<double>> run_simulation(
    double Kp, double Ki, double Kd,
    int steps,
    double target1, double target2, int switch_step,
    double feedforward_val)
{
    // Simulation Parameters
    double dt = 0.1;
    // Max output enough to hover (mass 1.0 -> gravity ~9.81)
    PID pid(Kp, Ki, Kd, dt, 50.0, 0.0);

    PhysicsEngine physics(1.0, 0.5, 0.1);

    // Data containers
    std::vector<double> time_log;
    std::vector<double> target_log;
    std::vector<double> actual_log;
    std::vector<double> output_log;

    time_log.reserve(steps);
    target_log.reserve(steps);
    actual_log.reserve(steps);
    output_log.reserve(steps);

    for (int i = 0; i < steps; i++)
    {
        double current_target = (i < switch_step) ? target1 : target2;
        double current_alt = physics.getPosition();

        // Use the passed feedforward value (e.g. 9.81 for gravity compensation)
        double motor_force = pid.calculate(current_target, current_alt, feedforward_val);

        physics.update(motor_force, dt);

        // Store data
        time_log.push_back(i * dt);
        target_log.push_back(current_target);
        actual_log.push_back(current_alt);
        output_log.push_back(motor_force);
    }

    std::map<std::string, std::vector<double>> result;
    result["time"] = time_log;
    result["target"] = target_log;
    result["actual"] = actual_log;
    result["output"] = output_log;

    return result;
}

PYBIND11_MODULE(aerostream_core, m) {
    m.doc() = "AeroStream Core C++ Simulation Module";

    m.def("run_simulation", &run_simulation, "Run the flight simulation",
          py::arg("Kp"), py::arg("Ki"), py::arg("Kd"),
          py::arg("steps"),
          py::arg("target1"), py::arg("target2"), py::arg("switch_step"),
          py::arg("feedforward") = 9.81);
}
