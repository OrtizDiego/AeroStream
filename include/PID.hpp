#pragma once

class PID {
public:
    // Constructor: Takes the 3 controller gains and a limit for the output
    PID(double kp, double ki, double kd, double dt, double max_output, double min_output);

    // The main function that computes the control signal
    double calculate(double setpoint, double pv);

    // Resets the integral error (useful when turning the system off/on)
    void reset();

    // Destructor
    ~PID();

private:
    double _kp;
    double _ki;
    double _kd;
    double _dt;          // Time step (loop interval)
    double _max_output;  // Saturation limits (Motor limits)
    double _min_output;

    double _pre_error;   // Previous error for Derivative term
    double _integral;    // Accumulated error for Integral term
};
