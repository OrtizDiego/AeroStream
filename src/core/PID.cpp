#include "PID.hpp"
#include <algorithm> // for std::clamp (C++17)

PID::PID(double kp, double ki, double kd, double dt, double max_output, double min_output)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt), _max_output(max_output), _min_output(min_output), _pre_error(0), _integral(0) 
{
}

double PID::calculate(double setpoint, double pv) {
    
    // 1. Calculate Error
    double error = setpoint - pv;

    // 2. Proportional Term
    double P = _kp * error;

    // 3. Integral Term (Accumulate error over time)
    _integral += error * _dt;
    double I = _ki * _integral;

    // 4. Derivative Term (Rate of change of error)
    double derivative = (error - _pre_error) / _dt;
    double D = _kd * derivative;

    // 5. Calculate Total Output
    double output = P + I + D;

    // 6. Clamp output to hardware limits (Safety!)
    output = std::clamp(output, _min_output, _max_output);

    // 7. Save error for next loop
    _pre_error = error;

    return output;
}

void PID::reset() {
    _integral = 0;
    _pre_error = 0;
}

PID::~PID() {}
