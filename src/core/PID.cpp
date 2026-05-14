#include "PID.hpp"
#include <algorithm>

namespace aerostream {

PID::PID(double kp, double ki, double kd, double dt,
         double max_output, double min_output, double N)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt),
      _max_output(max_output), _min_output(min_output),
      _N(N), _pre_error(0.0), _integral(0.0), _d_filtered(0.0)
{
}

double PID::calculate(double setpoint, double pv) {
    double error = setpoint - pv;

    double P = _kp * error;

    double tentative_integral = _integral + error * _dt;
    double I = _ki * tentative_integral;

    // First-order low-pass filtered derivative (eliminates kick on step inputs)
    double raw_derivative = (error - _pre_error) / _dt;
    double alpha = _N * _dt / (1.0 + _N * _dt);
    _d_filtered = alpha * raw_derivative + (1.0 - alpha) * _d_filtered;
    double D = _kd * _d_filtered;

    double output = P + I + D;

    double raw_output = output;
    output = std::clamp(output, _min_output, _max_output);

    bool saturated = (output != raw_output);
    bool same_sign = false;
    if (output >= _max_output && error > 0) { same_sign = true; }
    else if (output <= _min_output && error < 0) { same_sign = true; }

    if (!saturated || !same_sign) {
        _integral = tentative_integral;
    }

    _pre_error = error;
    return output;
}

void PID::reset() {
    _integral = 0.0;
    _pre_error = 0.0;
    _d_filtered = 0.0;
}

} // namespace aerostream
