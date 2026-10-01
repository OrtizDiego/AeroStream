#include "PID.hpp"
#include <algorithm>
#include <cmath>

namespace aerostream {

PID::PID(double kp, double ki, double kd, double dt,
         double max_output, double min_output, double N, double i_zone)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt),
      _max_output(max_output), _min_output(min_output),
      _N(N), _i_zone(i_zone), _pre_pv(0.0), _integral(0.0), _d_filtered(0.0), _initialized(false)
{
}

double PID::calculate(double setpoint, double pv) {
    double error = setpoint - pv;

    // Seed the derivative history on the first call so the first sample
    // does not see a fake jump from 0 to pv.
    if (!_initialized) {
        _pre_pv = pv;
        _initialized = true;
    }

    double P = _kp * error;

    double tentative_integral = _integral + error * _dt;

    // Derivative on measurement (-d(pv)/dt) instead of on error: setpoint
    // steps no longer produce a derivative kick. The raw derivative is
    // passed through a first-order low-pass filter (bandwidth N rad/s) to
    // attenuate sensor noise; smaller N = heavier filtering, more lag.
    double raw_derivative = -(pv - _pre_pv) / _dt;
    double alpha = _N * _dt / (1.0 + _N * _dt);
    _d_filtered = alpha * raw_derivative + (1.0 - alpha) * _d_filtered;
    double D = _kd * _d_filtered;

    double raw_output = P + _ki * tentative_integral + D;

    // Anti-windup:
    //  1. Clamping - only integrate while the output is not saturated. While
    //     the actuator is pinned at a limit the loop is effectively open.
    //  2. Integral zone (optional, i_zone > 0) - only integrate when
    //     |error| < i_zone. Error accumulated during a large transient would
    //     otherwise have to be unwound afterwards as overshoot and a slow creep
    //     back to the setpoint.
    bool saturated = (raw_output > _max_output) || (raw_output < _min_output);
    bool outside_zone = (_i_zone > 0.0) && (std::abs(error) > _i_zone);
    if (!saturated && !outside_zone) {
        _integral = tentative_integral;
    }

    // Use the committed integral so a frozen step really adds nothing.
    double output = std::clamp(P + _ki * _integral + D, _min_output, _max_output);

    _pre_pv = pv;
    return output;
}

void PID::reset() {
    _integral = 0.0;
    _pre_pv = 0.0;
    _d_filtered = 0.0;
    _initialized = false;
}

} // namespace aerostream
