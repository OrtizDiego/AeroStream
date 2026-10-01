#pragma once
#include "IController.hpp"

namespace aerostream {

class PID : public IController {
public:
    // N:      derivative low-pass filter bandwidth (rad/s)
    // i_zone: integrate only while |error| < i_zone (<= 0 disables the zone)
    PID(double kp, double ki, double kd, double dt,
        double max_output, double min_output, double N = 10.0, double i_zone = 0.0);

    double calculate(double setpoint, double pv) override;
    void reset() override;

private:
    double _kp, _ki, _kd, _dt;
    double _max_output, _min_output;
    double _N;
    double _i_zone;
    double _pre_pv;
    double _integral;
    double _d_filtered;
    bool _initialized;
};

} // namespace aerostream
