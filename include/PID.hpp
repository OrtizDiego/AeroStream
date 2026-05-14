#pragma once
#include "IController.hpp"

namespace aerostream {

class PID : public IController {
public:
    PID(double kp, double ki, double kd, double dt,
        double max_output, double min_output, double N = 10.0);

    double calculate(double setpoint, double pv) override;
    void reset() override;

private:
    double _kp, _ki, _kd, _dt;
    double _max_output, _min_output;
    double _N;
    double _pre_error;
    double _integral;
    double _d_filtered;
};

} // namespace aerostream
