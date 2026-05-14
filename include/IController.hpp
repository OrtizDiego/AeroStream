#pragma once

namespace aerostream {

class IController {
public:
    virtual ~IController() = default;
    virtual double calculate(double setpoint, double pv) = 0;
    virtual void reset() = 0;
};

} // namespace aerostream
