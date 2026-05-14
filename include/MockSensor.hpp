#pragma once
#include "ISensor.hpp"
#include <random>

namespace aerostream {

class MockSensor : public ISensor {
public:
    explicit MockSensor(double initial_value, double sigma = 0.5);
    void init() override;
    double readValue() override;
    void setValue(double v);

    [[deprecated("use setValue() + readValue() instead")]]
    void update(double step_value);

private:
    double _value;
    double _sigma;
    std::mt19937 _rng;
    std::normal_distribution<double> _dist;
};

} // namespace aerostream
