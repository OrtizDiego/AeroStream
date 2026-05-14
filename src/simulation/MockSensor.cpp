#include "MockSensor.hpp"
#include <iostream>

namespace aerostream {

MockSensor::MockSensor(double initial_value, double sigma)
    : _value(initial_value),
      _sigma(sigma),
      _rng(std::random_device{}()),
      _dist(0.0, sigma)
{
}

void MockSensor::init()
{
    std::cout << "[MockSensor] Initialized. sigma=" << _sigma << " m\n";
}

double MockSensor::readValue()
{
    return _value + _dist(_rng);
}

void MockSensor::setValue(double v)
{
    _value = v;
}

void MockSensor::update(double step_value)
{
    _value += step_value;
}

} // namespace aerostream
