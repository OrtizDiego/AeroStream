#include "MockSensor.hpp"
#include <iostream>

namespace aerostream {

MockSensor::MockSensor(double initial_value, double sigma)
    : _value(initial_value),
      _sigma(sigma),
      _rng(std::random_device{}()),
      _dist(0.0, sigma > 0.0 ? sigma : 1.0)
{
}

MockSensor::MockSensor(double initial_value, double sigma, unsigned int seed)
    : _value(initial_value),
      _sigma(sigma),
      _rng(seed),
      _dist(0.0, sigma > 0.0 ? sigma : 1.0)
{
}

void MockSensor::init()
{
    std::cout << "[MockSensor] Initialized. sigma=" << _sigma << " m\n";
}

double MockSensor::readValue()
{
    // std::normal_distribution requires sigma > 0; sigma <= 0 means "ideal sensor".
    if (_sigma <= 0.0) {
        return _value;
    }
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
