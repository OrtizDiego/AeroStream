#include "MockSensor.hpp"
#include <iostream>
#include <cstdlib> // For rand()

MockSensor::MockSensor(double initial_value) : _value(initial_value) {}

void MockSensor::init()
{
    std::cout << "[MockSensor] Initialized and calibrated." << std::endl;
}

double MockSensor::readValue()
{
    // Simulate sensor noise: +/- 0.5 meters random fluctuation
    double noise = ((std::rand() % 100) / 100.0) - 0.5;
    return _value + noise;
}

void MockSensor::update(double step_value)
{
    _value += step_value;
}