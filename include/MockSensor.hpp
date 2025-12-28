#pragma once
#include "ISensor.hpp"

class MockSensor : public ISensor
{
public:
    MockSensor(double initial_value);
    void init() override;
    double readValue() override;

    // Helper to update the internal state (simulating physics)
    void update(double step_value);

private:
    double _value;
};