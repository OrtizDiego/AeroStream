#pragma once

// Abstract Base Class
class ISensor {
public:
    virtual ~ISensor() {}

    // Pure virtual function: any sensor MUST implement this
    virtual void init() = 0;
    virtual double readValue() = 0;
};