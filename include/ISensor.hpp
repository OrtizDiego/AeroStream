#pragma once

namespace aerostream {

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual void init() = 0;
    virtual double readValue() = 0;
};

} // namespace aerostream
