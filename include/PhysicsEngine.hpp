#pragma once

namespace aerostream {

class PhysicsEngine {
public:
    PhysicsEngine(double mass = 1.0, double drag_coefficient = 0.5, double area = 0.1);

    void update(double force, double dt);
    double getPosition() const;
    double getVelocity() const;
    void reset();
    void setState(double position, double velocity);

private:
    double _mass;
    double _cd;
    double _area;
    double _velocity;
    double _position;

    static constexpr double GRAVITY = 9.81;
    static constexpr double AIR_DENSITY = 1.225;
};

} // namespace aerostream
