#include "PhysicsEngine.hpp"
#include <cmath>

namespace aerostream {

PhysicsEngine::PhysicsEngine(double mass, double drag_coefficient, double area)
    : _mass(mass), _cd(drag_coefficient), _area(area), _velocity(0.0), _position(0.0)
{
}

void PhysicsEngine::update(double motor_force, double dt)
{
    double F_gravity = -_mass * GRAVITY;

    double v_abs = std::abs(_velocity);
    double F_drag = 0.0;
    if (v_abs > 0.0001) {
        F_drag = -0.5 * AIR_DENSITY * _velocity * v_abs * _cd * _area;
    }

    double F_total = F_gravity + F_drag + motor_force;
    double acceleration = F_total / _mass;

    _velocity += acceleration * dt;
    _position += _velocity * dt;

    if (_position < 0.0) {
        _position = 0.0;
        _velocity = 0.0;
    }
}

double PhysicsEngine::getPosition() const {
    return _position;
}

double PhysicsEngine::getVelocity() const {
    return _velocity;
}

void PhysicsEngine::reset() {
    _velocity = 0.0;
    _position = 0.0;
}

void PhysicsEngine::setState(double position, double velocity) {
    _position = position;
    _velocity = velocity;
}

} // namespace aerostream
