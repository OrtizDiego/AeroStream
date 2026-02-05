#include "PhysicsEngine.hpp"
#include <cmath>
#include <iostream>

PhysicsEngine::PhysicsEngine(double mass, double drag_coefficient, double area)
    : _mass(mass), _cd(drag_coefficient), _area(area), _velocity(0.0), _position(0.0)
{
}

void PhysicsEngine::update(double motor_force, double dt)
{
    // 1. Calculate Gravity Force (downward)
    double F_gravity = -_mass * GRAVITY;

    // 2. Calculate Drag Force (opposing velocity)
    // F_drag = 0.5 * rho * v^2 * Cd * A
    // Direction is opposite to velocity: -v / |v|
    double v_abs = std::abs(_velocity);
    double F_drag = 0.0;
    if (v_abs > 0.0001) {
        F_drag = -0.5 * AIR_DENSITY * _velocity * v_abs * _cd * _area;
    }

    // 3. Total Force
    // We assume motor_force is always positive (upwards) relative to the drone body,
    // aligned with the vertical axis for this 1D simulation.
    // However, if the PID output can be negative (reversible motors?), we should respect that.
    // In this context, let's assume motor_force is the thrust.
    double F_total = F_gravity + F_drag + motor_force;

    // 4. Newton's Second Law: F = ma  ->  a = F / m
    double acceleration = F_total / _mass;

    // 5. Integrate Acceleration to get Velocity
    _velocity += acceleration * dt;

    // 6. Integrate Velocity to get Position
    _position += _velocity * dt;

    // 7. Ground collision (simple constraint)
    if (_position < 0.0) {
        _position = 0.0;
        _velocity = 0.0; // Stop if we hit the ground
        // In a real bounce, we might reverse velocity with restitution
    }
}

double PhysicsEngine::getPosition() const {
    return _position;
}

double PhysicsEngine::getVelocity() const {
    return _velocity;
}

void PhysicsEngine::reset() {
    _velocity = 0;
    _position = 0;
}
