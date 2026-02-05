#pragma once

class PhysicsEngine {
public:
    PhysicsEngine(double mass = 1.0, double drag_coefficient = 0.5, double area = 0.1);

    // Updates the physics state based on applied force and delta time
    void update(double force, double dt);

    // Returns current position (altitude)
    double getPosition() const;

    // Returns current velocity
    double getVelocity() const;

    // Reset state
    void reset();

private:
    double _mass;      // kg
    double _cd;        // Drag coefficient
    double _area;      // Cross-sectional area (m^2)
    double _velocity;  // m/s
    double _position;  // m (Altitude)

    const double GRAVITY = 9.81; // m/s^2
    const double AIR_DENSITY = 1.225; // kg/m^3 (approx at sea level)
};
