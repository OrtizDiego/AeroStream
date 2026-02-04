#include <gtest/gtest.h>
#include "PhysicsEngine.hpp"

// Test 1: Does it fall due to gravity when force is 0?
TEST(PhysicsEngineTest, GravityWorks)
{
    PhysicsEngine physics(1.0, 0.0, 0.1); // No drag
    // Initial v=0, pos=0.
    // Force = 0.
    // F_net = -mg = -9.81
    // a = -9.81
    // dt = 1.0
    // v_new = 0 + (-9.81) * 1 = -9.81
    // pos_new = 0 + (-9.81) * 1 = -9.81 (Should be clamped to 0 by ground collision?)

    // NOTE: Our implementation updates velocity THEN position.
    // And it has ground collision check.
    // Let's set initial position high so it can fall.
    // But we don't have a setter for position.
    // We can simulate it falling from 0?
    // Implementation: if (_position < 0.0) _position = 0.0;

    // So if we start at 0 and gravity pulls down, it stays at 0.
    // We need to allow it to lift off first or test the force calculation implicitly.

    // Let's apply a force > gravity to lift it, then stop force and see if it slows down.

    double dt = 0.1;
    // Lift off: Force = 20N. (Gravity is ~9.8N). Net ~ 10.2N. a ~ 10.2 m/s^2.
    physics.update(20.0, dt);

    EXPECT_GT(physics.getPosition(), 0.0);
    EXPECT_GT(physics.getVelocity(), 0.0);

    double v_after_lift = physics.getVelocity();

    // Now apply 0 force. Gravity should reduce velocity.
    physics.update(0.0, dt);

    EXPECT_LT(physics.getVelocity(), v_after_lift);
}

// Test 2: Hover check
TEST(PhysicsEngineTest, HoverCheck)
{
    // Mass 1.0 -> Gravity force 9.81 N.
    // To hover (v=constant), we need Force = 9.81 N (ignoring drag if v=0).
    PhysicsEngine physics(1.0, 0.0, 0.1);

    // Get it off the ground first
    physics.update(20.0, 0.1);
    physics.update(20.0, 0.1);

    double v_start = physics.getVelocity();

    // Apply exact gravity compensation
    physics.update(9.81, 0.1);

    // Velocity should be roughly unchanged (acceleration ~ 0)
    EXPECT_NEAR(physics.getVelocity(), v_start, 0.001);
}
