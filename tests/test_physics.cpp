#include <gtest/gtest.h>
#include "PhysicsEngine.hpp"

using namespace aerostream;

TEST(PhysicsEngineTest, GravityWorks)
{
    PhysicsEngine physics(1.0, 0.0, 0.1); // No drag
    physics.setState(50.0, 0.0);          // Start airborne, zero velocity

    const double dt = 0.1;
    physics.update(0.0, dt);  // Zero force — only gravity acts

    // a = -9.81 m/s², v = 0 + (-9.81)*0.1 = -0.981 m/s
    EXPECT_NEAR(physics.getVelocity(), -0.981, 0.001);
    // pos = 50.0 + (-0.981)*0.1 = 49.9019 m
    EXPECT_NEAR(physics.getPosition(), 49.9019, 0.001);
}

TEST(PhysicsEngineTest, HoverCheck)
{
    // Mass=1 kg, no drag. Gravity force = 9.81 N downward.
    // Applying F=9.81 N upward should yield zero net acceleration.
    PhysicsEngine physics(1.0, 0.0, 0.1);
    physics.setState(50.0, 0.0);

    double v_start = physics.getVelocity(); // 0.0

    physics.update(9.81, 0.1); // Exact gravity compensation

    EXPECT_NEAR(physics.getVelocity(), v_start, 0.001);
}

TEST(PhysicsEngineTest, SetStateRoundTrip)
{
    PhysicsEngine physics;
    physics.setState(123.4, -5.6);
    EXPECT_NEAR(physics.getPosition(), 123.4, 1e-9);
    EXPECT_NEAR(physics.getVelocity(), -5.6, 1e-9);
}

TEST(PhysicsEngineTest, GroundCollisionClampsToZero)
{
    PhysicsEngine physics(1.0, 0.0, 0.1);
    physics.setState(0.05, -5.0); // Very close to ground, moving downward

    physics.update(0.0, 0.1);    // Gravity + downward velocity → hits ground

    EXPECT_GE(physics.getPosition(), 0.0);
    EXPECT_GE(physics.getVelocity(), 0.0);
}
