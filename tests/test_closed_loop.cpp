#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "PID.hpp"
#include "PhysicsEngine.hpp"

using namespace aerostream;

namespace {

// Same loop as src/main.cpp with an ideal sensor. Returns the true altitude
// trace sampled every step.
std::vector<double> fly(double kp, double ki, double kd, double target,
                        bool hover_ff, int steps, double i_zone = 5.0)
{
    const double dt = 0.1;
    const double mass = 1.0;
    const double hover = hover_ff ? mass * 9.81 : 0.0;
    PID pid(kp, ki, kd, dt, 50.0 - hover, 0.0 - hover, 5.0, i_zone);
    PhysicsEngine physics(mass, 0.5, 0.1);
    std::vector<double> alt;
    for (int i = 0; i < steps; i++) {
        double force = hover + pid.calculate(target, physics.getPosition());
        physics.update(force, dt);
        alt.push_back(physics.getPosition());
    }
    return alt;
}

} // namespace

TEST(ClosedLoopTest, DefaultGainsSettleWithoutSustainedOscillation)
{
    const double target = 100.0;
    auto alt = fly(3.0, 0.2, 2.0, target, true, 600); // 60 s

    // Overshoot below 5% of the step
    EXPECT_LT(*std::max_element(alt.begin(), alt.end()), target * 1.05);

    // Within ±2% of the target from 10 s on, within 5 cm from 20 s on:
    // no sustained oscillation and no slow integral creep.
    for (size_t i = 100; i < alt.size(); i++) {
        ASSERT_NEAR(alt[i], target, target * 0.02) << "at step " << i;
    }
    for (size_t i = 200; i < alt.size(); i++) {
        ASSERT_NEAR(alt[i], target, 0.05) << "at step " << i;
    }
}

TEST(ClosedLoopTest, IntegralRemovesGravityOffsetWithoutFeedForward)
{
    // Without feed-forward, a P-D controller alone would hover 9.81/Kp below
    // the target; the integral term must remove that offset.
    auto alt = fly(3.0, 0.2, 2.0, 50.0, false, 600);
    EXPECT_NEAR(alt.back(), 50.0, 0.1);
}
