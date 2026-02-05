#include <gtest/gtest.h>
#include "PID.hpp"

// Test Feedforward: Does it add to output?
TEST(PIDTest, FeedforwardAddsToOutput)
{
    // Kp=0, Ki=0, Kd=0. Max output 100.
    PID pid(0.0, 0.0, 0.0, 0.1, 100.0, -100.0);

    // With 0 gains, output should exactly equal feedforward
    double ff = 10.0;
    double output = pid.calculate(50.0, 50.0, ff);

    EXPECT_DOUBLE_EQ(output, 10.0);
}

// Test Feedforward with Clamping
TEST(PIDTest, FeedforwardRespectsLimits)
{
    PID pid(0.0, 0.0, 0.0, 0.1, 10.0, -10.0);
    double ff = 20.0;
    double output = pid.calculate(50.0, 50.0, ff);

    EXPECT_DOUBLE_EQ(output, 10.0);
}
