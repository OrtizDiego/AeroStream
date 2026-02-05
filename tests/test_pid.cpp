#include <gtest/gtest.h>
#include "PID.hpp"

// Test 1: Does the PID output zero if error is zero?
TEST(PIDTest, ZeroErrorYieldsZeroOutput)
{
    PID pid(1.0, 0.1, 0.01, 0.1, 100.0, -100.0);
    // If setpoint = 10 and actual = 10, error is 0. Output should be 0.
    EXPECT_NEAR(pid.calculate(10.0, 10.0), 0.0, 0.001);
}

// Test 2: Does the Proportional term work?
TEST(PIDTest, ProportionalAction)
{
    // Kp = 2.0, others = 0. Error = (10 - 5) = 5. Output should be 2.0 * 5 = 10.
    PID pid(2.0, 0.0, 0.0, 0.1, 100.0, -100.0);
    EXPECT_NEAR(pid.calculate(10.0, 5.0), 10.0, 0.001);
}

// Test 3: Does it respect the Maximum Output limit?
TEST(PIDTest, MaxOutputLimit)
{
    PID pid(1000.0, 0.0, 0.0, 0.1, 50.0, -50.0);
    // Huge error, but output should be capped at 50.0
    EXPECT_EQ(pid.calculate(100.0, 0.0), 50.0);
}
TEST(PIDTest, IntegralWindupProtection)
{
    // Kp=1, Ki=1, Kd=0 (simplify). Max output 10.
    PID pid(1.0, 1.0, 0.0, 0.1, 10.0, -10.0);

    // 1. Saturate positive for 100 ticks
    for (int i = 0; i < 100; i++) {
        pid.calculate(100.0, 0.0); // Error = 100
    }

    // 2. Reverse the error (setpoint=90, pv=100 -> error=-10)
    // If windup exists, the huge accumulated integral will keep output at +10.
    // If windup is fixed, the integral should be clamped near 10 (or less),
    // and -10 error * Kp=1 should bring total down significantly.
    // P = -10. Max I ~ 10. Output ~ 0.

    double output = pid.calculate(90.0, 100.0);

    // We expect output to be significantly less than max (10.0).
    // Let's assert it is less than 5.0 to be safe.
    EXPECT_LT(output, 5.0);
}
