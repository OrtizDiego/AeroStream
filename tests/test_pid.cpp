#include <gtest/gtest.h>
#include "PID.hpp"

using namespace aerostream;

TEST(PIDTest, ZeroErrorYieldsZeroOutput)
{
    PID pid(1.0, 0.1, 0.01, 0.1, 100.0, -100.0);
    EXPECT_NEAR(pid.calculate(10.0, 10.0), 0.0, 0.001);
}

TEST(PIDTest, ProportionalAction)
{
    // Kp=2.0, Ki=0, Kd=0. Error=(10-5)=5. Output=2.0*5=10.
    PID pid(2.0, 0.0, 0.0, 0.1, 100.0, -100.0);
    EXPECT_NEAR(pid.calculate(10.0, 5.0), 10.0, 0.001);
}

TEST(PIDTest, MaxOutputLimit)
{
    PID pid(1000.0, 0.0, 0.0, 0.1, 50.0, -50.0);
    EXPECT_EQ(pid.calculate(100.0, 0.0), 50.0);
}

TEST(PIDTest, IntegralWindupProtection)
{
    PID pid(1.0, 1.0, 0.0, 0.1, 10.0, -10.0);

    for (int i = 0; i < 100; i++) {
        pid.calculate(100.0, 0.0);
    }

    double output = pid.calculate(90.0, 100.0);
    EXPECT_LT(output, 5.0);
}

TEST(PIDTest, NoDerivativeKickOnSetpointStep)
{
    // Derivative acts on the measurement, so a setpoint jump with a constant
    // measurement must change the output only through P (and I).
    PID pid(1.0, 0.0, 5.0, 0.1, 1000.0, -1000.0);
    pid.calculate(10.0, 10.0);
    EXPECT_NEAR(pid.calculate(60.0, 10.0), 50.0, 1e-9);
}

TEST(PIDTest, FirstCallHasNoDerivativeKick)
{
    // The first sample must not see a fake jump from 0 to pv.
    PID pid(0.0, 0.0, 5.0, 0.1, 1000.0, -1000.0);
    EXPECT_NEAR(pid.calculate(100.0, 40.0), 0.0, 1e-9);
}

TEST(PIDTest, DerivativeOpposesRisingMeasurement)
{
    PID pid(0.0, 0.0, 1.0, 0.1, 1000.0, -1000.0);
    pid.calculate(50.0, 10.0);
    EXPECT_LT(pid.calculate(50.0, 11.0), 0.0);
}

TEST(PIDTest, ResetClearsState)
{
    PID pid(1.0, 1.0, 1.0, 0.1, 1000.0, -1000.0);
    for (int i = 0; i < 10; i++) {
        pid.calculate(10.0, static_cast<double>(i));
    }
    pid.reset();
    PID fresh(1.0, 1.0, 1.0, 0.1, 1000.0, -1000.0);
    EXPECT_NEAR(pid.calculate(10.0, 3.0), fresh.calculate(10.0, 3.0), 1e-12);
}

TEST(PIDTest, IntegralZoneFreezesIntegralForLargeErrors)
{
    // Ki only, unsaturated: with i_zone = 5 an error of 10 must not accumulate.
    PID pid(0.0, 1.0, 0.0, 0.1, 1000.0, -1000.0, 10.0, 5.0);
    for (int i = 0; i < 10; i++) {
        EXPECT_NEAR(pid.calculate(10.0, 0.0), 0.0, 1e-12);
    }
    // Inside the zone it integrates normally: 1.0 * (0 + 2 * 0.1) = 0.2
    EXPECT_NEAR(pid.calculate(2.0, 0.0), 0.2, 1e-12);
}
