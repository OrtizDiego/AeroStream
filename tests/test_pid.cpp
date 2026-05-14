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
