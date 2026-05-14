#include <gtest/gtest.h>
#include "MockSensor.hpp"
#include <cmath>

using namespace aerostream;

TEST(MockSensorTest, SetValueRoundTrip)
{
    MockSensor sensor(0.0, 0.0); // sigma=0: no noise
    sensor.setValue(42.0);
    EXPECT_NEAR(sensor.readValue(), 42.0, 1e-9);
}

TEST(MockSensorTest, ZeroSigmaIsNoiseless)
{
    MockSensor sensor(10.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        EXPECT_DOUBLE_EQ(sensor.readValue(), 10.0);
    }
}

TEST(MockSensorTest, NoiseMeanIsNearZero)
{
    MockSensor sensor(0.0, 1.0);
    double sum = 0.0;
    const int N = 10000;
    for (int i = 0; i < N; ++i) {
        sum += sensor.readValue();
    }
    EXPECT_NEAR(sum / N, 0.0, 0.1);
}

TEST(MockSensorTest, NoiseStaysWithinFourSigma)
{
    const double sigma = 0.5;
    MockSensor sensor(0.0, sigma);
    int outliers = 0;
    const int N = 10000;
    for (int i = 0; i < N; ++i) {
        if (std::abs(sensor.readValue()) > 4.0 * sigma) { ++outliers; }
    }
    // P(|X| > 4σ) ≈ 0.0063% → expect ~0-6 outliers in 10000; allow 20 for safety
    EXPECT_LE(outliers, 20);
}

TEST(MockSensorTest, DifferentSeedsProduceDifferentSequences)
{
    MockSensor s1(0.0, 1.0);
    MockSensor s2(0.0, 1.0);
    bool all_identical = true;
    for (int i = 0; i < 20; ++i) {
        if (s1.readValue() != s2.readValue()) {
            all_identical = false;
            break;
        }
    }
    EXPECT_FALSE(all_identical);
}
