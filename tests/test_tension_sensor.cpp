// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Alex Conesa

#include <gtest/gtest.h>

#include <loadcell/tension_sensor.h>

#include <cmath>

#include "mock_adc24.h"

namespace {

    using ungula::LoadCell;
    using ungula::TensionSensor;
    using FU = LoadCell::ForceUnit;

    constexpr float kCountsPerNewton = 1000.0F;

    class TensionSensorTest : public ::testing::Test {
        protected:
            void SetUp() override {
                adc.setInitialized(true);
                cell.setCountsPerNewton(kCountsPerNewton);
                cell.setOffset(0);
            }

            MockAdc24 adc;
            LoadCell cell{adc};
            TensionSensor sensor{cell};
    };

    // ---- Initial state ----

    TEST_F(TensionSensorTest, InitialStateIsUnstableWithNan) {
        EXPECT_FALSE(sensor.isStable());
        EXPECT_TRUE(std::isnan(sensor.lastTension()));
        EXPECT_TRUE(std::isnan(sensor.filteredTension()));
        EXPECT_TRUE(std::isnan(sensor.lastVariation()));
    }

    TEST_F(TensionSensorTest, TargetInitiallyNan) {
        EXPECT_TRUE(std::isnan(sensor.target()));
        EXPECT_TRUE(std::isnan(sensor.errorToTarget()));
        EXPECT_FALSE(sensor.isAtTarget());
        EXPECT_FALSE(sensor.isAboveTarget());
        EXPECT_FALSE(sensor.isBelowTarget());
    }

    // ---- readInstant ----

    TEST_F(TensionSensorTest, ReadInstantReturnsFalseWhenNoSample) {
        float t = 0.0F;
        EXPECT_FALSE(sensor.readInstant(t));
    }

    TEST_F(TensionSensorTest, ReadInstantReturnsConvertedValue) {
        adc.pushSample(5000);
        float t = 0.0F;
        EXPECT_TRUE(sensor.readInstant(t));
        EXPECT_FLOAT_EQ(t, 5.0F);
        EXPECT_FLOAT_EQ(sensor.lastTension(), 5.0F);
    }

    TEST_F(TensionSensorTest, ReadInstantRespectsConfiguredUnit) {
        TensionSensor::Config cfg;
        cfg.unit = FU::KGF;
        sensor.configure(cfg);

        cell.setCountsPerKgf(kCountsPerNewton);

        adc.pushSample(5000);
        float t = 0.0F;
        EXPECT_TRUE(sensor.readInstant(t));
        EXPECT_FLOAT_EQ(sensor.lastTension(), 5.0F);
    }

    // ---- Filtering (EMA) ----

    TEST_F(TensionSensorTest, FirstSampleBecomesFilteredDirectly) {
        adc.pushSample(3000);
        float t = 0.0F;
        sensor.readInstant(t);
        EXPECT_FLOAT_EQ(sensor.filteredTension(), 3.0F);
    }

    TEST_F(TensionSensorTest, SubsequentSamplesAreExponentiallyFiltered) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 4;
        sensor.configure(cfg);

        // alpha = 2/(4+1) = 0.4
        adc.pushSample(1000);
        float t = 0.0F;
        sensor.readInstant(t);
        EXPECT_FLOAT_EQ(sensor.filteredTension(), 1.0F);

        adc.pushSample(2000);
        sensor.readInstant(t);
        // filtered = 0.4*2.0 + 0.6*1.0 = 1.4
        EXPECT_NEAR(sensor.filteredTension(), 1.4F, 0.001F);
    }

    // ---- Stability ----

    TEST_F(TensionSensorTest, StabilityDetectedWhenVariationWithinTolerance) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.stabilityTolerance = 0.1F;
        sensor.configure(cfg);

        // With averageSamples=1, alpha=2/2=1.0, so filtered = sample directly
        adc.pushSample(5000);
        float t = 0.0F;
        sensor.readInstant(t);
        EXPECT_FALSE(sensor.isStable());

        adc.pushSample(5050);
        sensor.readInstant(t);
        EXPECT_TRUE(sensor.isStable());
        EXPECT_NEAR(sensor.lastVariation(), 0.05F, 0.001F);
    }

    TEST_F(TensionSensorTest, UnstableWhenVariationExceedsTolerance) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.stabilityTolerance = 0.01F;
        sensor.configure(cfg);

        adc.pushSample(5000);
        float t = 0.0F;
        sensor.readInstant(t);

        adc.pushSample(6000);
        sensor.readInstant(t);
        EXPECT_FALSE(sensor.isStable());
    }

    // ---- Target tracking ----

    TEST_F(TensionSensorTest, SetTargetAndCheckError) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.targetTolerance = 0.1F;
        sensor.configure(cfg);
        sensor.setTarget(5.0F);

        adc.pushSample(5000);
        float t = 0.0F;
        sensor.readInstant(t);

        EXPECT_FLOAT_EQ(sensor.target(), 5.0F);
        EXPECT_NEAR(sensor.errorToTarget(), 0.0F, 0.001F);
        EXPECT_TRUE(sensor.isAtTarget());
        EXPECT_FALSE(sensor.isAboveTarget());
        EXPECT_FALSE(sensor.isBelowTarget());
    }

    TEST_F(TensionSensorTest, BelowTarget) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.targetTolerance = 0.05F;
        sensor.configure(cfg);
        sensor.setTarget(5.0F);

        adc.pushSample(4000);
        float t = 0.0F;
        sensor.readInstant(t);

        EXPECT_TRUE(sensor.isBelowTarget());
        EXPECT_FALSE(sensor.isAtTarget());
        EXPECT_FALSE(sensor.isAboveTarget());
    }

    TEST_F(TensionSensorTest, AboveTarget) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.targetTolerance = 0.05F;
        sensor.configure(cfg);
        sensor.setTarget(5.0F);

        adc.pushSample(6000);
        float t = 0.0F;
        sensor.readInstant(t);

        EXPECT_TRUE(sensor.isAboveTarget());
        EXPECT_FALSE(sensor.isAtTarget());
        EXPECT_FALSE(sensor.isBelowTarget());
    }

    TEST_F(TensionSensorTest, AtTargetWithinTolerance) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.targetTolerance = 0.1F;
        sensor.configure(cfg);
        sensor.setTarget(5.0F);

        adc.pushSample(5080);
        float t = 0.0F;
        sensor.readInstant(t);

        EXPECT_TRUE(sensor.isAtTarget());
        EXPECT_FALSE(sensor.isAboveTarget());
        EXPECT_FALSE(sensor.isBelowTarget());
    }

    // ---- readStable ----

    TEST_F(TensionSensorTest, ReadStableReturnsTrueWhenStabilizes) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.stabilityTolerance = 0.1F;
        sensor.configure(cfg);

        adc.pushSamples({5000, 5020, 5010});
        float t = 0.0F;
        EXPECT_TRUE(sensor.readStable(t, 1000));
        EXPECT_NEAR(t, 5.02F, 0.01F);
    }

    TEST_F(TensionSensorTest, ReadStableReturnsFalseOnTimeout) {
        TensionSensor::Config cfg;
        cfg.averageSamples = 1;
        cfg.stabilityTolerance = 0.001F;
        sensor.configure(cfg);

        adc.pushSamples({1000, 5000});
        float t = 0.0F;
        EXPECT_FALSE(sensor.readStable(t, 50));
    }

    // ---- No target set ----

    TEST_F(TensionSensorTest, TargetQueriesReturnFalseWithoutTarget) {
        adc.pushSample(5000);
        float t = 0.0F;
        sensor.readInstant(t);

        EXPECT_FALSE(sensor.isAtTarget());
        EXPECT_FALSE(sensor.isAboveTarget());
        EXPECT_FALSE(sensor.isBelowTarget());
        EXPECT_TRUE(std::isnan(sensor.errorToTarget()));
    }

}  // namespace
