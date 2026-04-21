// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include <gtest/gtest.h>

#include <loadcell/load_cell.h>

#include <cmath>

#include "mock_adc24.h"

namespace {

    using ungula::LoadCell;
    using MU = LoadCell::ForceUnit;

    // Standard gravity used inside LoadCell — duplicated here to cross-check math.
    constexpr float kG = 9.80665F;

    // ---- Calibration: setCountsPer ----

    TEST(LoadCellTest, UncalibratedReadsReturnNan) {
        MockAdc24 adc;
        LoadCell cell(adc);

        EXPECT_TRUE(std::isnan(cell.rawToUnit(0)));
        EXPECT_TRUE(std::isnan(cell.rawToUnit(1000, MU::KGF)));
        EXPECT_TRUE(std::isnan(cell.netToUnit(1000, MU::NEWTONS)));
    }

    TEST(LoadCellTest, SetCountsPerNewtonsStoresDirectly) {
        MockAdc24 adc;
        LoadCell cell(adc);

        cell.setCountsPerNewton(50000.0F);
        EXPECT_FLOAT_EQ(cell.countsPerNewton(), 50000.0F);
    }

    TEST(LoadCellTest, SetCountsPerKgfConvertsWithGravity) {
        MockAdc24 adc;
        LoadCell cell(adc);

        // counts_per_N = counts_per_kg / g.
        cell.setCountsPerKgf(490332.0F);
        EXPECT_NEAR(cell.countsPerNewton(), 490332.0F / kG, 0.01F);
    }

    // ---- Calibration: calibrate() ----

    TEST(LoadCellTest, CalibrateWithNewtonsComputesScale) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(1000);

        // Apply 10 N, read raw = 1000 + 500000. net = 500000.
        // counts_per_N = 500000 / 10 = 50000.
        cell.calibrate(10.0F, 501000, MU::NEWTONS);
        EXPECT_FLOAT_EQ(cell.countsPerNewton(), 50000.0F);
    }

    TEST(LoadCellTest, CalibrateWithKgfConvertsReferenceToNewtons) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(0);

        // 2 kg -> 2*g N. Raw = 100000. counts_per_N = 100000 / (2*g).
        cell.calibrate(2.0F, 100000, MU::KGF);
        EXPECT_NEAR(cell.countsPerNewton(), 100000.0F / (2.0F * kG), 0.01F);
    }

    TEST(LoadCellTest, CalibrateIgnoresNonPositiveReference) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setCountsPerNewton(12345.0F);

        cell.calibrate(0.0F, 100000);
        EXPECT_FLOAT_EQ(cell.countsPerNewton(), 12345.0F);

        cell.calibrate(-5.0F, 100000);
        EXPECT_FLOAT_EQ(cell.countsPerNewton(), 12345.0F);
    }

    // ---- Offset ----

    TEST(LoadCellTest, OffsetRoundTrips) {
        MockAdc24 adc;
        LoadCell cell(adc);

        cell.setOffset(-12345);
        EXPECT_EQ(cell.offset(), -12345);
    }

    TEST(LoadCellTest, CaptureZeroAveragesSamples) {
        MockAdc24 adc;
        LoadCell cell(adc);

        adc.pushSamples({98, 100, 102, 99, 101});
        EXPECT_TRUE(cell.captureZero(5, 1000));
        EXPECT_EQ(cell.offset(), 100);
    }

    TEST(LoadCellTest, CaptureZeroPropagatesTimeoutAndPreservesOffset) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(777);

        adc.pushSamples({10, 20});  // fewer than requested
        EXPECT_FALSE(cell.captureZero(5, 1000));
        EXPECT_EQ(cell.offset(), 777);  // unchanged on failure
    }

    TEST(LoadCellTest, CaptureZeroZeroSampleCountTreatedAsOne) {
        MockAdc24 adc;
        LoadCell cell(adc);

        adc.pushSample(42);
        EXPECT_TRUE(cell.captureZero(0, 1000));
        EXPECT_EQ(cell.offset(), 42);
    }

    // ---- Unit conversion ----

    TEST(LoadCellTest, RawToUnitSubtractsOffset) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(1000);
        cell.setCountsPerNewton(50000.0F);

        // raw = 1000 + 50000 -> net = 50000 -> 1 N.
        EXPECT_FLOAT_EQ(cell.rawToUnit(51000, MU::NEWTONS), 1.0F);
        EXPECT_FLOAT_EQ(cell.rawToUnit(51000, MU::KGF), 1.0F / kG);
    }

    TEST(LoadCellTest, NetToUnitSkipsOffset) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(9999);  // ignored by netToUnit
        cell.setCountsPerNewton(50000.0F);

        EXPECT_FLOAT_EQ(cell.netToUnit(100000, MU::NEWTONS), 2.0F);
        EXPECT_FLOAT_EQ(cell.netToUnit(100000, MU::KGF), 2.0F / kG);
    }

    TEST(LoadCellTest, KgfAndNewtonsAreConsistent) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setCountsPerKgf(490332.0F);

        // Applying 1 kg should read ~1 kgf and ~g Newtons.
        const int32_t raw = 490332;
        EXPECT_NEAR(cell.rawToUnit(raw, MU::KGF), 1.0F, 1e-4F);
        EXPECT_NEAR(cell.rawToUnit(raw, MU::NEWTONS), kG, 1e-2F);
    }

    // ---- Read delegation ----

    TEST(LoadCellTest, ReadIfReadyReturnsFalseWhenAdcNotReady) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setCountsPerNewton(50000.0F);

        float value = -1.0F;
        EXPECT_FALSE(cell.readIfReady(value));
        EXPECT_FLOAT_EQ(value, -1.0F);  // untouched
    }

    TEST(LoadCellTest, ReadIfReadyConvertsThroughCalibration) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(500);
        cell.setCountsPerNewton(50000.0F);

        adc.pushSample(50500);  // net = 50000 -> 1 N
        float value = 0.0F;
        EXPECT_TRUE(cell.readIfReady(value, MU::NEWTONS));
        EXPECT_FLOAT_EQ(value, 1.0F);
    }

    TEST(LoadCellTest, ReadWithinConvertsThroughCalibration) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setOffset(0);
        cell.setCountsPerKgf(490332.0F);

        adc.pushSample(980664);  // ~2 kg
        float kg = 0.0F;
        EXPECT_TRUE(cell.readWithin(kg, 1000, 0, MU::KGF));
        EXPECT_NEAR(kg, 2.0F, 1e-3F);
    }

    TEST(LoadCellTest, ReadWithinReturnsFalseOnTimeout) {
        MockAdc24 adc;
        LoadCell cell(adc);
        cell.setCountsPerNewton(50000.0F);

        float value = 0.0F;
        EXPECT_FALSE(cell.readWithin(value, 10, 0));
    }

    // ---- Persistence round-trip ----

    TEST(LoadCellTest, OffsetAndScaleRoundTripThroughNewtons) {
        MockAdc24 adc;

        LoadCell source(adc);
        source.setOffset(12345);
        source.setCountsPerKgf(50000.0F);  // canonical = 50000 / g

        const int32_t savedOffset = source.offset();
        const float savedCountsN = source.countsPerNewton();

        LoadCell restored(adc);
        restored.setOffset(savedOffset);
        restored.setCountsPerNewton(savedCountsN);

        EXPECT_EQ(restored.offset(), source.offset());
        EXPECT_FLOAT_EQ(restored.countsPerNewton(), source.countsPerNewton());
    }

}  // namespace
