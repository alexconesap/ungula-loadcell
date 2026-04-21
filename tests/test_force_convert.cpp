// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Alex Conesa

#include <gtest/gtest.h>

#include <loadcell/force_convert.h>

#include <cmath>

namespace {

    using namespace ungula::force;

    // ---- Force conversions ----

    TEST(ForceConvertTest, NewtonsToKgfRoundTrip) {
        EXPECT_NEAR(nToKgf(9.80665F), 1.0F, 1e-4F);
        EXPECT_NEAR(kgfToN(1.0F), 9.80665F, 1e-3F);
        EXPECT_NEAR(nToKgf(kgfToN(3.5F)), 3.5F, 1e-4F);
    }

    TEST(ForceConvertTest, NewtonsToLbfRoundTrip) {
        EXPECT_NEAR(nToLbf(4.44822F), 1.0F, 1e-3F);
        EXPECT_NEAR(lbfToN(1.0F), 4.44822F, 1e-2F);
        EXPECT_NEAR(nToLbf(lbfToN(7.0F)), 7.0F, 1e-3F);
    }

    TEST(ForceConvertTest, KgfToLbfRoundTrip) {
        EXPECT_NEAR(kgfToLbf(1.0F), 2.20462F, 1e-3F);
        EXPECT_NEAR(lbfToKgf(2.20462F), 1.0F, 1e-3F);
    }

    // ---- Stress conversions ----

    TEST(ForceConvertTest, MpaToPsiRoundTrip) {
        EXPECT_NEAR(mpaToPsi(1.0F), 145.0377F, 0.1F);
        EXPECT_NEAR(psiToMpa(145.0377F), 1.0F, 1e-3F);
        EXPECT_NEAR(mpaToPsi(psiToMpa(30000.0F)), 30000.0F, 1.0F);
    }

    TEST(ForceConvertTest, MpaToKsiRoundTrip) {
        EXPECT_NEAR(mpaToKsi(6.895F), 1.0F, 1e-2F);
        EXPECT_NEAR(ksiToMpa(1.0F), 6.895F, 0.01F);
    }

    // ---- Length conversions ----

    TEST(ForceConvertTest, MmToInchRoundTrip) {
        EXPECT_NEAR(mmToInch(25.4F), 1.0F, 1e-4F);
        EXPECT_NEAR(inchToMm(1.0F), 25.4F, 1e-4F);
    }

    // ---- Area ----

    TEST(ForceConvertTest, CircularAreaMm2) {
        // d=1mm -> r=0.5mm -> area = pi*0.25 ~ 0.7854
        EXPECT_NEAR(circularAreaMm2(1.0F), 0.7854F, 1e-3F);
        // d=0.5mm -> area ~ 0.1963
        EXPECT_NEAR(circularAreaMm2(0.5F), 0.1963F, 1e-3F);
    }

    // ---- Safe tension calculator ----

    TEST(ForceConvertTest, SafeTensionN_304SS_0p5mm) {
        // 304 SS: yield ~215 MPa, d=0.5mm, safety=0.3
        // area = pi*(0.25)^2 = 0.1963 mm^2
        // force = 215 * 0.1963 * 0.3 = 12.66 N
        float result = safeTensionN(215.0F, 0.5F, 0.3F);
        EXPECT_NEAR(result, 12.66F, 0.1F);
    }

    TEST(ForceConvertTest, SafeTensionN_Nitinol_0p3mm) {
        // Nitinol: yield ~500 MPa, d=0.3mm, safety=0.25
        // area = pi*(0.15)^2 = 0.0707 mm^2
        // force = 500 * 0.0707 * 0.25 = 8.84 N
        float result = safeTensionN(500.0F, 0.3F, 0.25F);
        EXPECT_NEAR(result, 8.84F, 0.1F);
    }

    TEST(ForceConvertTest, SafeTensionFromPsi) {
        // 304 SS: yield ~31200 psi, d=0.020" (0.508mm), safety=0.3
        // Convert: 31200 psi = 215.1 MPa, 0.020" = 0.508 mm
        // area = pi*(0.254)^2 = 0.2027 mm^2
        // force = 215.1 * 0.2027 * 0.3 = 13.08 N
        float result = safeTensionFromPsi(31200.0F, 0.020F, 0.3F);
        EXPECT_NEAR(result, 13.08F, 0.2F);
    }

    TEST(ForceConvertTest, SafeTensionDefaultSafetyFactor) {
        float withDefault = safeTensionN(215.0F, 0.5F);
        float withExplicit = safeTensionN(215.0F, 0.5F, 0.3F);
        EXPECT_FLOAT_EQ(withDefault, withExplicit);
    }

    // ---- Integration: compute target, feed to TensionSensor ----

    TEST(ForceConvertTest, EndToEndTargetComputation) {
        // 304 SS mandrel, 0.5mm diameter, yield 215 MPa, safety 30%
        float safeN = safeTensionN(215.0F, 0.5F, 0.3F);
        float safeKgf = nToKgf(safeN);
        float safeLbf = nToLbf(safeN);

        EXPECT_NEAR(safeN, 12.66F, 0.1F);
        EXPECT_NEAR(safeKgf, 1.29F, 0.02F);
        EXPECT_NEAR(safeLbf, 2.85F, 0.05F);
    }

}  // namespace
