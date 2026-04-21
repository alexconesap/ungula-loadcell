// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <cmath>

namespace ungula {
    namespace force {

        // ---- Force unit conversions ----

        inline float nToKgf(float n) {
            return n / 9.80665F;
        }
        inline float kgfToN(float kgf) {
            return kgf * 9.80665F;
        }

        inline float nToLbf(float n) {
            return n * 0.2248089431F;
        }
        inline float lbfToN(float lbf) {
            return lbf / 0.2248089431F;
        }

        inline float kgfToLbf(float kgf) {
            return kgf * 2.20462262F;
        }
        inline float lbfToKgf(float lbf) {
            return lbf / 2.20462262F;
        }

        // ---- Stress unit conversions ----

        inline float mpaToKsi(float mpa) {
            return mpa * 0.1450377F;
        }
        inline float ksiToMpa(float ksi) {
            return ksi / 0.1450377F;
        }

        inline float mpaToPsi(float mpa) {
            return mpa * 145.0377F;
        }
        inline float psiToMpa(float psi) {
            return psi / 145.0377F;
        }

        // ---- Length unit conversions ----

        inline float mmToInch(float mm) {
            return mm / 25.4F;
        }
        inline float inchToMm(float inch) {
            return inch * 25.4F;
        }

        // ---- Cross-section area from diameter ----

        inline float circularAreaMm2(float diameterMm) {
            const float r = diameterMm * 0.5F;
            return 3.14159265F * r * r;
        }

        inline float circularAreaIn2(float diameterInch) {
            const float r = diameterInch * 0.5F;
            return 3.14159265F * r * r;
        }

        // ---- Safe working tension from material properties ----

        inline float safeTensionN(float yieldMpa, float diameterMm, float safetyFactor = 0.3F) {
            return yieldMpa * circularAreaMm2(diameterMm) * safetyFactor;
        }

        inline float safeTensionFromPsi(float yieldPsi, float diameterInch,
                                        float safetyFactor = 0.3F) {
            const float yieldMpa = psiToMpa(yieldPsi);
            const float diameterMm = inchToMm(diameterInch);
            return yieldMpa * circularAreaMm2(diameterMm) * safetyFactor;
        }

    }  // namespace force
}  // namespace ungula
