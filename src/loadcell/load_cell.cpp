// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "load_cell.h"

#include <time/time_control.h>
#include <cmath>

namespace ungula {

    bool LoadCell::captureZero(uint8_t sampleCount, uint32_t timeoutPerSampleMs) {
        if (sampleCount == 0U) {
            sampleCount = 1U;
        }

        int64_t sum = 0;

        for (uint8_t i = 0; i < sampleCount; ++i) {
            int32_t raw = 0;
            if (!adc_.readRawWithin(raw, timeoutPerSampleMs, 0U)) {
                return false;
            }
            sum += raw;

            TimeControl::yield();
        }

        offset_ = static_cast<int32_t>(sum / sampleCount);
        return true;
    }

    void LoadCell::setCountsPerNewton(float rawCounts) {
        countsPerNewton_ = rawCounts;
    }

    void LoadCell::setCountsPerKgf(float rawCounts) {
        countsPerNewton_ = rawCounts / kGravity;
    }

    void LoadCell::setCountsPerLbf(float countsPerLbf) {
        countsPerNewton_ = countsPerLbf * kNewtonToLbf;
    }

    void LoadCell::calibrate(float knownValue, int32_t rawAtForce, ForceUnit unit) {
        if (knownValue <= 0.0F) {
            return;
        }

        float forceN = knownValue;
        if (unit == ForceUnit::KGF) {
            forceN = knownValue * kGravity;
        } else if (unit == ForceUnit::LBF) {
            forceN = knownValue / kNewtonToLbf;
        }

        const int32_t net = rawAtForce - offset_;
        countsPerNewton_ = static_cast<float>(net) / forceN;
    }

    float LoadCell::rawToUnit(int32_t raw, ForceUnit unit) const {
        return netToUnit(raw - offset_, unit);
    }

    float LoadCell::netToUnit(int32_t netValue, ForceUnit unit) const {
        // Single conversion boundary: raw counts -> user unit.
        if (countsPerNewton_ == 0.0F) {
            return NAN;
        }

        const float newtons = static_cast<float>(netValue) / countsPerNewton_;

        switch (unit) {
            case ForceUnit::NEWTONS:
                return newtons;
            case ForceUnit::KGF:
                return newtons / kGravity;
            case ForceUnit::LBF:
                return newtons * kNewtonToLbf;
        }
        return newtons;
    }

    bool LoadCell::readIfReady(float& outValue, ForceUnit unit) {
        int32_t raw = 0;
        if (!adc_.readRawIfReady(raw)) {
            return false;
        }
        outValue = rawToUnit(raw, unit);
        return true;
    }

    bool LoadCell::readWithin(float& outValue, uint32_t timeoutMs, uint32_t pollDelayMs,
                              ForceUnit unit) {
        int32_t raw = 0;
        if (!adc_.readRawWithin(raw, timeoutMs, pollDelayMs)) {
            return false;
        }
        outValue = rawToUnit(raw, unit);
        return true;
    }

}  // namespace ungula
