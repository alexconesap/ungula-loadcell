// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Alex Conesa
// See LICENSE file for details.

#include "tension_sensor.h"

#include <time/time_control.h>

namespace ungula {

    bool TensionSensor::readInstant(float& outTension) {
        float value = 0.0F;
        if (!loadCell_.readIfReady(value, config_.unit)) {
            return false;
        }

        lastTension_ = value;
        updateFiltered(value);
        outTension = filteredTension_;
        return true;
    }

    bool TensionSensor::readStable(float& outTension, uint32_t timeoutMs, uint32_t pollDelayMs) {
        const auto start = TimeControl::millis();

        while ((TimeControl::millis() - start) < timeoutMs) {
            float value = 0.0F;
            if (loadCell_.readIfReady(value, config_.unit)) {
                lastTension_ = value;
                updateFiltered(value);

                if (stable_) {
                    outTension = filteredTension_;
                    return true;
                }
            }

            if (pollDelayMs > 0U) {
                TimeControl::delayMs(pollDelayMs);
            } else {
                TimeControl::yield();
            }
        }

        outTension = filteredTension_;
        return false;
    }

    void TensionSensor::updateFiltered(float sample) {
        previousFilteredTension_ = filteredTension_;

        if (std::isnan(filteredTension_)) {
            filteredTension_ = sample;
        } else {
            const float alpha = 2.0F / (static_cast<float>(config_.averageSamples) + 1.0F);
            filteredTension_ = alpha * sample + (1.0F - alpha) * filteredTension_;
        }

        if (!std::isnan(previousFilteredTension_)) {
            stable_ = std::fabs(filteredTension_ - previousFilteredTension_) <=
                      config_.stabilityTolerance;
        }
    }

}  // namespace ungula
