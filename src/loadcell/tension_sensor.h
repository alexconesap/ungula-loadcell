// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>
#include <cmath>

#include "load_cell.h"

namespace ungula {

    class TensionSensor {
        public:
            using ForceUnit = LoadCell::ForceUnit;

            struct Config {
                    uint8_t averageSamples = 8;
                    float stabilityTolerance = 0.05F;
                    float targetTolerance = 0.05F;
                    ForceUnit unit = ForceUnit::NEWTONS;
            };

            explicit TensionSensor(LoadCell& loadCell) : loadCell_(loadCell) {}

            void configure(const Config& config) {
                config_ = config;
            }

            bool readInstant(float& outTension);
            bool readStable(float& outTension, uint32_t timeoutMs, uint32_t pollDelayMs = 0);

            bool isStable() const {
                return stable_;
            }
            float lastTension() const {
                return lastTension_;
            }
            float filteredTension() const {
                return filteredTension_;
            }

            float lastVariation() const {
                if (std::isnan(filteredTension_) || std::isnan(previousFilteredTension_)) {
                    return NAN;
                }
                return filteredTension_ - previousFilteredTension_;
            }

            void setTarget(float target) {
                target_ = target;
            }
            float target() const {
                return target_;
            }

            float errorToTarget() const {
                if (std::isnan(filteredTension_) || std::isnan(target_)) {
                    return NAN;
                }
                return filteredTension_ - target_;
            }

            bool isAtTarget() const {
                if (std::isnan(filteredTension_) || std::isnan(target_)) {
                    return false;
                }
                return std::fabs(filteredTension_ - target_) <= config_.targetTolerance;
            }

            bool isAboveTarget() const {
                if (std::isnan(filteredTension_) || std::isnan(target_)) {
                    return false;
                }
                return filteredTension_ > (target_ + config_.targetTolerance);
            }

            bool isBelowTarget() const {
                if (std::isnan(filteredTension_) || std::isnan(target_)) {
                    return false;
                }
                return filteredTension_ < (target_ - config_.targetTolerance);
            }

        private:
            void updateFiltered(float sample);

            LoadCell& loadCell_;
            Config config_{};

            float lastTension_ = NAN;
            float filteredTension_ = NAN;
            float previousFilteredTension_ = NAN;
            float target_ = NAN;
            bool stable_ = false;
    };

}  // namespace ungula
