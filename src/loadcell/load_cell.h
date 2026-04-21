// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "i_adc24.h"

namespace ungula {

    /// @brief Chip-neutral load-cell semantics on top of any IAdc24 driver.
    ///
    /// Responsibilities:
    /// - Zero offset capture and storage (raw counts)
    /// - Calibration scale (counts per Newton — internal canonical unit)
    /// - Unit conversion at the output boundary (Newtons <-> kilograms-force)
    /// - Convenience readIfReady / readWithin helpers that convert in one step
    ///
    /// The constructor takes a reference to any IAdc24 implementation (HX711, NAU7802, ADS1220,
    /// ADS1232, ...). Calibration state lives here, never on the driver — swap the chip without
    /// losing the calibration layer.
    class LoadCell {
        public:
            /// @brief Output unit for calibrated readings.
            enum class ForceUnit : uint8_t { NEWTONS, KGF, LBF };

            /// @param adc Reference to the ADC driver. Must outlive this LoadCell.
            explicit LoadCell(IAdc24& adc) : adc_(adc) {}

            // ---- Zero capture and calibration ----

            /// Capture the current unloaded ADC value as the zero reference.
            /// Averages sampleCount readings to reduce noise.
            /// Returns false if any sample times out (offset is left unchanged).
            bool captureZero(uint8_t sampleCount = 10, uint32_t timeoutPerSampleMs = 1000);

            /// Set calibration factor in raw counts per unit, with Newtons input.
            void setCountsPerNewton(float rawCountsPerUnit);

            /// Set calibration factor in raw counts per unit, with kgf input. Converts to counts
            /// per Newton internally.
            void setCountsPerKgf(float rawCountsPerUnit);

            /// Set calibration factor in raw counts per unit, with lbf input. Converts to counts
            /// per Newton internally.
            void setCountsPerLbf(float countsPerLbf);

            /// Compute the scale factor from a known load and its raw reading.
            /// @param knownValue   reference value (must be > 0)
            /// @param rawAtForce   raw ADC reading with the reference load applied
            /// @param unit         unit of knownValue (NEWTONS or KGF)
            void calibrate(float knownValue, int32_t rawAtForce,
                           ForceUnit unit = ForceUnit::NEWTONS);

            /// Canonical calibration factor (counts per Newton).
            float countsPerNewton() const {
                return countsPerNewton_;
            }

            /// Convenience accessor for counts per kgf, derived from counts per Newton.
            float countsPerKgf() const {
                return countsPerNewton_ * kGravity;
            }

            /// Convenience accessor for counts per lbf, derived from counts per Newton.
            float countsPerLbf() const {
                return countsPerNewton_ / kNewtonToLbf;
            }

            /// Store a previously captured zero offset (raw counts).
            void setOffset(int32_t rawOffset) {
                offset_ = rawOffset;
            }

            /// Current zero offset (raw counts).
            int32_t offset() const {
                return offset_;
            }

            // ---- Conversion helpers ----

            /// Convert an absolute raw ADC reading to the requested unit.
            /// Offset is subtracted internally.
            float rawToUnit(int32_t raw, ForceUnit unit = ForceUnit::NEWTONS) const;

            /// Convert a net value (caller already subtracted offset) to the requested unit.
            float netToUnit(int32_t netValue, ForceUnit unit = ForceUnit::NEWTONS) const;

            // ---- Calibrated reads (thin wrappers around the ADC) ----

            /// Non-blocking read in the requested unit. Returns false if no sample is ready.
            bool readIfReady(float& outValue, ForceUnit unit = ForceUnit::NEWTONS);

            /// Blocking read in the requested unit. Returns false if no sample became ready.
            bool readWithin(float& outValue, uint32_t timeoutMs, uint32_t pollDelayMs = 0,
                            ForceUnit unit = ForceUnit::NEWTONS);

            /// Access the underlying ADC driver for raw reads, power management, chip-specific
            /// config.
            IAdc24& adc() {
                return adc_;
            }
            const IAdc24& adc() const {
                return adc_;
            }

        private:
            // Standard gravity — converts kgf to Newtons.
            static constexpr float kGravity = 9.80665F;
            static constexpr float kNewtonToLbf = 0.2248089431F;

            IAdc24& adc_;
            int32_t offset_ = 0;
            float countsPerNewton_ = 0.0F;
    };

}  // namespace ungula
