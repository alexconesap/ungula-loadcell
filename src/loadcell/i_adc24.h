// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

namespace ungula {

    /// @brief Chip-neutral interface for 24-bit signed delta-sigma ADCs commonly used with load
    /// cells, thermocouples and pressure transducers.
    ///
    /// Covers the minimum each chip in this family actually exposes at runtime: ready polling,
    /// raw sample retrieval, power management and reset. Per-chip quirks that do not generalise
    /// well — gain values, channel multiplexing, data-rate selection, bus wiring — stay on the
    /// concrete driver class.
    ///
    /// Concrete drivers (HX711, NAU7802, ADS1220, ADS1232, ...) configure their own pins in a
    /// chip-specific begin() and then honour this interface for day-to-day use. Anything higher
    /// level (zero capture, calibration, unit conversion) is implemented once in LoadCell on top
    /// of this interface — the high-level class is wire-agnostic.
    class IAdc24 {
        public:
            virtual ~IAdc24() = default;

            /// Returns true if the driver configured its wiring successfully.
            virtual bool isInitialized() const = 0;

            /// Returns true when a fresh 24-bit conversion is ready to be read.
            virtual bool isReady() const = 0;

            /// Read one raw sample only if a fresh value is already available.
            /// Returns false if no sample is ready yet (non-blocking).
            virtual bool readRawIfReady(int32_t& outRaw) = 0;

            /// Wait up to timeoutMs for a sample to become ready, then read it.
            /// pollDelayMs is a cooperative yield between ready checks.
            virtual bool readRawWithin(int32_t& outRaw, uint32_t timeoutMs,
                                       uint32_t pollDelayMs) = 0;

            /// Enter low-power / sleep mode. Current consumption drops to chip-specific minimum.
            virtual void powerDown() = 0;

            /// Wake from low-power mode and block until the first post-wake sample is ready.
            /// Returns false if no sample became ready within readyTimeoutMs.
            virtual bool powerUp(uint32_t readyTimeoutMs) = 0;

            /// Reset internal chip state. Each chip interprets this in its own way — soft reset
            /// register, power cycle via clock, or reset pin toggle. The driver should return to a
            /// known-good default configuration so the caller can issue begin()-equivalent writes
            /// afterwards if needed.
            virtual void reset() = 0;
    };

}  // namespace ungula
