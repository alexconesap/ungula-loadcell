// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "../i_adc24.h"

namespace ungula {

    /// @brief HX711 24-bit ADC driver — implements IAdc24.
    ///
    /// The HX711 is very simple hardware: no registers, no SPI commands, no config API.
    /// So the designers reused the clock signal:
    /// "After the read, count how many extra pulses I get -> that defines next mode."
    ///
    /// This class handles only the wire protocol (GPIO bit-bang) and sample lifecycle. Load-cell
    /// calibration, offset and unit conversion live in LoadCell on top of any IAdc24. Channel and
    /// gain selection stay here because they map to HX711-specific extra-pulse counts and have no
    /// equivalent value set on other chips in the family.
    ///
    /// Intended usage:
    /// - poll the driver at a reasonable rate (e.g. every 100-200 ms) to check for new samples
    /// - read only when a fresh sample is ready
    /// - keep threshold logic outside the driver
    class HX711 : public IAdc24 {
        public:
            /// @brief Input channel and gain configuration.
            /// HX711 uses extra pulses after the 24-bit read to select the mode for the next
            /// conversion.
            enum class InputConfig : uint8_t {
                A128,  // Channel A, 128x gain (most sensitive, default)
                A64,   // Channel A, 64x gain (less sensitive, wider range)
                B32    // Channel B, 32x gain (second input channel)
            };

            HX711() = default;
            ~HX711() override = default;

            // ---- Chip-specific initialization ----

            /// Configure GPIOs and select the operating mode.
            /// Returns false if pin setup fails.
            bool begin(uint8_t dataPin, uint8_t clockPin, InputConfig config = InputConfig::A128);

            // ---- IAdc24 interface ----

            bool isInitialized() const override;
            bool isReady() const override;
            bool readRawIfReady(int32_t& outRaw) override;
            bool readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) override;
            void powerDown() override;
            bool powerUp(uint32_t readyTimeoutMs) override;

            /// HX711 has no reset register. The equivalent is a power-cycle via the clock line:
            /// powerDown() then powerUp() — brings the chip back to A128 defaults with a fresh
            /// conversion pipeline.
            void reset() override;

            // ---- HX711-specific configuration ----

            /// Select the input channel and gain mode for the next conversion.
            /// The first sample after a change is discarded automatically.
            void setInputConfig(InputConfig config);

            /// Currently configured input channel and gain mode.
            InputConfig inputConfig() const;

        private:
            // HX711 clock pulses need a very short high/low timing margin.
            static constexpr uint32_t kClockPulseDelayUs = 1U;

            // If setInputConfig is called during a session, the 'next' read will be in the new
            // mode, but the current read may still be in the old mode. This flag tracks that state
            // so we can avoid returning wrong readings after a config change.
            bool discardNextSample_ = false;

            uint8_t dataPin_ = 0;
            uint8_t clockPin_ = 0;
            uint8_t extraPulses_ = 1U;
            InputConfig config_ = InputConfig::A128;
            bool initialized_ = false;

            static uint8_t configToPulseCount(InputConfig config);
            static uint8_t shiftInByteMsbFirst(uint8_t dataPin, uint8_t clockPin);
            static int32_t signExtend24(uint8_t b2, uint8_t b1, uint8_t b0);

            bool waitReadyUntil(uint32_t timeoutMs, uint32_t pollDelayMs) const;
            void applyConfigSelection();
            int32_t readRawNow();
    };

}  // namespace ungula
