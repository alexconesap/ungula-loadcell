// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "../i_adc24.h"

namespace ungula {

    /// @brief ADS1232 24-bit ADC driver — implements IAdc24.
    ///
    /// The ADS1232 is a precision 24-bit ADC from TI designed for bridge sensors. Like the
    /// HX711, it uses GPIO bit-bang for data readout (no SPI/I2C registers). Unlike the HX711:
    ///
    ///   - Gain selection via hardware pins GAIN0/GAIN1 (1, 2, 64, 128), not clock pulses.
    ///   - Channel selection via A0 pin (ch1 or ch2).
    ///   - Sample rate via SPEED pin (10 SPS or 80 SPS).
    ///   - Dedicated PDWN pin for power-down (active low, separate from the clock line).
    ///   - TEMP pin to enable internal temperature sensor mode.
    ///   - Read is 25 SCLK pulses: 24 data + 1 ack (no extra mode-selection pulses).
    ///   - 26 pulses triggers self-offset calibration (~800ms).
    ///   - DRDY = DOUT going low (same as HX711).
    ///
    /// All control pins (PDWN, SPEED, A0, GAIN0, GAIN1, TEMP) are optional — pass
    /// GPIO_NONE (0xFF) to leave them unmanaged (tie them externally in that case).
    class ADS1232 : public IAdc24 {
        public:
            enum class SampleRate : uint8_t {
                SPS_10 = 0,  // SPEED pin low
                SPS_80 = 1   // SPEED pin high
            };

            /// @brief Gain selection via GAIN0/GAIN1 pins.
            enum class Gain : uint8_t {
                X1 = 0,   // GAIN1=0, GAIN0=0
                X2 = 1,   // GAIN1=0, GAIN0=1
                X64 = 2,  // GAIN1=1, GAIN0=0
                X128 = 3  // GAIN1=1, GAIN0=1
            };

            enum class Channel : uint8_t { CH1 = 0, CH2 = 1 };

            static constexpr uint8_t GPIO_NONE = 0xFF;

            ADS1232() = default;
            ~ADS1232() override = default;

            /// @brief Configure GPIO pins and initialize the chip.
            /// @param doutPin   Data/DRDY output pin.
            /// @param sclkPin   Serial clock pin.
            /// @param pdwnPin   Power-down pin (active low). GPIO_NONE if tied externally.
            /// @param speedPin  Sample rate pin. GPIO_NONE if tied externally.
            /// @param a0Pin     Channel select pin. GPIO_NONE if tied externally.
            /// @param gain0Pin  Gain bit 0 pin. GPIO_NONE if tied externally.
            /// @param gain1Pin  Gain bit 1 pin. GPIO_NONE if tied externally.
            /// @param tempPin   Temperature mode pin. GPIO_NONE if tied externally.
            /// @return true on success.
            bool begin(uint8_t doutPin, uint8_t sclkPin, uint8_t pdwnPin = GPIO_NONE,
                       uint8_t speedPin = GPIO_NONE, uint8_t a0Pin = GPIO_NONE,
                       uint8_t gain0Pin = GPIO_NONE, uint8_t gain1Pin = GPIO_NONE,
                       uint8_t tempPin = GPIO_NONE);

            // ---- IAdc24 interface ----

            bool isInitialized() const override;
            bool isReady() const override;
            bool readRawIfReady(int32_t& outRaw) override;
            bool readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) override;
            void powerDown() override;
            bool powerUp(uint32_t readyTimeoutMs) override;
            void reset() override;

            // ---- ADS1232-specific configuration ----

            void setSampleRate(SampleRate rate);
            void setGain(Gain gain);
            void setChannel(Channel channel);
            void setTemperatureMode(bool enabled);

            /// Trigger internal self-offset calibration (26th SCLK pulse).
            /// Blocks until DOUT goes low again (~800ms at 10 SPS).
            bool selfCalibrate(uint32_t timeoutMs = 2000);

        private:
            static constexpr uint32_t kClockPulseDelayUs = 1U;

            uint8_t doutPin_ = 0;
            uint8_t sclkPin_ = 0;
            uint8_t pdwnPin_ = GPIO_NONE;
            uint8_t speedPin_ = GPIO_NONE;
            uint8_t a0Pin_ = GPIO_NONE;
            uint8_t gain0Pin_ = GPIO_NONE;
            uint8_t gain1Pin_ = GPIO_NONE;
            uint8_t tempPin_ = GPIO_NONE;
            bool initialized_ = false;

            static uint8_t shiftInByteMsbFirst(uint8_t doutPin, uint8_t sclkPin);
            static int32_t signExtend24(uint8_t byte2, uint8_t byte1, uint8_t byte0);
            bool waitReadyUntil(uint32_t timeoutMs, uint32_t pollDelayMs) const;
            int32_t readRawNow();
            void clockPulse();
    };

}  // namespace ungula
