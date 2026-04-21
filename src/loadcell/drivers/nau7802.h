// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "../i_adc24.h"

namespace ungula {
    namespace i2c {
        class I2cMaster;
    }

    /// @brief NAU7802 24-bit ADC driver — implements IAdc24.
    ///
    /// The NAU7802 is a Nuvoton precision 24-bit ADC with integrated LDO and PGA, purpose-built
    /// for bridge sensors. Communicates over I2C at a fixed 7-bit address (0x2A).
    ///
    /// Chip-specific features (not on IAdc24 — they differ across chips):
    ///   - Programmable gain: 1, 2, 4, 8, 16, 32, 64, 128
    ///   - Channel selection: CH1 or CH2
    ///   - Sample rate: 10, 20, 40, 80, 320 SPS
    ///   - Internal LDO voltage selection (2.4V–4.5V)
    ///   - Internal offset/gain calibration commands
    ///
    /// Usage:
    ///   ungula::i2c::I2cMaster bus(0);
    ///   bus.begin(21, 22, 400000);
    ///   ungula::NAU7802 adc;
    ///   adc.begin(bus);
    ///   adc.setGain(NAU7802::Gain::X128);
    ///   adc.setSampleRate(NAU7802::SampleRate::SPS_80);
    ///   ungula::LoadCell cell(adc);
    class NAU7802 : public IAdc24 {
        public:
            static constexpr uint8_t I2C_ADDR = 0x2A;

            enum class Gain : uint8_t {
                X1 = 0,
                X2 = 1,
                X4 = 2,
                X8 = 3,
                X16 = 4,
                X32 = 5,
                X64 = 6,
                X128 = 7
            };

            enum class SampleRate : uint8_t {
                SPS_10 = 0,
                SPS_20 = 1,
                SPS_40 = 2,
                SPS_80 = 3,
                SPS_320 = 7
            };

            enum class LdoVoltage : uint8_t {
                V_4_5 = 0,
                V_4_2 = 1,
                V_3_9 = 2,
                V_3_6 = 3,
                V_3_3 = 4,
                V_3_0 = 5,
                V_2_7 = 6,
                V_2_4 = 7
            };

            enum class Channel : uint8_t { CH1 = 0, CH2 = 1 };

            NAU7802() = default;
            ~NAU7802() override = default;

            /// @brief Initialize the NAU7802 via I2C.
            /// Performs the power-up sequence: RR reset, PUA, PUR check, enable LDO.
            /// @param bus Reference to the I2C master. Must outlive this driver.
            /// @return true if the chip responded and powered up correctly.
            bool begin(i2c::I2cMaster& bus);

            // ---- IAdc24 interface ----

            bool isInitialized() const override;
            bool isReady() const override;
            bool readRawIfReady(int32_t& outRaw) override;
            bool readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) override;
            void powerDown() override;
            bool powerUp(uint32_t readyTimeoutMs) override;
            void reset() override;

            // ---- NAU7802-specific configuration ----

            void setGain(Gain gain);
            void setSampleRate(SampleRate rate);
            void setLdoVoltage(LdoVoltage voltage);
            void setChannel(Channel channel);

            /// Trigger internal offset calibration. Blocks until complete (up to 1s).
            /// Call after changing gain, channel or sample rate for best accuracy.
            bool calibrateAfe();

        private:
            // NAU7802 register addresses.
            static constexpr uint8_t REG_PU_CTRL = 0x00;
            static constexpr uint8_t REG_CTRL1 = 0x01;
            static constexpr uint8_t REG_CTRL2 = 0x02;
            static constexpr uint8_t REG_ADC_B2 = 0x12;
            static constexpr uint8_t REG_ADC_B1 = 0x13;
            static constexpr uint8_t REG_ADC_B0 = 0x14;
            static constexpr uint8_t REG_OTP_B1 = 0x15;
            static constexpr uint8_t REG_POWER_CTRL = 0x1C;

            // PU_CTRL bits.
            static constexpr uint8_t PU_CTRL_RR = 0x01;     // Register reset
            static constexpr uint8_t PU_CTRL_PUD = 0x02;    // Power-up digital
            static constexpr uint8_t PU_CTRL_PUA = 0x04;    // Power-up analog
            static constexpr uint8_t PU_CTRL_PUR = 0x08;    // Power-up ready (read-only)
            static constexpr uint8_t PU_CTRL_CS = 0x10;     // Conversion start (cycle start)
            static constexpr uint8_t PU_CTRL_CR = 0x20;     // Conversion ready (read-only, DRDY)
            static constexpr uint8_t PU_CTRL_AVDDS = 0x80;  // AVDD source select (1 = internal LDO)

            // CTRL2 bits.
            static constexpr uint8_t CTRL2_CALMOD_OFFSET = 0x00;
            static constexpr uint8_t CTRL2_CALS = 0x04;     // Calibration start
            static constexpr uint8_t CTRL2_CAL_ERR = 0x08;  // Calibration error (read-only)

            i2c::I2cMaster* bus_ = nullptr;
            bool initialized_ = false;

            bool writeReg(uint8_t reg, uint8_t value);
            bool readReg(uint8_t reg, uint8_t& value);
            bool setBits(uint8_t reg, uint8_t mask);
            bool clearBits(uint8_t reg, uint8_t mask);
            bool readBit(uint8_t reg, uint8_t bit);
    };

}  // namespace ungula
