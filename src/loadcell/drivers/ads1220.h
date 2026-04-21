// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once

#include <stdint.h>

#include "../i_adc24.h"

namespace ungula {
    namespace spi {
        class SpiMaster;
    }

    /// @brief ADS1220 24-bit ADC driver — implements IAdc24.
    ///
    /// The ADS1220 is a TI precision 24-bit delta-sigma ADC with integrated PGA, voltage
    /// reference, and IDAC. Communicates over SPI (mode 1, CPOL=0, CPHA=1, MSB first).
    ///
    /// Chip-specific features (not on IAdc24):
    ///   - Programmable gain: 1, 2, 4, 8, 16, 32, 64, 128
    ///   - 4-channel input MUX (differential and single-ended)
    ///   - Data rate: 20–1000 SPS (normal mode)
    ///   - Conversion mode: single-shot or continuous
    ///   - Internal 2.048V reference or external reference pins
    ///   - Excitation current sources (IDAC) for RTD applications
    ///   - Internal temperature sensor
    ///
    /// DRDY is signalled via a dedicated GPIO pin (active low) separate from the SPI lines.
    ///
    /// Usage:
    ///   ungula::spi::SpiMaster spi;
    ///   spi.begin(sclk, miso, mosi, cs, 1000000, 1);
    ///   ungula::ADS1220 adc;
    ///   adc.begin(spi, drdyPin);
    ///   adc.setGain(ADS1220::Gain::X128);
    ///   adc.setDataRate(ADS1220::DataRate::SPS_20);
    ///   ungula::LoadCell cell(adc);
    class ADS1220 : public IAdc24 {
        public:
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

            /// Input MUX configuration. Common load-cell setups use AIN0_AIN1 (differential).
            enum class Mux : uint8_t {
                AIN0_AIN1 = 0x00,  // Differential AIN0-AIN1
                AIN0_AIN2 = 0x01,
                AIN0_AIN3 = 0x02,
                AIN1_AIN2 = 0x03,
                AIN1_AIN3 = 0x04,
                AIN2_AIN3 = 0x05,
                AIN1_AIN0 = 0x06,
                AIN3_AIN2 = 0x07,
                AIN0_AVSS = 0x08,  // Single-ended AIN0 vs AVSS
                AIN1_AVSS = 0x09,
                AIN2_AVSS = 0x0A,
                AIN3_AVSS = 0x0B,
            };

            enum class DataRate : uint8_t {
                SPS_20 = 0,
                SPS_45 = 1,
                SPS_90 = 2,
                SPS_175 = 3,
                SPS_330 = 4,
                SPS_600 = 5,
                SPS_1000 = 6
            };

            enum class VoltageRef : uint8_t {
                INTERNAL_2V048 = 0,  // Internal 2.048V reference
                REFP0_REFN0 = 1,     // External REFP0/REFN0 pins
                AIN0_AIN3 = 2,       // AIN0/AIN3 as reference (ratiometric)
                AVDD_AVSS = 3        // Analog supply
            };

            enum class ConversionMode : uint8_t { SINGLE_SHOT = 0, CONTINUOUS = 1 };

            ADS1220() = default;
            ~ADS1220() override = default;

            /// @brief Initialize via SPI. Sends RESET, configures default registers.
            /// @param spi Reference to the SPI master. Must outlive this driver.
            /// @param drdyPin GPIO pin connected to the DRDY output (active low).
            /// @return true if reset and configuration succeeded.
            bool begin(spi::SpiMaster& spi, uint8_t drdyPin);

            // ---- IAdc24 interface ----

            bool isInitialized() const override;
            bool isReady() const override;
            bool readRawIfReady(int32_t& outRaw) override;
            bool readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) override;
            void powerDown() override;
            bool powerUp(uint32_t readyTimeoutMs) override;
            void reset() override;

            // ---- ADS1220-specific configuration ----

            void setGain(Gain gain);
            void setMux(Mux mux);
            void setDataRate(DataRate rate);
            void setVoltageRef(VoltageRef ref);
            void setConversionMode(ConversionMode mode);

            /// Enable or disable the internal temperature sensor (overrides normal input).
            void setTemperatureSensor(bool enabled);

            /// Bypass the PGA (force gain to 1 regardless of gain register).
            void setPgaBypass(bool bypass);

            /// Start a conversion (required in single-shot mode after each read).
            void startConversion();

        private:
            // SPI commands.
            static constexpr uint8_t CMD_RESET = 0x06;
            static constexpr uint8_t CMD_START = 0x08;
            static constexpr uint8_t CMD_POWERDOWN = 0x02;
            static constexpr uint8_t CMD_RDATA = 0x10;
            static constexpr uint8_t CMD_RREG = 0x20;
            static constexpr uint8_t CMD_WREG = 0x40;

            spi::SpiMaster* spi_ = nullptr;
            uint8_t drdyPin_ = 0;
            bool initialized_ = false;

            // Shadow copies of the 4 config registers.
            uint8_t reg_[4] = {};

            void sendCommand(uint8_t cmd);
            bool writeRegister(uint8_t regAddr, uint8_t value);
            bool readRegister(uint8_t regAddr, uint8_t& value);
            void writeAllRegisters();
            bool waitReadyUntil(uint32_t timeoutMs, uint32_t pollDelayMs) const;
            int32_t readConversionData();
    };

}  // namespace ungula
