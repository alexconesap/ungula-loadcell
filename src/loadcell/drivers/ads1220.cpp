// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "ads1220.h"

#include <hal/gpio/gpio_access.h>
#include <hal/spi/spi_master.h>
#include <time/time_control.h>

namespace ungula {

    // ---- SPI command helpers ----

    void ADS1220::sendCommand(uint8_t cmd) {
        spi_->write(&cmd, 1);
    }

    bool ADS1220::writeRegister(uint8_t regAddr, uint8_t value) {
        // WREG command: 0x40 | (reg << 2) | (count-1). For 1 register: 0x40 | (reg << 2).
        uint8_t buf[2] = {static_cast<uint8_t>(CMD_WREG | ((regAddr & 0x03U) << 2U)), value};
        return spi_->write(buf, 2);
    }

    bool ADS1220::readRegister(uint8_t regAddr, uint8_t& value) {
        // RREG command: 0x20 | (reg << 2) | (count-1). For 1 register: 0x20 | (reg << 2).
        uint8_t cmd = static_cast<uint8_t>(CMD_RREG | ((regAddr & 0x03U) << 2U));
        return spi_->writeRead(&cmd, 1, &value, 1);
    }

    void ADS1220::writeAllRegisters() {
        // Write all 4 config registers in one go: WREG starting at reg 0, count 4.
        uint8_t buf[5] = {static_cast<uint8_t>(CMD_WREG | 0x03U),  // reg 0, 4 bytes (n-1 = 3)
                          reg_[0], reg_[1], reg_[2], reg_[3]};
        spi_->write(buf, 5);
    }

    // ---- Lifecycle ----

    bool ADS1220::begin(spi::SpiMaster& spi, uint8_t drdyPin) {
        spi_ = &spi;
        drdyPin_ = drdyPin;
        initialized_ = false;

        gpio::configInput(drdyPin_);

        // Send RESET command, wait for settling.
        sendCommand(CMD_RESET);
        TimeControl::delayMs(1);

        // Default register config: AIN0-AIN1 differential, gain 128, 20 SPS, continuous, internal
        // ref.
        reg_[0] = (static_cast<uint8_t>(Mux::AIN0_AIN1) << 4U) |
                  (static_cast<uint8_t>(Gain::X128) << 1U);  // PGA enabled (bit 0 = 0)
        reg_[1] = (static_cast<uint8_t>(DataRate::SPS_20) << 5U) | (0U << 4U) |  // Normal mode
                  (static_cast<uint8_t>(ConversionMode::CONTINUOUS) << 3U);      // Continuous
        reg_[2] = (static_cast<uint8_t>(VoltageRef::INTERNAL_2V048) << 6U);
        reg_[3] = 0x00;  // No IDAC, DRDY on dedicated pin

        writeAllRegisters();

        // Start conversions.
        sendCommand(CMD_START);

        initialized_ = true;
        return true;
    }

    bool ADS1220::isInitialized() const {
        return initialized_;
    }

    bool ADS1220::isReady() const {
        // DRDY pin goes low when a new conversion result is available.
        return initialized_ && gpio::isLow(drdyPin_);
    }

    bool ADS1220::readRawIfReady(int32_t& outRaw) {
        if (!isReady()) {
            return false;
        }
        outRaw = readConversionData();
        return true;
    }

    bool ADS1220::readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) {
        if (!waitReadyUntil(timeoutMs, pollDelayMs)) {
            return false;
        }
        outRaw = readConversionData();
        return true;
    }

    void ADS1220::powerDown() {
        if (spi_ != nullptr) {
            sendCommand(CMD_POWERDOWN);
        }
    }

    bool ADS1220::powerUp(uint32_t readyTimeoutMs) {
        if (spi_ == nullptr) {
            return false;
        }
        // Wake by sending START/SYNC. Chip exits power-down and begins conversion.
        sendCommand(CMD_START);
        return waitReadyUntil(readyTimeoutMs, 5U);
    }

    void ADS1220::reset() {
        if (spi_ == nullptr) {
            return;
        }
        sendCommand(CMD_RESET);
        TimeControl::delayMs(1);
        initialized_ = false;
    }

    // ---- Configuration ----

    void ADS1220::setGain(Gain gain) {
        reg_[0] = (reg_[0] & 0xF1U) | (static_cast<uint8_t>(gain) << 1U);
        if (initialized_) {
            writeRegister(0, reg_[0]);
        }
    }

    void ADS1220::setMux(Mux mux) {
        reg_[0] = (reg_[0] & 0x0FU) | (static_cast<uint8_t>(mux) << 4U);
        if (initialized_) {
            writeRegister(0, reg_[0]);
        }
    }

    void ADS1220::setDataRate(DataRate rate) {
        reg_[1] = (reg_[1] & 0x1FU) | (static_cast<uint8_t>(rate) << 5U);
        if (initialized_) {
            writeRegister(1, reg_[1]);
        }
    }

    void ADS1220::setVoltageRef(VoltageRef ref) {
        reg_[2] = (reg_[2] & 0x3FU) | (static_cast<uint8_t>(ref) << 6U);
        if (initialized_) {
            writeRegister(2, reg_[2]);
        }
    }

    void ADS1220::setConversionMode(ConversionMode mode) {
        if (mode == ConversionMode::CONTINUOUS) {
            reg_[1] |= 0x08U;
        } else {
            reg_[1] &= 0xF7U;
        }
        if (initialized_) {
            writeRegister(1, reg_[1]);
        }
    }

    void ADS1220::setTemperatureSensor(bool enabled) {
        if (enabled) {
            reg_[1] |= 0x04U;
        } else {
            reg_[1] &= 0xFBU;
        }
        if (initialized_) {
            writeRegister(1, reg_[1]);
        }
    }

    void ADS1220::setPgaBypass(bool bypass) {
        if (bypass) {
            reg_[0] |= 0x01U;
        } else {
            reg_[0] &= 0xFEU;
        }
        if (initialized_) {
            writeRegister(0, reg_[0]);
        }
    }

    void ADS1220::startConversion() {
        if (spi_ != nullptr) {
            sendCommand(CMD_START);
        }
    }

    // ---- Internal helpers ----

    bool ADS1220::waitReadyUntil(uint32_t timeoutMs, uint32_t pollDelayMs) const {
        const TimeControl::tick_ms_t start = TimeControl::millis();
        while ((TimeControl::millis() - start) < timeoutMs) {
            if (isReady()) {
                return true;
            }
            TimeControl::delayMs(pollDelayMs);
        }
        return false;
    }

    int32_t ADS1220::readConversionData() {
        // Send RDATA (0x10) then clock out 3 bytes of data.
        uint8_t cmd = CMD_RDATA;
        uint8_t data[3] = {};
        spi_->writeRead(&cmd, 1, data, 3);

        // MSB first, 24-bit two's complement → sign-extend to int32_t.
        const uint32_t raw24 = (static_cast<uint32_t>(data[0]) << 16U) |
                               (static_cast<uint32_t>(data[1]) << 8U) |
                               static_cast<uint32_t>(data[2]);
        if ((data[0] & 0x80U) != 0U) {
            return static_cast<int32_t>(raw24 | 0xFF000000UL);
        }
        return static_cast<int32_t>(raw24);
    }

}  // namespace ungula
