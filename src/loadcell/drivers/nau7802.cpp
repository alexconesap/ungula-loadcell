// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "nau7802.h"

#include <hal/i2c/i2c_master.h>
#include <time/time_control.h>

namespace ungula {

    // ---- I2C register helpers ----

    bool NAU7802::writeReg(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        return bus_->write(I2C_ADDR, buf, 2);
    }

    bool NAU7802::readReg(uint8_t reg, uint8_t& value) {
        return bus_->writeRead(I2C_ADDR, &reg, 1, &value, 1);
    }

    bool NAU7802::setBits(uint8_t reg, uint8_t mask) {
        uint8_t val = 0;
        if (!readReg(reg, val)) {
            return false;
        }
        return writeReg(reg, val | mask);
    }

    bool NAU7802::clearBits(uint8_t reg, uint8_t mask) {
        uint8_t val = 0;
        if (!readReg(reg, val)) {
            return false;
        }
        return writeReg(reg, val & static_cast<uint8_t>(~mask));
    }

    bool NAU7802::readBit(uint8_t reg, uint8_t bit) {
        uint8_t val = 0;
        if (!readReg(reg, val)) {
            return false;
        }
        return (val & (1U << bit)) != 0;
    }

    // ---- Lifecycle ----

    bool NAU7802::begin(i2c::I2cMaster& bus) {
        bus_ = &bus;
        initialized_ = false;

        // Reset all registers.
        if (!setBits(REG_PU_CTRL, PU_CTRL_RR)) {
            return false;
        }
        if (!clearBits(REG_PU_CTRL, PU_CTRL_RR)) {
            return false;
        }

        // Power up digital.
        if (!setBits(REG_PU_CTRL, PU_CTRL_PUD)) {
            return false;
        }

        // Power up analog.
        if (!setBits(REG_PU_CTRL, PU_CTRL_PUA)) {
            return false;
        }

        // Wait for PUR (power-up ready) — max ~200ms.
        const TimeControl::ms_tick_t start = TimeControl::millis();
        while ((TimeControl::millis() - start) < 500U) {
            uint8_t puCtrl = 0;
            if (readReg(REG_PU_CTRL, puCtrl) && (puCtrl & PU_CTRL_PUR) != 0) {
                break;
            }
            TimeControl::delayMs(1);
        }

        // Check if PUR is set.
        uint8_t puCtrl = 0;
        if (!readReg(REG_PU_CTRL, puCtrl) || (puCtrl & PU_CTRL_PUR) == 0) {
            return false;
        }

        // Enable internal LDO (default 3.0V).
        if (!setBits(REG_PU_CTRL, PU_CTRL_AVDDS)) {
            return false;
        }
        setLdoVoltage(LdoVoltage::V_3_3);

        // Default config: gain 128, 80 SPS, channel 1.
        setGain(Gain::X128);
        setSampleRate(SampleRate::SPS_80);

        // Enable the 330pF decoupling cap on Ch2 (OTP register trick — well-known NAU7802
        // startup fix from Nuvoton application notes).
        uint8_t otp = 0;
        if (readReg(REG_OTP_B1, otp)) {
            writeReg(REG_OTP_B1, otp | 0x30U);
        }

        initialized_ = true;
        return true;
    }

    bool NAU7802::isInitialized() const {
        return initialized_;
    }

    bool NAU7802::isReady() const {
        if (!initialized_ || bus_ == nullptr) {
            return false;
        }
        // CR bit (bit 5) in PU_CTRL indicates conversion ready.
        uint8_t puCtrl = 0;
        if (!const_cast<NAU7802*>(this)->readReg(REG_PU_CTRL, puCtrl)) {
            return false;
        }
        return (puCtrl & PU_CTRL_CR) != 0;
    }

    bool NAU7802::readRawIfReady(int32_t& outRaw) {
        if (!isReady()) {
            return false;
        }

        // Read 3 bytes from ADC result registers (0x12, 0x13, 0x14).
        uint8_t byte2 = 0;
        uint8_t byte1 = 0;
        uint8_t byte0 = 0;
        if (!readReg(REG_ADC_B2, byte2) || !readReg(REG_ADC_B1, byte1) ||
            !readReg(REG_ADC_B0, byte0)) {
            return false;
        }

        const uint32_t raw24 = (static_cast<uint32_t>(byte2) << 16U) |
                               (static_cast<uint32_t>(byte1) << 8U) | static_cast<uint32_t>(byte0);
        if ((byte2 & 0x80U) != 0U) {
            outRaw = static_cast<int32_t>(raw24 | 0xFF000000UL);
        } else {
            outRaw = static_cast<int32_t>(raw24);
        }
        return true;
    }

    bool NAU7802::readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) {
        const TimeControl::ms_tick_t start = TimeControl::millis();
        while ((TimeControl::millis() - start) < timeoutMs) {
            if (readRawIfReady(outRaw)) {
                return true;
            }
            TimeControl::delayMs(pollDelayMs);
        }
        return false;
    }

    void NAU7802::powerDown() {
        if (bus_ == nullptr) {
            return;
        }
        clearBits(REG_PU_CTRL, PU_CTRL_PUA);
        clearBits(REG_PU_CTRL, PU_CTRL_PUD);
    }

    bool NAU7802::powerUp(uint32_t readyTimeoutMs) {
        if (bus_ == nullptr) {
            return false;
        }
        setBits(REG_PU_CTRL, PU_CTRL_PUD);
        setBits(REG_PU_CTRL, PU_CTRL_PUA);

        const TimeControl::ms_tick_t start = TimeControl::millis();
        while ((TimeControl::millis() - start) < readyTimeoutMs) {
            uint8_t puCtrl = 0;
            if (readReg(REG_PU_CTRL, puCtrl) && (puCtrl & PU_CTRL_PUR) != 0) {
                return true;
            }
            TimeControl::delayMs(1);
        }
        return false;
    }

    void NAU7802::reset() {
        if (bus_ == nullptr) {
            return;
        }
        // RR bit triggers a full register reset.
        setBits(REG_PU_CTRL, PU_CTRL_RR);
        TimeControl::delayMs(1);
        clearBits(REG_PU_CTRL, PU_CTRL_RR);

        initialized_ = false;
    }

    // ---- Configuration ----

    void NAU7802::setGain(Gain gain) {
        if (bus_ == nullptr) {
            return;
        }
        uint8_t ctrl1 = 0;
        if (!readReg(REG_CTRL1, ctrl1)) {
            return;
        }
        ctrl1 = (ctrl1 & 0xF8U) | (static_cast<uint8_t>(gain) & 0x07U);
        writeReg(REG_CTRL1, ctrl1);
    }

    void NAU7802::setSampleRate(SampleRate rate) {
        if (bus_ == nullptr) {
            return;
        }
        uint8_t ctrl2 = 0;
        if (!readReg(REG_CTRL2, ctrl2)) {
            return;
        }
        // Sample rate is bits [6:4] in CTRL2.
        ctrl2 = (ctrl2 & 0x8FU) | ((static_cast<uint8_t>(rate) & 0x07U) << 4U);
        writeReg(REG_CTRL2, ctrl2);
    }

    void NAU7802::setLdoVoltage(LdoVoltage voltage) {
        if (bus_ == nullptr) {
            return;
        }
        uint8_t ctrl1 = 0;
        if (!readReg(REG_CTRL1, ctrl1)) {
            return;
        }
        // LDO voltage is bits [5:3] in CTRL1.
        ctrl1 = (ctrl1 & 0xC7U) | ((static_cast<uint8_t>(voltage) & 0x07U) << 3U);
        writeReg(REG_CTRL1, ctrl1);
    }

    void NAU7802::setChannel(Channel channel) {
        if (bus_ == nullptr) {
            return;
        }
        uint8_t ctrl2 = 0;
        if (!readReg(REG_CTRL2, ctrl2)) {
            return;
        }
        // Channel select is bit 7 in CTRL2.
        if (channel == Channel::CH2) {
            ctrl2 |= 0x80U;
        } else {
            ctrl2 &= 0x7FU;
        }
        writeReg(REG_CTRL2, ctrl2);
    }

    bool NAU7802::calibrateAfe() {
        if (bus_ == nullptr) {
            return false;
        }
        // Start internal offset calibration.
        if (!setBits(REG_CTRL2, CTRL2_CALS)) {
            return false;
        }

        // Wait for CALS bit to clear (calibration complete).
        const TimeControl::ms_tick_t start = TimeControl::millis();
        while ((TimeControl::millis() - start) < 1000U) {
            uint8_t ctrl2 = 0;
            if (readReg(REG_CTRL2, ctrl2) && (ctrl2 & CTRL2_CALS) == 0) {
                return (ctrl2 & CTRL2_CAL_ERR) == 0;  // true if no error
            }
            TimeControl::delayMs(1);
        }
        return false;  // timeout
    }

}  // namespace ungula
