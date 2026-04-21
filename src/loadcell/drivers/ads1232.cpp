// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "ads1232.h"

#include <hal/gpio/gpio_access.h>
#include <time/time_control.h>

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#endif

namespace ungula {

    namespace {
#if defined(ESP_PLATFORM)
        portMUX_TYPE g_ads1232Mux = portMUX_INITIALIZER_UNLOCKED;
#endif
    }  // namespace

    bool ADS1232::begin(uint8_t doutPin, uint8_t sclkPin, uint8_t pdwnPin, uint8_t speedPin,
                        uint8_t a0Pin, uint8_t gain0Pin, uint8_t gain1Pin, uint8_t tempPin) {
        if (!gpio::configInput(doutPin) || !gpio::configOutput(sclkPin)) {
            initialized_ = false;
            return false;
        }

        doutPin_ = doutPin;
        sclkPin_ = sclkPin;
        pdwnPin_ = pdwnPin;
        speedPin_ = speedPin;
        a0Pin_ = a0Pin;
        gain0Pin_ = gain0Pin;
        gain1Pin_ = gain1Pin;
        tempPin_ = tempPin;

        gpio::setLow(sclkPin_);

        // Configure optional control pins with safe defaults.
        if (pdwnPin_ != GPIO_NONE) {
            gpio::configOutput(pdwnPin_);
            gpio::setHigh(pdwnPin_);  // active low — high = running
        }
        if (speedPin_ != GPIO_NONE) {
            gpio::configOutput(speedPin_);
            gpio::setLow(speedPin_);  // default 10 SPS
        }
        if (a0Pin_ != GPIO_NONE) {
            gpio::configOutput(a0Pin_);
            gpio::setLow(a0Pin_);  // default channel 1
        }
        if (gain0Pin_ != GPIO_NONE) {
            gpio::configOutput(gain0Pin_);
            gpio::setHigh(gain0Pin_);  // default GAIN0=1
        }
        if (gain1Pin_ != GPIO_NONE) {
            gpio::configOutput(gain1Pin_);
            gpio::setHigh(gain1Pin_);  // default GAIN1=1 → gain 128
        }
        if (tempPin_ != GPIO_NONE) {
            gpio::configOutput(tempPin_);
            gpio::setLow(tempPin_);  // default normal mode (bridge input)
        }

        initialized_ = true;
        return true;
    }

    bool ADS1232::isInitialized() const {
        return initialized_;
    }

    bool ADS1232::isReady() const {
        return initialized_ && gpio::isLow(doutPin_);
    }

    bool ADS1232::readRawIfReady(int32_t& outRaw) {
        if (!isReady()) {
            return false;
        }
        outRaw = readRawNow();
        return true;
    }

    bool ADS1232::readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) {
        if (!waitReadyUntil(timeoutMs, pollDelayMs)) {
            return false;
        }
        outRaw = readRawNow();
        return true;
    }

    void ADS1232::powerDown() {
        if (pdwnPin_ != GPIO_NONE) {
            gpio::setLow(pdwnPin_);  // PDWN active low — low = power down
            TimeControl::delayUs(100);
        }
    }

    bool ADS1232::powerUp(uint32_t readyTimeoutMs) {
        if (pdwnPin_ != GPIO_NONE) {
            gpio::setHigh(pdwnPin_);  // PDWN high = normal operation
        }
        return waitReadyUntil(readyTimeoutMs, 5U);
    }

    void ADS1232::reset() {
        // Toggle PDWN pin to power-cycle the chip.
        powerDown();
        (void)powerUp(500U);
    }

    void ADS1232::setSampleRate(SampleRate rate) {
        if (speedPin_ == GPIO_NONE) {
            return;
        }
        gpio::write(speedPin_, rate == SampleRate::SPS_80);
    }

    void ADS1232::setGain(Gain gain) {
        uint8_t val = static_cast<uint8_t>(gain);
        if (gain0Pin_ != GPIO_NONE) {
            gpio::write(gain0Pin_, (val & 0x01U) != 0);
        }
        if (gain1Pin_ != GPIO_NONE) {
            gpio::write(gain1Pin_, (val & 0x02U) != 0);
        }
    }

    void ADS1232::setChannel(Channel channel) {
        if (a0Pin_ == GPIO_NONE) {
            return;
        }
        gpio::write(a0Pin_, channel == Channel::CH2);
    }

    void ADS1232::setTemperatureMode(bool enabled) {
        if (tempPin_ == GPIO_NONE) {
            return;
        }
        gpio::write(tempPin_, enabled);
    }

    bool ADS1232::selfCalibrate(uint32_t timeoutMs) {
        // Wait for a ready sample, read the 24 bits + ack pulse (25 clocks),
        // then send the 26th pulse to trigger self-offset calibration.
        if (!waitReadyUntil(timeoutMs, 5U)) {
            return false;
        }
        (void)readRawNow();  // 25 clocks (24 data + 1 ack)
        clockPulse();        // 26th clock → triggers self-offset calibration

        // Wait for DOUT to go low again (calibration complete, ~800ms at 10 SPS).
        return waitReadyUntil(timeoutMs, 10U);
    }

    // ---- Internal helpers ----

    void ADS1232::clockPulse() {
        gpio::setHigh(sclkPin_);
        TimeControl::delayUs(kClockPulseDelayUs);
        gpio::setLow(sclkPin_);
        TimeControl::delayUs(kClockPulseDelayUs);
    }

    uint8_t ADS1232::shiftInByteMsbFirst(uint8_t doutPin, uint8_t sclkPin) {
        uint8_t value = 0U;
        for (uint8_t i = 0; i < 8U; ++i) {
            gpio::setHigh(sclkPin);
            TimeControl::delayUs(kClockPulseDelayUs);

            value <<= 1U;
            if (gpio::read(doutPin)) {
                value |= 0x01U;
            }

            gpio::setLow(sclkPin);
            TimeControl::delayUs(kClockPulseDelayUs);
        }
        return value;
    }

    int32_t ADS1232::signExtend24(uint8_t byte2, uint8_t byte1, uint8_t byte0) {
        const uint32_t raw24 = (static_cast<uint32_t>(byte2) << 16U) |
                               (static_cast<uint32_t>(byte1) << 8U) | static_cast<uint32_t>(byte0);
        if ((byte2 & 0x80U) != 0U) {
            return static_cast<int32_t>(raw24 | 0xFF000000UL);
        }
        return static_cast<int32_t>(raw24);
    }

    bool ADS1232::waitReadyUntil(uint32_t timeoutMs, uint32_t pollDelayMs) const {
        const TimeControl::ms_tick_t start = TimeControl::millis();
        while ((TimeControl::millis() - start) < timeoutMs) {
            if (isReady()) {
                return true;
            }
            TimeControl::delayMs(pollDelayMs);
        }
        return false;
    }

    int32_t ADS1232::readRawNow() {
        // ADS1232: 24 data bits + 1 ack pulse = 25 total SCLK pulses.
        // The 25th pulse forces DOUT back high (data read acknowledged).
        uint8_t byte2 = 0U;
        uint8_t byte1 = 0U;
        uint8_t byte0 = 0U;

#if defined(ESP_PLATFORM)
        portENTER_CRITICAL(&g_ads1232Mux);
#endif

        byte2 = shiftInByteMsbFirst(doutPin_, sclkPin_);
        byte1 = shiftInByteMsbFirst(doutPin_, sclkPin_);
        byte0 = shiftInByteMsbFirst(doutPin_, sclkPin_);
        clockPulse();  // 25th clock — ack, forces DOUT high

#if defined(ESP_PLATFORM)
        portEXIT_CRITICAL(&g_ads1232Mux);
#endif

        return signExtend24(byte2, byte1, byte0);
    }

}  // namespace ungula
