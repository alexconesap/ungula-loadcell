// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#include "hx711.h"

#include <hal/gpio/gpio_access.h>
#include <time/time_control.h>

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#endif

namespace ungula {

    namespace {
#if defined(ESP_PLATFORM)
        // Protect the HX711 read sequence from interruption.
        portMUX_TYPE g_hx711Mux = portMUX_INITIALIZER_UNLOCKED;
#endif
    }  // namespace

    // After reading the 24-bit value from the HX711, we must send extra clock pulses
    // to tell the chip "which channel and gain to use for the NEXT conversion". This is the way the
    // HX711 works, folks.
    //
    // A128 -> 1 pulse
    // B32  -> 2 pulses
    // A64  -> 3 pulses
    uint8_t HX711::configToPulseCount(InputConfig config) {
        switch (config) {
            case InputConfig::A128:
                return 1U;
            case InputConfig::A64:
                return 3U;
            case InputConfig::B32:
                return 2U;
            default:
                return 1U;
        }
    }

    uint8_t HX711::shiftInByteMsbFirst(uint8_t dataPin, uint8_t clockPin) {
        // Bit-banged byte read with fixed timing.
        uint8_t value = 0U;

        for (uint8_t i = 0; i < 8U; ++i) {
            gpio::setHigh(clockPin);
            TimeControl::delayUs(kClockPulseDelayUs);

            value <<= 1U;
            if (gpio::read(dataPin)) {
                value |= 0x01U;
            }

            gpio::setLow(clockPin);
            TimeControl::delayUs(kClockPulseDelayUs);
        }

        return value;
    }

    int32_t HX711::signExtend24(uint8_t b2, uint8_t b1, uint8_t b0) {
        // Expand the chip 24-bit signed format into int32_t.
        const uint32_t raw24 = (static_cast<uint32_t>(b2) << 16U) |
                               (static_cast<uint32_t>(b1) << 8U) | static_cast<uint32_t>(b0);

        if ((b2 & 0x80U) != 0U) {
            return static_cast<int32_t>(raw24 | 0xFF000000UL);
        }

        return static_cast<int32_t>(raw24);
    }

    bool HX711::begin(uint8_t dataPin, uint8_t clockPin, InputConfig config) {
        const bool okClock = gpio::configOutput(clockPin);
        const bool okData = gpio::configInput(dataPin);

        if (!okClock || !okData) {
            initialized_ = false;
            return false;
        }

        dataPin_ = dataPin;
        clockPin_ = clockPin;
        config_ = config;
        extraPulses_ = configToPulseCount(config);

        gpio::setLow(clockPin_);

        initialized_ = true;
        return true;
    }

    bool HX711::isInitialized() const {
        return initialized_;
    }

    bool HX711::isReady() const {
        // DOUT goes low when a conversion is ready.
        return initialized_ && gpio::isLow(dataPin_);
    }

    bool HX711::readRawIfReady(int32_t& outRaw) {
        if (!isReady()) {
            return false;
        }

        const int32_t raw = readRawNow();

        // discard first sample after mode change to avoid returning wrong readings
        if (discardNextSample_) {
            discardNextSample_ = false;
            return false;
        }

        outRaw = raw;
        return true;
    }

    bool HX711::readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs) {
        while (true) {
            if (!waitReadyUntil(timeoutMs, pollDelayMs)) {
                return false;
            }

            const int32_t raw = readRawNow();

            if (discardNextSample_) {
                discardNextSample_ = false;
                continue;  // discard first sample after config change
            }

            outRaw = raw;
            return true;
        }
    }

    void HX711::setInputConfig(InputConfig config) {
        if (config_ == config) {
            return;
        }
        config_ = config;
        extraPulses_ = configToPulseCount(config);
        discardNextSample_ = true;
    }

    HX711::InputConfig HX711::inputConfig() const {
        return config_;
    }

    void HX711::powerDown() {
        // HX711 enters power-down when PD_SCK stays high for more than 60us (datasheet).
        // 70us gives a small margin.
        gpio::setHigh(clockPin_);
        TimeControl::delayUs(70);
    }

    bool HX711::powerUp(uint32_t readyTimeoutMs) {
        // Bringing PD_SCK low wakes the chip. After wake the HX711 always returns to A128 mode
        // regardless of the previous configuration — keep our internal state consistent with that.
        gpio::setLow(clockPin_);

        config_ = InputConfig::A128;
        extraPulses_ = configToPulseCount(config_);
        discardNextSample_ = true;

        // First conversion after wake takes ~400ms (10 SPS) or ~50ms (80 SPS). Block until DOUT
        // goes low so callers don't have to remember this quirk. Poll every 5ms to stay responsive
        // without hammering the GPIO.
        return waitReadyUntil(readyTimeoutMs, 5U);
    }

    void HX711::reset() {
        // HX711 has no reset register — power-cycle via the clock line and wait up to 500ms
        // for the first fresh sample.
        powerDown();
        (void)powerUp(500U);
    }

    bool HX711::waitReadyUntil(uint32_t timeoutMs, uint32_t pollDelayMs) const {
        const TimeControl::ms_tick_t start = TimeControl::millis();

        while ((TimeControl::millis() - start) < timeoutMs) {
            if (isReady()) {
                return true;
            }
            TimeControl::delayMs(pollDelayMs);
        }

        return false;  // timeout expired
    }

    void HX711::applyConfigSelection() {
        for (uint8_t i = 0; i < extraPulses_; ++i) {
            gpio::setHigh(clockPin_);
            TimeControl::delayUs(kClockPulseDelayUs);
            gpio::setLow(clockPin_);
            TimeControl::delayUs(kClockPulseDelayUs);
        }
    }

    int32_t HX711::readRawNow() {
        uint8_t by2 = 0U;
        uint8_t by1 = 0U;
        uint8_t by0 = 0U;

#if defined(ESP_PLATFORM)
        portENTER_CRITICAL(&g_hx711Mux);
#endif

        by2 = shiftInByteMsbFirst(dataPin_, clockPin_);
        by1 = shiftInByteMsbFirst(dataPin_, clockPin_);
        by0 = shiftInByteMsbFirst(dataPin_, clockPin_);
        applyConfigSelection();

#if defined(ESP_PLATFORM)
        portEXIT_CRITICAL(&g_hx711Mux);
#endif

        return signExtend24(by2, by1, by0);
    }

}  // namespace ungula
