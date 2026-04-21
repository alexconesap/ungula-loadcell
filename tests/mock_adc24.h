// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa

#pragma once

#include <loadcell/i_adc24.h>

#include <cstddef>
#include <vector>

// Scripted IAdc24 mock. Pre-populate a queue of raw values — each readRawIfReady /
// readRawWithin call pops the next value. When the queue empties the mock reports
// "not ready" or times out. Power management / reset are counted for inspection.
class MockAdc24 : public ungula::IAdc24 {
    public:
        bool isInitialized() const override {
            return initialized_;
        }

        bool isReady() const override {
            return initialized_ && !samples_.empty();
        }

        bool readRawIfReady(int32_t& outRaw) override {
            if (samples_.empty()) {
                return false;
            }
            outRaw = samples_.front();
            samples_.erase(samples_.begin());
            ++readCount_;
            return true;
        }

        bool readRawWithin(int32_t& outRaw, uint32_t /*timeoutMs*/,
                           uint32_t /*pollDelayMs*/) override {
            if (samples_.empty()) {
                return false;
            }
            outRaw = samples_.front();
            samples_.erase(samples_.begin());
            ++readCount_;
            return true;
        }

        void powerDown() override {
            ++powerDownCount_;
        }

        bool powerUp(uint32_t /*readyTimeoutMs*/) override {
            ++powerUpCount_;
            return true;
        }

        void reset() override {
            ++resetCount_;
        }

        // --- Scripting helpers (test-side only) ---

        void setInitialized(bool value) {
            initialized_ = value;
        }

        void pushSample(int32_t raw) {
            samples_.push_back(raw);
        }

        void pushSamples(std::initializer_list<int32_t> values) {
            for (int32_t value : values) {
                samples_.push_back(value);
            }
        }

        void clearSamples() {
            samples_.clear();
        }

        std::size_t pendingSamples() const {
            return samples_.size();
        }

        int readCount() const {
            return readCount_;
        }

        int powerDownCount() const {
            return powerDownCount_;
        }

        int powerUpCount() const {
            return powerUpCount_;
        }

        int resetCount() const {
            return resetCount_;
        }

    private:
        bool initialized_ = true;
        std::vector<int32_t> samples_;
        int readCount_ = 0;
        int powerDownCount_ = 0;
        int powerUpCount_ = 0;
        int resetCount_ = 0;
};
