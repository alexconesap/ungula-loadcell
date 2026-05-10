// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.
//
// Host-side compile and IAdc24 contract tests for all four ADC drivers.
// The desktop stubs back all HAL calls, so we can verify construction,
// begin(), and the IAdc24 virtual dispatch without real hardware.

#include <gtest/gtest.h>

#include <ungula/loadcell/drivers/ads1220.h>
#include <ungula/loadcell/drivers/ads1232.h>
#include <ungula/loadcell/drivers/hx711.h>
#include <ungula/loadcell/drivers/nau7802.h>
#include <ungula/loadcell/i_adc24.h>

#include <ungula/hal/i2c/i2c_master.h>
#include <ungula/hal/spi/spi_master.h>

namespace
{

    // -- HX711 ----------------------------------------------------------------

    TEST(HX711DriverTest, NotInitializedBeforeBegin)
    {
        ungula::loadcell::HX711 adc;
        EXPECT_FALSE(adc.isInitialized());
    }

    TEST(HX711DriverTest, BeginSucceeds)
    {
        ungula::loadcell::HX711 adc;
        EXPECT_TRUE(adc.begin(16, 4));
        EXPECT_TRUE(adc.isInitialized());
    }

    TEST(HX711DriverTest, ImplementsIAdc24)
    {
        ungula::loadcell::HX711 adc;
        ungula::loadcell::IAdc24 *iface = &adc;
        EXPECT_FALSE(iface->isInitialized());
    }

    TEST(HX711DriverTest, InputConfigRoundTrips)
    {
        ungula::loadcell::HX711 adc;
        adc.begin(16, 4);
        adc.setInputConfig(ungula::loadcell::HX711::InputConfig::B32);
        EXPECT_EQ(adc.inputConfig(), ungula::loadcell::HX711::InputConfig::B32);
    }

    // -- NAU7802 --------------------------------------------------------------

    TEST(NAU7802DriverTest, NotInitializedBeforeBegin)
    {
        ungula::loadcell::NAU7802 adc;
        EXPECT_FALSE(adc.isInitialized());
    }

    TEST(NAU7802DriverTest, BeginWithStubI2cFailsPurTimeout)
    {
        ungula::hal::i2c::I2cMaster bus(0);
        bus.begin(21, 22, 400000);
        ungula::loadcell::NAU7802 adc;
        // Stub I2C reads return 0 for all registers, so PUR bit never gets set.
        // begin() times out waiting for PUR — expected failure, proves it compiles and links.
        EXPECT_FALSE(adc.begin(bus));
        EXPECT_FALSE(adc.isInitialized());
    }

    TEST(NAU7802DriverTest, ImplementsIAdc24)
    {
        ungula::loadcell::NAU7802 adc;
        ungula::loadcell::IAdc24 *iface = &adc;
        EXPECT_FALSE(iface->isInitialized());
    }

    // -- ADS1220 --------------------------------------------------------------

    TEST(ADS1220DriverTest, NotInitializedBeforeBegin)
    {
        ungula::loadcell::ADS1220 adc;
        EXPECT_FALSE(adc.isInitialized());
    }

    TEST(ADS1220DriverTest, BeginWithStubSpiSucceeds)
    {
        ungula::hal::spi::SpiMaster spi;
        spi.begin(18, 19, 23, 5, 1000000, 1);
        ungula::loadcell::ADS1220 adc;
        EXPECT_TRUE(adc.begin(spi, 4));
        EXPECT_TRUE(adc.isInitialized());
    }

    TEST(ADS1220DriverTest, ImplementsIAdc24)
    {
        ungula::loadcell::ADS1220 adc;
        ungula::loadcell::IAdc24 *iface = &adc;
        EXPECT_FALSE(iface->isInitialized());
    }

    // -- ADS1232 --------------------------------------------------------------

    TEST(ADS1232DriverTest, NotInitializedBeforeBegin)
    {
        ungula::loadcell::ADS1232 adc;
        EXPECT_FALSE(adc.isInitialized());
    }

    TEST(ADS1232DriverTest, BeginSucceeds)
    {
        ungula::loadcell::ADS1232 adc;
        EXPECT_TRUE(adc.begin(16, 4));
        EXPECT_TRUE(adc.isInitialized());
    }

    TEST(ADS1232DriverTest, ImplementsIAdc24)
    {
        ungula::loadcell::ADS1232 adc;
        ungula::loadcell::IAdc24 *iface = &adc;
        EXPECT_FALSE(iface->isInitialized());
    }

    TEST(ADS1232DriverTest, BeginWithAllPins)
    {
        ungula::loadcell::ADS1232 adc;
        EXPECT_TRUE(adc.begin(16, 4, 25, 26, 27, 32, 33, 14));
        EXPECT_TRUE(adc.isInitialized());
    }

    // -- IAdc24 polymorphism --------------------------------------------------

    TEST(IAdc24ContractTest, AllDriversReadyFalseBeforeBegin)
    {
        ungula::loadcell::HX711 hx;
        ungula::loadcell::NAU7802 nau;
        ungula::loadcell::ADS1220 ads12;
        ungula::loadcell::ADS1232 ads32;

        ungula::loadcell::IAdc24 *drivers[] = { &hx, &nau, &ads12, &ads32 };

        for (auto *drv : drivers) {
            EXPECT_FALSE(drv->isInitialized());
            EXPECT_FALSE(drv->isReady());
            int32_t raw = 0;
            EXPECT_FALSE(drv->readRawIfReady(raw));
        }
    }

} // namespace
