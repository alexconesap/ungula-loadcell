// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once
#ifndef __cplusplus
#error UngulaLoadcell requires a C++ compiler
#endif

// Ungula Load Cell Library - load cell drivers for embedded projects
// Include this header to activate the library in Arduino

// Depend on UngulaCore and UngulaHal — must be included first so
// Arduino CLI discovers their include paths before our headers reference them.
#include <ungula_hal.h>
#include <ungula_core.h>

// Chip-neutral interface (implemented by every concrete ADC driver)
#include "loadcell/i_adc24.h"

// Concrete ADC drivers
#include "loadcell/drivers/ads1220.h"
#include "loadcell/drivers/ads1232.h"
#include "loadcell/drivers/hx711.h"
#include "loadcell/drivers/nau7802.h"

// Chip-neutral load-cell semantics (calibration, offset, unit conversion)
#include "loadcell/load_cell.h"

// Filtered tension reading with stability detection and target tracking
#include "loadcell/tension_sensor.h"

// Force/stress unit conversions and safe working tension calculator
#include "loadcell/force_convert.h"
