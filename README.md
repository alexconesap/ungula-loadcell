# UngulaLoadcell

> **High-performance embedded C++ libraries for ESP32, STM32 and other MCUs** — load cell drivers (HX711, NAU7802, ADS1220, ADS1232) with tension sensing.

Load cell driver library for constrained embedded systems (ESP32, Arduino).

## Table of Contents

- [Features](#features)
- [Supported chips](#supported-chips)
- [Dependencies](#dependencies)
- [Quick Start](#quick-start)
- [Construction vs. use](#construction-vs-use)
- [Calibration](#calibration)
  - [Step 1 — Capture Zero](#step-1--capture-zero)
  - [Step 2 — Determine Scale Factor](#step-2--determine-scale-factor)
  - [Reading in Different Units](#reading-in-different-units)
  - [Converting Cached Values](#converting-cached-values)
- [HX711 input configuration](#hx711-input-configuration)
- [Raw reads (uncalibrated)](#raw-reads-uncalibrated)
- [Power management](#power-management)
- [Storing and Restoring Calibration](#storing-and-restoring-calibration)
- [Polling Pattern (Typical Embedded Loop)](#polling-pattern-typical-embedded-loop)
- [TensionSensor](#tensionsensor)
  - [Reading and controlling](#reading-and-controlling)
  - [API](#api)
  - [Config](#config)
- [Force Conversion and Safe Tension Calculator](#force-conversion-and-safe-tension-calculator)
  - [Unit conversion functions](#unit-conversion-functions)
  - [Safe tension calculator](#safe-tension-calculator)
  - [Example 1 — 304 Stainless Steel mandrel, 0.5 mm diameter](#example-1--304-stainless-steel-mandrel-05-mm-diameter)
  - [Example 2 — Nitinol mandrel, 0.3 mm diameter](#example-2--nitinol-mandrel-03-mm-diameter)
  - [Example 3 — Imperial datasheet (psi + inches)](#example-3--imperial-datasheet-psi--inches)
  - [Example 4 — Comparing materials side by side](#example-4--comparing-materials-side-by-side)
  - [Standalone conversions](#standalone-conversions)
- [Testing](#testing)
- [License](#license)

The library is split into two layers so you can swap the 24-bit ADC chip without touching the calibration or unit-conversion code:

- **`IAdc24`** — chip-neutral interface for any 24-bit signed delta-sigma ADC (ready polling, raw sample, power management, reset).
- **`HX711`, `NAU7802`, `ADS1220`, `ADS1232`** — concrete drivers. Each handles its own wiring (GPIO bit-bang, I2C, SPI) and chip-specific config (gain values, channel multiplexing, data rate).
- **`LoadCell`** — chip-agnostic load-cell semantics: zero offset, calibration scale, unit conversion (Newtons / kilograms-force / pounds-force), convenience reads. Takes any `IAdc24` via constructor injection.
- **`TensionSensor`** — filtered tension reading with EMA smoothing, stability detection, and target tolerance bands. Wraps a `LoadCell` reference for process-control use cases (e.g. mandrel winding, tension monitoring).
- **`force_convert.h`** — header-only utilities for force/stress/length unit conversions and safe working tension calculation from material datasheets.

## Features

- Clean separation of wire protocol, sample lifecycle, and load-cell math
- All internal state kept as raw ADC counts (int32_t). Unit conversion happens only at the output boundary when the caller asks for a float value
- Calibrated output in Newtons or kilograms-force
- Calibration accepts either kg (weight) or N (force gauge) — gravity conversion is internal
- Zero capture with multi-sample averaging
- Power management (sleep / wake, blocks until first sample is ready)
- HX711/ADS1232: critical section protection on ESP32 (FreeRTOS portMUX)

## Supported chips

| Chip | Bus | Gain | Data rate | Driver class |
| ---- | --- | ---- | --------- | ------------ |
| HX711 | GPIO bit-bang | 32, 64, 128 (via extra pulses) | 10/80 SPS | `ungula::HX711` |
| NAU7802 | I2C (0x2A) | 1–128 (register) | 10–320 SPS | `ungula::NAU7802` |
| ADS1220 | SPI (mode 1) | 1–128 (register) | 20–1000 SPS | `ungula::ADS1220` |
| ADS1232 | GPIO bit-bang | 1, 2, 64, 128 (GAIN0/GAIN1 pins) | 10/80 SPS | `ungula::ADS1232` |

All four expose the same `IAdc24` surface. Pick the chip at construction time and the rest of the code stays unchanged.

## Dependencies

- `UngulaCore` — provides `TimeControl` abstraction.
- `UngulaHal` — provides `gpio::`, `i2c::I2cMaster`, `spi::SpiMaster`.

## Quick Start

```cpp
#include <ungula_loadcell.h>

using MU = ungula::LoadCell::ForceUnit;

ungula::HX711 adc;
ungula::LoadCell cell(adc);  // wire the load-cell layer on top of the HX711

void setup() {
  // Data on GPIO 16, Clock on GPIO 4, default gain A128
  if (!adc.begin(16, 4)) {
    // handle error
    return;
  }

  // Capture zero with 10 samples (no load on the cell)
  cell.captureZero(10, 1000);

  // Set calibration: raw counts per kg (from a previous calibration session)
  cell.setCountsPerKgf(5000.0F);
}

void loop() {
  float kg = 0.0F;
  if (cell.readIfReady(kg, MU::KGF)) {
    // kg holds the weight in kilograms read from the cell
  }
}
```

## Construction vs. use

The **construction** of the ADC is chip-specific and always will be — GPIO pins, I2C bus + address, SPI bus + CS + DRDY are not interchangeable.
After construction, every chip speaks the same `IAdc24` dialect, and `LoadCell` does not care which one it is driving behind the hood.

```cpp
// Option 1 — HX711 (GPIO bit-bang)
ungula::HX711 hx;
hx.begin(dataPin, clockPin);
ungula::LoadCell cell(hx);

// Option 2 — NAU7802 on I2C
ungula::i2c::I2cMaster i2cBus(0);
i2cBus.begin(sdaPin, sclPin, 400000);
ungula::NAU7802 nau;
nau.begin(i2cBus);
nau.setGain(ungula::NAU7802::Gain::X128);
ungula::LoadCell cell(nau);

// Option 3 — ADS1220 on SPI with DRDY
ungula::spi::SpiMaster spiBus;
spiBus.begin(sclkPin, misoPin, mosiPin, csPin, 1000000, 1);
ungula::ADS1220 ads;
ads.begin(spiBus, drdyPin);
ads.setGain(ungula::ADS1220::Gain::X128);
ungula::LoadCell cell(ads);

// Option 4 — ADS1232 (GPIO bit-bang, hardware gain)
ungula::ADS1232 ads32;
ads32.begin(doutPin, sclkPin, pdwnPin, speedPin, a0Pin, gain0Pin, gain1Pin, tempPin);
ads32.setGain(ungula::ADS1232::Gain::X128);
ungula::LoadCell cell(ads32);
```

The `LoadCell` API is identical across all four.

## Calibration

`LoadCell` handles the calibration math on raw counts. The driver is a black box: calibrate once with a known reference, then read in whatever unit you want. The gravity constant and unit conversions are applied internally.

Calibration requires two steps:

- Capturing the zero offset (cell read value when no weight/force is applied) and
- Determining the scale factor (for example how many cell counts mean 1Kg in the real-world). You will need a crane scale or a digital force gauge to check it yourself.

### Step 1 — Capture Zero

With nothing on the load cell:

```cpp
// Average 20 samples, 1 second timeout per sample
cell.captureZero(20, 1000);
```

### Step 2 — Determine Scale Factor

Three options, depending on what calibration tool you have.

#### Option A — Calibrate with a known weight (kg)

Place a known weight on the load cell (e.g. a 2 kg calibration weight), read the raw ADC value, then tell the driver what weight that was:

```cpp
using MU = ungula::LoadCell::ForceUnit;

int32_t rawAtForce = 0;
if (adc.readRawWithin(rawAtForce, 2000, 0)) {
  // "The weight on the cell right now is 2.0 kg"
  cell.calibrate(2.0F, rawAtForce, MU::KGF);
}
```

The library converts kg to Newtons internally using the standard gravity constant (9.80665 m/s²).

#### Option B — Calibrate with a force gauge (Newtons)

If you have a force gauge or electronic crane scale that reads in Newtons:

```cpp
using MU = ungula::LoadCell::ForceUnit;

int32_t rawAtForce = 0;
if (adc.readRawWithin(rawAtForce, 2000, 0)) {
  // "The force gauge reads 19.613 N right now"
  cell.calibrate(19.613F, rawAtForce, MU::NEWTONS);
}
```

#### Option C — Set scale factor directly

If you already know the scale factor from a previous calibration session:

```cpp
using MU = ungula::LoadCell::ForceUnit;

// Counts per kg — from a previous calibration
cell.setCountsPerKgf(4500.0F);

// Or equivalently, counts per Newton
cell.setCountsPerNewton(50000.0F);
```

Both calls produce the same internal state. `LoadCell` always stores the factor as counts-per-Newton, regardless of the unit you pass in.

### Reading in Different Units

After calibration, read in any unit. The library converts internally:

```cpp
using MU = ungula::LoadCell::ForceUnit;

float newtons = 0.0F;
float kilograms = 0.0F;

// Non-blocking — returns false if no sample ready
cell.readIfReady(newtons, MU::NEWTONS);
cell.readIfReady(kilograms, MU::KGF);

// Blocking — waits up to 500 ms
cell.readWithin(newtons, 500, 0, MU::NEWTONS);
cell.readWithin(kilograms, 500, 0, MU::KGF);
```

### Converting Cached Values

If you cache raw readings and need to convert them later:

```cpp
using MU = ungula::LoadCell::ForceUnit;

int32_t raw = 0;
adc.readRawIfReady(raw);

// Convert an absolute raw ADC value (offset subtracted by LoadCell)
float newtons = cell.rawToUnit(raw, MU::NEWTONS);
float kg = cell.rawToUnit(raw, MU::KGF);

// If you already subtracted the offset yourself
int32_t net = raw - cell.offset();
float newtonsFromNet = cell.netToUnit(net, MU::NEWTONS);
float kgFromNet = cell.netToUnit(net, MU::KGF);
```

Both functions return `NAN` if the cell has not been calibrated yet (scale factor is zero). Check with `isnan()` before using the value in downstream math.

## HX711 input configuration

The HX711 supports three input modes, accessed through the concrete `HX711` class (not through `IAdc24` — gain sets differ across chips):

| Config | Channel | Gain | Use case                                     |
| ------ | ------- | ---- | -------------------------------------------- |
| A128   | A       | 128x | Most sensitive, default for single load cell |
| A64    | A       | 64x  | Wider range, less sensitive                  |
| B32    | B       | 32x  | Second input channel                         |

```cpp
// Set at initialization
adc.begin(dataPin, clockPin, ungula::HX711::InputConfig::A64);

// Change at runtime (first sample after change is discarded automatically)
adc.setInputConfig(ungula::HX711::InputConfig::B32);
```

## Raw reads (uncalibrated)

For direct access to the 24-bit signed ADC value, bypassing calibration entirely, go through the driver:

```cpp
int32_t raw = 0;

// Non-blocking
if (adc.readRawIfReady(raw)) {
  // raw is the signed 24-bit value (-8388608 to 8388607)
}

// Blocking with timeout
if (adc.readRawWithin(raw, 1000, 0)) {
  // got a sample within 1 second
}
```

## Power management

```cpp
adc.powerDown();                // chip enters low-power mode
// ... do other work ...
if (!adc.powerUp(500)) {        // blocks until the first sample is ready (up to 500 ms)
  // log_warn("ADC did not wake up in time");
}

adc.reset();                    // chip-specific reset — returns to defaults
```

## Storing and Restoring Calibration

Save offset and scale to NVS or EEPROM for use across reboots. `countsPerNewton()` always returns the canonical counts-per-Newton value, regardless of the unit used during calibration:

```cpp
using MU = ungula::LoadCell::ForceUnit;

// Save
int32_t savedOffset = cell.offset();
float savedScale = cell.countsPerNewton();

// Restore
cell.setOffset(savedOffset);
cell.setCountsPerNewton(savedScale);
```

## Polling Pattern (Typical Embedded Loop)

```cpp
using MU = ungula::LoadCell::ForceUnit;

void loop() {
  float kg = 0.0F;

  if (cell.readIfReady(kg, MU::KGF)) {
    if (kg > threshold) {
      // trigger action
    }
  }

  // HX711 outputs at ~10 or ~80 SPS depending on RATE pin.
  // No need to poll faster than the conversion rate.
  TimeControl::delayMs(100);
}
```

## TensionSensor

`TensionSensor` wraps a `LoadCell` and adds exponential moving average (EMA) filtering, stability detection, and target comparison with configurable tolerance bands. It keeps the filtering and "am I close enough?" logic in one place instead of spreading it across every caller.

```cpp
#include <ungula_loadcell.h>

using FU = ungula::LoadCell::ForceUnit;

ungula::HX711 adc;
ungula::LoadCell cell(adc);

// ... adc.begin(), cell.captureZero(), cell.setCountsPerKgf() ...

ungula::TensionSensor tension(cell);

ungula::TensionSensor::Config cfg;
cfg.averageSamples = 8;
cfg.stabilityTolerance = 0.03F;   // kgf
cfg.targetTolerance = 0.02F;      // kgf
cfg.unit = FU::KGF;

tension.configure(cfg);
tension.setTarget(0.85F);
```

### Reading and controlling

```cpp
void loop() {
  float current = 0.0F;
  if (tension.readStable(current, 200, 1)) {
    if (tension.isAtTarget()) {
      motor.stop();
    } else if (tension.isBelowTarget()) {
      motor.stepBackward();
    } else {
      motor.stepForward();
    }
  }
}
```

### API

| Method | Description |
| ------ | ----------- |
| `configure(config)` | Set averaging window, stability tolerance, target tolerance, and output unit |
| `readInstant(outTension)` | Non-blocking single read through the EMA filter. Returns false if no ADC sample is ready |
| `readStable(outTension, timeoutMs, pollDelayMs)` | Blocks until the filtered reading stabilizes (variation within `stabilityTolerance`), or times out |
| `isStable()` | Whether the last two filtered values were within `stabilityTolerance` |
| `lastTension()` | Most recent raw (unfiltered) reading in the configured unit |
| `filteredTension()` | Current EMA-filtered value |
| `lastVariation()` | Difference between current and previous filtered values |
| `setTarget(target)` | Set the tension target in the configured unit |
| `target()` | Current target value |
| `errorToTarget()` | `filteredTension - target` (signed) |
| `isAtTarget()` | `abs(error) <= targetTolerance` |
| `isAboveTarget()` | `filteredTension > target + targetTolerance` |
| `isBelowTarget()` | `filteredTension < target - targetTolerance` |

### Config

| Field | Default | Description |
| ----- | ------- | ----------- |
| `averageSamples` | 8 | EMA window size. Alpha = 2 / (N + 1) |
| `stabilityTolerance` | 0.05 | Max variation between consecutive filtered values to be considered stable |
| `targetTolerance` | 0.05 | Deadband around target for `isAtTarget()` |
| `unit` | NEWTONS | Output unit for all readings and comparisons |

## Force Conversion and Safe Tension Calculator

`force_convert.h` provides inline conversion functions and a safe working tension calculator. All functions live in `ungula::force`.

Mandrel manufacturers specify material properties (yield strength in MPa or psi) and geometry (diameter), not process force. Your control system needs force in Newtons, kgf, or lbf. These utilities bridge the gap.

### Unit conversion functions

| Function | Description |
| -------- | ----------- |
| `nToKgf(n)` / `kgfToN(kgf)` | Newtons ↔ kilograms-force |
| `nToLbf(n)` / `lbfToN(lbf)` | Newtons ↔ pounds-force |
| `kgfToLbf(kgf)` / `lbfToKgf(lbf)` | kgf ↔ lbf |
| `mpaToPsi(mpa)` / `psiToMpa(psi)` | Megapascals ↔ psi |
| `mpaToKsi(mpa)` / `ksiToMpa(ksi)` | Megapascals ↔ ksi (thousands of psi) |
| `mmToInch(mm)` / `inchToMm(inch)` | Millimeters ↔ inches |
| `circularAreaMm2(diameterMm)` | Cross-section area in mm² from diameter |
| `circularAreaIn2(diameterInch)` | Cross-section area in in² from diameter |

### Safe tension calculator

| Function | Description |
| -------- | ----------- |
| `safeTensionN(yieldMpa, diameterMm, safetyFactor)` | Safe working force in Newtons from metric inputs |
| `safeTensionFromPsi(yieldPsi, diameterInch, safetyFactor)` | Same, from imperial inputs |

The `safetyFactor` defaults to 0.3 (30% of yield). The formula is:

```text
safe force (N) = yield stress (MPa) × cross-section area (mm²) × safety factor
```

### Example 1 — 304 Stainless Steel mandrel, 0.5 mm diameter

The datasheet says yield strength is 215 MPa. No "max tension" is given — you compute it.

```cpp
#include <loadcell/force_convert.h>

using namespace ungula::force;

// 304 SS, 0.5 mm diameter, 30% safety margin
float safeN = safeTensionN(215.0F, 0.5F, 0.3F);   // -> 12.66 N
float safeKgf = nToKgf(safeN);                      // -> 1.29 kgf
float safeLbf = nToLbf(safeN);                      // -> 2.85 lbf

// Feed directly into the TensionSensor
tension.configure({.unit = FU::KGF});
tension.setTarget(safeKgf);                          // 1.29 kgf
```

### Example 2 — Nitinol mandrel, 0.3 mm diameter

Nitinol is much stronger but you want a wider safety margin because it's used in medical catheters.

```cpp
using namespace ungula::force;

// Nitinol, yield ~500 MPa, 0.3 mm diameter, 25% safety margin
float safeN = safeTensionN(500.0F, 0.3F, 0.25F);   // -> 8.84 N
float safeKgf = nToKgf(safeN);                      // -> 0.90 kgf

tension.configure({.unit = FU::NEWTONS});
tension.setTarget(safeN);                            // 8.84 N
```

### Example 3 — Imperial datasheet (psi + inches)

Some US suppliers specify yield in psi and diameter in inches.

```cpp
using namespace ungula::force;

// 304 SS, yield 31200 psi, diameter 0.020", safety 30%
float safeN = safeTensionFromPsi(31200.0F, 0.020F, 0.3F);  // -> 13.08 N
float safeLbf = nToLbf(safeN);                              // -> 2.94 lbf

tension.configure({.unit = FU::LBF});
tension.setTarget(safeLbf);
```

### Example 4 — Comparing materials side by side

```cpp
using namespace ungula::force;

// Same diameter (0.5 mm), same safety factor (30%), different materials
float ss304  = safeTensionN(215.0F,  0.5F);  // 304 SS     -> 12.66 N
float ss316  = safeTensionN(205.0F,  0.5F);  // 316 SS     -> 12.07 N
float nitinol = safeTensionN(500.0F, 0.5F);  // Nitinol    -> 29.45 N
float tungsten = safeTensionN(1500.0F, 0.5F); // Tungsten  -> 88.36 N (brittle!)

// For tungsten you would use a lower safety factor because it is brittle
float tungstenSafe = safeTensionN(1500.0F, 0.5F, 0.15F);  // -> 44.18 N
```

### Standalone conversions

The conversion functions are useful on their own, independent of the calculator.

```cpp
using namespace ungula::force;

// The machine reads 2.5 N — what is that in lbf?
float lbf = nToLbf(2.5F);     // -> 0.562 lbf

// A US operator reports 5 lbf — what is that in kgf?
float kgf = lbfToKgf(5.0F);   // -> 2.268 kgf

// Datasheet says 31200 psi — what is that in MPa?
float mpa = psiToMpa(31200.0F);  // -> 215.1 MPa

// Cross-section area for a 0.8 mm mandrel
float area = circularAreaMm2(0.8F);  // -> 0.503 mm²
```

## Testing

Host unit tests live in `tests/`. Four test executables:

- **test_ungula_loadcell** — `LoadCell` tests using `MockAdc24`. Covers calibration, offset capture, unit conversion and read delegation without real hardware (18 tests).
- **test_ungula_drivers** — compile and IAdc24 contract tests for all four concrete drivers (HX711, NAU7802, ADS1220, ADS1232). Uses the desktop HAL stubs to verify construction, begin(), and polymorphic dispatch (15 tests).
- **test_ungula_tension** — `TensionSensor` tests using `MockAdc24` + `LoadCell`. Covers EMA filtering, stability detection, target tracking, and `readStable` timeout behaviour (16 tests).
- **test_ungula_force_convert** — unit conversion round-trips and safe tension calculator with real-world material examples (12 tests).

```bash
cd tests
./1_build.sh
./2_run.sh
```

## Acknowledgements

Thanks to Claude and ChatGPT for helping on generating this documentation.

## License

MIT — Copyright (c) 2025-2026 Alex Conesa
