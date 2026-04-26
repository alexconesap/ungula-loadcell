# UngulaLoadcell

Load-cell stack for embedded C++: a chip-neutral 24-bit ADC interface
(`IAdc24`), four concrete drivers (HX711, NAU7802, ADS1220, ADS1232),
calibration / unit conversion (`LoadCell`), filtered tension reading
with stability and target tracking (`TensionSensor`), and force / stress
unit helpers (`force_convert.h`). Pick the chip at construction; the
rest of the code does not change.

All public symbols live in namespace `ungula`. Activate the library
with a single include: `#include <ungula_loadcell.h>`.

---

## Usage

### Use case: HX711 + LoadCell, calibrate with a known weight, read kgf

```cpp
#include <Arduino.h>
#include <ungula_loadcell.h>

using ungula::LoadCell;

ungula::HX711 adc;
LoadCell cell(adc);

void setup() {
    adc.begin(/*dataPin=*/4, /*clockPin=*/5, ungula::HX711::InputConfig::A128);

    // 1) Capture zero with no load on the cell.
    cell.captureZero(/*sampleCount=*/16, /*timeoutPerSampleMs=*/1000);

    // 2) Apply a known mass (e.g. 1.000 kg). Read the raw counts and calibrate.
    int32_t rawAtForce = 0;
    adc.readRawWithin(rawAtForce, 1000, 0);
    cell.calibrate(1.0F, rawAtForce, LoadCell::ForceUnit::KGF);
}

void loop() {
    float kgf = 0.0F;
    if (cell.readIfReady(kgf, LoadCell::ForceUnit::KGF)) {
        // use kgf
    }
}
```

When to use this: simplest path. HX711 is GPIO bit-bang, no bus needed.

### Use case: NAU7802 over I2C

```cpp
#include <Arduino.h>
#include <ungula_loadcell.h>

ungula::i2c::I2cMaster bus(0);
ungula::NAU7802 adc;
ungula::LoadCell cell(adc);

void setup() {
    bus.begin(/*sda=*/21, /*scl=*/22, /*hz=*/400000);
    adc.begin(bus);
    adc.setGain(ungula::NAU7802::Gain::X128);
    adc.setSampleRate(ungula::NAU7802::SampleRate::SPS_80);
    adc.calibrateAfe();          // run after gain / channel / rate changes

    cell.captureZero(16, 1000);
    cell.setCountsPerKgf(123456.0F);  // pre-determined factor
}

void loop() {
    float n = 0.0F;
    if (cell.readWithin(n, 50, 1, ungula::LoadCell::ForceUnit::NEWTONS)) {
        // use n
    }
}
```

When to use this: I2C bridge sensor with on-chip PGA / LDO.

### Use case: ADS1220 over SPI

```cpp
#include <Arduino.h>
#include <ungula_loadcell.h>

ungula::spi::SpiMaster spi;
ungula::ADS1220 adc;
ungula::LoadCell cell(adc);

void setup() {
    spi.begin(/*sclk=*/18, /*miso=*/19, /*mosi=*/23, /*cs=*/5,
              /*hz=*/1000000, /*mode=*/1);
    adc.begin(spi, /*drdyPin=*/22);
    adc.setMux(ungula::ADS1220::Mux::AIN0_AIN1);
    adc.setGain(ungula::ADS1220::Gain::X128);
    adc.setDataRate(ungula::ADS1220::DataRate::SPS_20);
    adc.setConversionMode(ungula::ADS1220::ConversionMode::CONTINUOUS);

    cell.captureZero(16, 1000);
}

void loop() {
    float n = 0.0F;
    if (cell.readIfReady(n)) {
        // Newtons by default
    }
}
```

When to use this: ratiometric bridge, IDAC, SPI bus already present.

### Use case: ADS1232 (bit-bang) with hardware gain pins

```cpp
#include <Arduino.h>
#include <ungula_loadcell.h>

ungula::ADS1232 adc;
ungula::LoadCell cell(adc);

void setup() {
    // Tie SPEED, A0, GAIN0/1, TEMP externally — pass GPIO_NONE to leave them.
    adc.begin(/*dout=*/4, /*sclk=*/5, /*pdwn=*/16);
    adc.selfCalibrate(2000);

    cell.captureZero(16, 1000);
    cell.setCountsPerKgf(98765.0F);
}

void loop() {
    float kgf = 0.0F;
    if (cell.readIfReady(kgf, ungula::LoadCell::ForceUnit::KGF)) { /* ... */ }
}
```

### Use case: TensionSensor — EMA-filtered reading with stability and target

```cpp
#include <Arduino.h>
#include <ungula_loadcell.h>

ungula::HX711 adc;
ungula::LoadCell cell(adc);
ungula::TensionSensor tension(cell);

void setup() {
    adc.begin(4, 5);
    cell.captureZero(16, 1000);
    cell.setCountsPerNewton(420.0F);

    ungula::TensionSensor::Config cfg{};
    cfg.averageSamples = 8;
    cfg.stabilityTolerance = 0.05F;     // Newtons
    cfg.targetTolerance = 0.10F;
    cfg.unit = ungula::LoadCell::ForceUnit::NEWTONS;
    tension.configure(cfg);

    tension.setTarget(2.5F);            // 2.5 N target

    float n = 0.0F;
    if (tension.readStable(n, /*timeoutMs=*/2000)) {
        // n is the filtered, stable value
    }
}

void loop() {
    float n = 0.0F;
    if (tension.readInstant(n)) {
        if (tension.isAtTarget())   { /* on target */ }
        if (tension.isAboveTarget()){ /* relax     */ }
        if (tension.isBelowTarget()){ /* tighten   */ }
    }
}
```

When to use this: closed-loop tension control where raw samples are too
noisy and you need a debounced "is it stable / on target" signal.

### Use case: Safe working tension from material properties

```cpp
#include <ungula_loadcell.h>

using namespace ungula::force;

// 304 SS, yield 215 MPa, 0.5 mm wire, 30% safety factor.
const float maxN = safeTensionN(215.0F, 0.5F, 0.3F);

// Imperial datasheet (psi + inches).
const float maxN2 = safeTensionFromPsi(31000.0F, 0.020F, 0.3F);
```

When to use this: pick a target tension before runtime from a datasheet
yield strength and wire diameter.

---

## API

### Header: `<ungula_loadcell.h>`

Single include. Pulls `<ungula_hal.h>` and `<ungula_core.h>` first
(needed for Arduino CLI library discovery), then every public header in
the loadcell tree:

- `loadcell/i_adc24.h`
- `loadcell/drivers/{hx711,ads1220,ads1232,nau7802}.h`
- `loadcell/load_cell.h`
- `loadcell/tension_sensor.h`
- `loadcell/force_convert.h`

---

## Public types

### `ungula::IAdc24` (abstract)

Chip-neutral interface for 24-bit signed delta-sigma ADCs. All concrete
drivers implement it. `LoadCell` and `TensionSensor` only see this
interface — swap the chip without touching higher layers.

Methods (all pure virtual):

- `bool isInitialized() const` — driver wired up successfully.
- `bool isReady() const` — fresh sample available.
- `bool readRawIfReady(int32_t& outRaw)` — non-blocking; `false` if no
  sample.
- `bool readRawWithin(int32_t& outRaw, uint32_t timeoutMs, uint32_t pollDelayMs)`
  — wait up to `timeoutMs`; `pollDelayMs` is a cooperative yield between
  ready checks (`0` falls back to `TimeControl::yield()`).
- `void powerDown()` — chip-specific low-power mode.
- `bool powerUp(uint32_t readyTimeoutMs)` — wake and block until first
  post-wake sample.
- `void reset()` — return chip to default config.

### `ungula::HX711 : IAdc24`

GPIO bit-bang, two pins (data, clock). Gain / channel selection via
extra clock pulses after the 24-bit read.

- `enum class InputConfig : uint8_t { A128, A64, B32 };`
  - `A128` channel A, gain 128 (default, most sensitive)
  - `A64` channel A, gain 64
  - `B32` channel B, gain 32
- `bool begin(uint8_t dataPin, uint8_t clockPin, InputConfig = A128);`
- `void setInputConfig(InputConfig);` — first sample after a change is
  discarded automatically.
- `InputConfig inputConfig() const;`
- `void reset() override;` — equivalent to `powerDown()` + `powerUp()`;
  returns to A128 defaults.

### `ungula::NAU7802 : IAdc24`

I2C bridge ADC. Fixed address `NAU7802::I2C_ADDR == 0x2A`.

- Enums: `Gain { X1..X128 }`, `SampleRate { SPS_10, SPS_20, SPS_40,
  SPS_80, SPS_320 }`, `LdoVoltage { V_4_5..V_2_4 }`, `Channel { CH1, CH2 }`.
- `bool begin(ungula::i2c::I2cMaster& bus);` — bus must outlive driver.
- `void setGain(Gain);`
- `void setSampleRate(SampleRate);`
- `void setLdoVoltage(LdoVoltage);`
- `void setChannel(Channel);`
- `bool calibrateAfe();` — internal offset cal, blocks up to ~1 s. Run
  after changing gain / channel / rate.

### `ungula::ADS1220 : IAdc24`

SPI bridge ADC, mode 1 (CPOL=0, CPHA=1), MSB first. DRDY on a separate
GPIO (active low).

- Enums: `Gain { X1..X128 }`, `Mux { AIN0_AIN1, ..., AIN3_AVSS }`,
  `DataRate { SPS_20, SPS_45, SPS_90, SPS_175, SPS_330, SPS_600,
  SPS_1000 }`, `VoltageRef { INTERNAL_2V048, REFP0_REFN0, AIN0_AIN3,
  AVDD_AVSS }`, `ConversionMode { SINGLE_SHOT, CONTINUOUS }`.
- `bool begin(ungula::spi::SpiMaster& spi, uint8_t drdyPin);`
- `void setGain(Gain);` `void setMux(Mux);` `void setDataRate(DataRate);`
- `void setVoltageRef(VoltageRef);`
- `void setConversionMode(ConversionMode);`
- `void setTemperatureSensor(bool enabled);`
- `void setPgaBypass(bool bypass);`
- `void startConversion();` — required after each read in `SINGLE_SHOT`.

### `ungula::ADS1232 : IAdc24`

GPIO bit-bang ADC. Read is 25 SCLK pulses (24 data + 1 ack); 26 pulses
trigger self-offset calibration. Hardware control pins are optional —
pass `ADS1232::GPIO_NONE` (`0xFF`) to leave a pin unmanaged when it is
tied externally.

- Enums: `SampleRate { SPS_10, SPS_80 }`, `Gain { X1, X2, X64, X128 }`,
  `Channel { CH1, CH2 }`.
- `static constexpr uint8_t GPIO_NONE = 0xFF;`
- `bool begin(uint8_t doutPin, uint8_t sclkPin,
             uint8_t pdwnPin = GPIO_NONE, uint8_t speedPin = GPIO_NONE,
             uint8_t a0Pin = GPIO_NONE, uint8_t gain0Pin = GPIO_NONE,
             uint8_t gain1Pin = GPIO_NONE, uint8_t tempPin = GPIO_NONE);`
- `void setSampleRate(SampleRate);` `void setGain(Gain);`
  `void setChannel(Channel);` `void setTemperatureMode(bool);`
- `bool selfCalibrate(uint32_t timeoutMs = 2000);` — blocks ~800 ms at
  10 SPS.

### `ungula::LoadCell`

Chip-agnostic load-cell semantics on top of any `IAdc24`. Stores
calibration state (offset and counts-per-Newton); the canonical internal
unit is **Newtons**. Unit conversion only happens at the output boundary.

- `enum class ForceUnit : uint8_t { NEWTONS, KGF, LBF };`
- `explicit LoadCell(IAdc24& adc);` — adc must outlive the LoadCell.

Calibration:

- `bool captureZero(uint8_t sampleCount = 10, uint32_t timeoutPerSampleMs = 1000);`
  Averages `sampleCount` raw reads (uses `readRawWithin`). Returns
  `false` and leaves the offset unchanged if any sample times out.
- `void setOffset(int32_t rawOffset);` / `int32_t offset() const;`
- `void setCountsPerNewton(float);` / `void setCountsPerKgf(float);` /
  `void setCountsPerLbf(float);`
- `void calibrate(float knownValue, int32_t rawAtForce,
                 ForceUnit unit = ForceUnit::NEWTONS);`
  Computes counts-per-Newton from a known reference load and its raw
  reading. **No-op if `knownValue <= 0`.**
- `float countsPerNewton() const;` / `countsPerKgf()` / `countsPerLbf()`
  — derived from the canonical Newton factor.

Conversion:

- `float rawToUnit(int32_t raw, ForceUnit = NEWTONS) const;`
  Subtracts offset internally. Returns `NAN` if calibration scale is 0.
- `float netToUnit(int32_t netValue, ForceUnit = NEWTONS) const;`
  Caller already subtracted offset. Same `NAN` behavior.

Calibrated reads:

- `bool readIfReady(float& out, ForceUnit = NEWTONS);`
- `bool readWithin(float& out, uint32_t timeoutMs,
                  uint32_t pollDelayMs = 0, ForceUnit = NEWTONS);`

Escape hatch:

- `IAdc24& adc();` / `const IAdc24& adc() const;` — for raw reads, power
  control, chip-specific config.

### `ungula::TensionSensor`

Filtered tension on top of a `LoadCell`. EMA smoothing, stability
detection, target tracking. State (last raw, filtered, previous
filtered, target, stable flag) starts as `NAN` until the first sample.

- `using ForceUnit = LoadCell::ForceUnit;`
- `struct Config { uint8_t averageSamples = 8;
                   float stabilityTolerance = 0.05F;
                   float targetTolerance = 0.05F;
                   ForceUnit unit = ForceUnit::NEWTONS; };`
  EMA `alpha = 2 / (averageSamples + 1)` (standard EMA-from-N relation).
- `explicit TensionSensor(LoadCell&);`
- `void configure(const Config&);`
- `bool readInstant(float& outTension);` — non-blocking; updates filter,
  returns `true` and writes `filteredTension_` if a new sample landed.
- `bool readStable(float& outTension, uint32_t timeoutMs,
                  uint32_t pollDelayMs = 0);` — polls until stable or
  timeout. On timeout returns `false` and writes the latest filtered
  value (may be `NAN` if no sample ever arrived).
- `bool isStable() const;` — true once two consecutive filtered samples
  differ by `<= stabilityTolerance`.
- `float lastTension() const;` — last raw sample in configured unit.
- `float filteredTension() const;` — current EMA value.
- `float lastVariation() const;` — `filtered - previousFiltered`, or
  `NAN`.
- `void setTarget(float);` / `float target() const;`
- `float errorToTarget() const;` — `filtered - target`, or `NAN`.
- `bool isAtTarget() const;` — `|filtered - target| <= targetTolerance`.
- `bool isAboveTarget() const;` / `bool isBelowTarget() const;`

### `ungula::force::*` (header-only, in `force_convert.h`)

Free functions, all `inline float`. No state.

Force:
- `nToKgf(n)`, `kgfToN(kgf)` — `g = 9.80665`.
- `nToLbf(n)`, `lbfToN(lbf)` — factor `0.2248089431`.
- `kgfToLbf(kgf)`, `lbfToKgf(lbf)` — factor `2.20462262`.

Stress:
- `mpaToKsi(mpa)`, `ksiToMpa(ksi)` — factor `0.1450377`.
- `mpaToPsi(mpa)`, `psiToMpa(psi)` — factor `145.0377`.

Length / area:
- `mmToInch(mm)`, `inchToMm(inch)` — factor `25.4`.
- `circularAreaMm2(diameterMm)` / `circularAreaIn2(diameterInch)`.

Safe working tension:
- `safeTensionN(yieldMpa, diameterMm, safetyFactor = 0.3F)` — Newtons.
- `safeTensionFromPsi(yieldPsi, diameterInch, safetyFactor = 0.3F)` —
  Newtons (converts internally).

---

## Lifecycle

1. Construct the bus where applicable (`i2c::I2cMaster`,
   `spi::SpiMaster`) and call its `begin(...)` first.
2. Construct the ADC driver and call its chip-specific `begin(...)`.
   Returns `false` if wiring or chip handshake fails — do not proceed.
3. Configure chip-specific knobs (gain, MUX, sample rate, channel).
   For NAU7802 call `calibrateAfe()` after gain / channel / rate
   changes. For ADS1232 call `selfCalibrate()`. For ADS1220 in
   `SINGLE_SHOT` mode, call `startConversion()` per sample.
4. Construct `LoadCell` referencing the driver, then `captureZero()`
   with the cell unloaded, then either `calibrate(knownValue, raw, unit)`
   or `setCountsPerKgf/Newton/Lbf(...)` from a stored factor.
5. Operate via `readIfReady` / `readWithin`, or wrap in `TensionSensor`.
6. Use `powerDown()` / `powerUp()` for low-power cycles. After
   `powerUp(timeoutMs)` the first sample is ready.

References (`IAdc24& adc_` in `LoadCell`, `LoadCell& loadCell_` in
`TensionSensor`) are held by reference. The driver and load-cell
objects must outlive everything that wraps them.

---

## Error handling

No exceptions, no error codes — boolean returns and `NAN` sentinels.

- `IAdc24::readRawIfReady` / `readRawWithin` return `false` on no-sample
  / timeout. `outRaw` is unspecified in that case.
- `LoadCell::readIfReady` / `readWithin` return `false` if the
  underlying ADC call returned `false`.
- `LoadCell::captureZero` returns `false` on first sample timeout and
  leaves `offset_` unchanged.
- `LoadCell::calibrate(knownValue <= 0, ...)` is a silent no-op.
- `LoadCell::rawToUnit` / `netToUnit` return `NAN` when
  `countsPerNewton == 0` (no calibration applied yet).
- `TensionSensor` predicate methods (`isAtTarget`, `isAboveTarget`,
  `isBelowTarget`) return `false` when filtered or target is `NAN`.
  `errorToTarget` and `lastVariation` return `NAN` in the same
  conditions.
- `NAU7802::calibrateAfe` and `ADS1232::selfCalibrate` return `false`
  on timeout / cal-error.
- Driver `begin(...)` returns `false` on wiring or handshake failure.

---

## Threading / timing / hardware notes

- All time / delay calls go through `ungula::TimeControl` (project
  rule). Do not call `millis()` / `delay()` directly when extending or
  composing this library.
- Bit-bang drivers (HX711, ADS1232) take a critical section on ESP32
  (FreeRTOS `portMUX`) around the 24-bit read so an interrupt does not
  corrupt the clock pulse train. Read latency is therefore short and
  bounded but not interrupt-safe to call **from** an ISR. Call from
  task / loop context only.
- `readWithin(..., pollDelayMs)` cooperates: `0` yields via
  `TimeControl::yield()`, non-zero sleeps via `TimeControl::delayMs()`.
- All driver state is heap-free after `begin()`. `LoadCell` and
  `TensionSensor` allocate nothing at runtime.
- Wire structs / packing are not used here — there is no network layer.
- Concurrency: a single `IAdc24` instance is **not** thread-safe; gate
  it with a mutex if shared between tasks.

---

## Internals not part of the public API

Do not call or depend on these even though they are reachable in
headers:

- `HX711::shiftInByteMsbFirst`, `signExtend24`, `applyConfigSelection`,
  `readRawNow`, `waitReadyUntil`, `configToPulseCount` — wire-protocol
  helpers.
- `HX711::kClockPulseDelayUs`, `extraPulses_`, `discardNextSample_` —
  internal timing / state.
- `ADS1232::shiftInByteMsbFirst`, `signExtend24`, `clockPulse`,
  `waitReadyUntil`, `readRawNow`, `kClockPulseDelayUs` — same role.
- `ADS1220::sendCommand`, `writeRegister`, `readRegister`,
  `writeAllRegisters`, `waitReadyUntil`, `readConversionData`,
  `CMD_*`, `reg_[]` shadow registers.
- `NAU7802::writeReg`, `readReg`, `setBits`, `clearBits`, `readBit`,
  all `REG_*` / `PU_CTRL_*` / `CTRL2_*` constants.
- `LoadCell::kGravity`, `kNewtonToLbf` — internal constants; use
  `force::kgfToN` / `force::nToLbf` if you need the same numbers.
- `TensionSensor::updateFiltered`, `previousFilteredTension_`,
  `filteredTension_` storage — observe via `filteredTension()` /
  `lastVariation()` only.

If you need a behavior that is currently private, ask — do not paste
private logic into your code.

---

## LLM usage rules

- Use only the public API documented above. Do not include a driver's
  `.cpp` or call private helpers.
- Always go through `LoadCell` for calibrated reads. Only drop down to
  `IAdc24::readRaw*` for diagnostics (raw counts, signal-quality
  checks).
- Internal canonical unit is Newtons. Convert at the boundary via the
  `ForceUnit` parameter, not by hand.
- Always `captureZero` before applying a calibration factor; otherwise
  `calibrate(...)` will fold the offset into the scale.
- For SPI ADS1220 in `SINGLE_SHOT`, remember to call `startConversion()`
  after each `readRaw*`. In `CONTINUOUS` mode it is not needed.
- For NAU7802, call `calibrateAfe()` after every gain / channel /
  sample-rate change.
- Do not call `millis()` / `delay()` directly — use `TimeControl`.
- Do not allocate after `setup()`. None of the classes here need it.
- Preserve the project's terminology: `ForceUnit::NEWTONS / KGF / LBF`,
  `captureZero`, `countsPerNewton`, `readIfReady` / `readWithin`,
  `TensionSensor::Config`, `isAtTarget` / `isAboveTarget` /
  `isBelowTarget`.
