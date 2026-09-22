<h1 align="center">🧪 Esky Sampler</h1>
<h3 align="center">IoT-Enabled Automated Water Sampler</h3>

<p align="center">
  <em>Arduino firmware for a remote, solar-friendly peristaltic pump autosampler with cellular telemetry</em>
</p>

<p align="center">
  <a href="#overview">Overview</a> •
  <a href="#features">Features</a> •
  <a href="#hardware">Hardware</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#configuration">Configuration</a> •
  <a href="#getting-started">Getting Started</a> •
  <a href="#operation">Operation</a> •
  <a href="#power-management">Power Management</a>
</p>

---

## Overview

**Esky Sampler** is Arduino (AVR) firmware for an automated water-sampling station. A peristaltic pump draws precise volumes of water through a tube, rinses the intake, and deposits samples into a collection vessel — all controlled remotely via a cellular (LTE/CAT-M1) data link.

The system is designed for **unattended, long-duration field deployments** where mains power is unavailable. It spends most of its time in deep sleep (µA-level draw), wakes on a configurable interval, checks a remote server for instructions, executes sampling cycles, reports status, and goes back to sleep.

---

## Features

| Category | Details |
|----------|---------|
| 🔄 **Precision Pumping** | Hall-effect sensor counts pump-head revolutions for accurate mL delivery (`ml_per_rev` calibration) |
| 🌊 **Auto-Rinse** | XKC non-contact liquid-level sensor detects when water reaches the sample vessel; retries with reverse-purge if blocked |
| 📡 **Cellular Telemetry** | SIM7000 modem (LTE CAT-M1) uploads status, downloads configuration, and sends alerts via HTTP to a BOSL IoT server |
| 🔋 **Ultra-Low Power** | AVR `POWER_DOWN` deep sleep + clock-prescaling idle; draws single-digit µA between cycles |
| ⏰ **RTC Time Sync** | MCP7940 real-time clock synced to network UTC on every modem wake — keeps timestamps accurate across months |
| 🌐 **Remote Configuration** | Sampling parameters (volume, cycles, delay, pump speed) are pulled from the server each wake — no need to visit the site |
| 🛡️ **Robust Recovery** | Automatic modem hard-reset on repeated failures, zombie-UART detection, bearer-state-machine race-condition fixes, and rate-limited retry logic |
| 📊 **Alerting** | Sends coded alerts (rinse failure, target reached, errors) to the server for operator notification |

---

## Hardware

### Required Components

| Component | Role | Pin(s) |
|-----------|------|--------|
| **Arduino Mega 2560** (or compatible AVR board) | Main controller | — |
| **Peristaltic Pump** + motor driver | Water sampling | `D2` (REV), `D3` (FWD), `D13` (PWM speed), `A9` (SWA enable) |
| **Hall-Effect Sensor** | Revolution counting for volume measurement | `A9` |
| **XKC-Y25-T12V** non-contact liquid sensor | Water-level detection in sample vessel | `D4` |
| **SIM7000** LTE CAT-M1 module | Cellular data (Hologram SIM) | `TX1/RX1`, `D38` (PWRKEY), `D40` (RESET), `D45` (STATUS), `D25` (BUF_EN) |
| **MCP7940** RTC | Timekeeping during deep sleep | I²C |
| **Hologram SIM card** | Cellular connectivity (APN: `hologram`) | — |

### Pin Map

```
PUMP_FWD_PIN    = D3      // Pump forward drive
PUMP_REV_PIN    = D2      // Pump reverse drive
SPEED           = D13     // PWM motor speed
SWA             = A13     // Motor driver enable
XKC_SENSOR_PIN  = D4      // Liquid-level sensor
HALL_SENSOR_PIN = A9      // Hall-effect pump rotation
PWRKEY          = D38     // SIM7000 power key
SIM_RESET       = D40     // SIM7000 hardware reset
SIM_STATUS_BFD  = D45     // SIM7000 status readback
SIM_BUF_EN      = D25     // SIM7000 buffer enable
RTC_INTERRUPT   = D7      // MCP7940 alarm (future use)
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MAIN LOOP                                │
│                                                                 │
│  1. Wake from deep sleep                                        │
│  2. Power on SIM7000 → register on LTE network                  │
│  3. retrieveFromWeb() ─┬─ ReadState()                           │
│                        ├─ ReadMLSample()                        │
│                        ├─ ReadMLRev()         ┌──────────────┐  │
│                        ├─ ReadCycle()    ◄────│ BOSL IoT     │  │
│                        ├─ ReadTotalTargetVol() │ Web Server   │  │
│                        └─ ReadXDelay()        └──────────────┘  │
│  4. If state == 0 → sleep for xdelay seconds                    │
│  5. If state == 1 → run sampling cycle:                         │
│     a. Reverse purge (2× revolutions)                           │
│     b. Triple rinse with retry logic                            │
│     c. Forward pump (ml_per_sample / ml_per_rev revolutions)    │
│     d. Accumulate currentML                                     │
│     e. logToWeb() → upload status + alerts                      │
│  6. If currentML ≥ totalTargetVolume → permanent sleep          │
│  7. Deep sleep for remaining interval                           │
│  8. Repeat                                                      │
└─────────────────────────────────────────────────────────────────┘
```

### Startup Sequence (`setup()`)

1. Disable watchdog
2. Initialise all hardware pins
3. Power on SIM7000 → register on network → open data bearer
4. **Calibrate pump sensor** — `GetMinsMaxs()` spins the pump to measure Hall-effect min/max thresholds
5. **Calibrate revolutions** — `NoOfRevolutions()` counts revs until XKC triggers to establish the tube-fill baseline
6. Shut down modem to conserve power
7. Enter `loop()`

---

## Configuration

All configurable parameters are `#define` constants at the top of `esky_sampler_me.ino`:

### Pump & Sampling

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FWD` / `REV` | `255` | PWM duty (0–255) for forward/reverse pump speed |
| `ML_PER_REV` | `0.799` | Millilitres per pump-head revolution |
| `TARGET` | `500` | Total sample volume target (mL) |
| `CYCLES` | `10` | Number of sampling sub-cycles |
| `RETRIES` | `5` | Retry attempts on rinse failure |
| `DELAY` | `60` | Default inter-cycle delay (seconds) |

### Cellular & Network

| Parameter | Default | Description |
|-----------|---------|-------------|
| `APN` | `"hologram"` | SIM card APN |
| `MCCMNC` | `"302220"` | Preferred network (Telus Canada) |
| `ALLOW_GSM` | `false` | Enable legacy 2G fallback |
| `ALLOW_NBIOT` | `false` | Enable NB-IoT mode |
| `ACCEPT_NON_ROAMING` | `true` | Accept home-network registration |
| `LOG_TO_WEB` | `true` | Enable HTTP telemetry |

### Timing & Power

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LOOP_INTERVAL_MINUTES` | `1` | Sensor scan interval |
| `AVERAGING_INTERVAL_MINUTES` | `6` | Averaging/upload interval |
| `MAX_HTTP_INTERVAL_MINUTES` | `60` | Max time between uploads |

### IoT Server

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SITE_DIR` | `"microscape"` | Server directory path |
| `SITE_ID` | `"autosampler"` | Device identifier on server |

> **Remote overrides:** `state`, `ml_per_sample`, `ml_per_rev`, `cycle`, `totalTargetVolume`, and `xdelay` are all pulled from the BOSL server at each wake cycle, so you can reconfigure the device without a site visit.

---

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) 1.8+ or Arduino CLI
- Board package: **Arduino AVR Boards** (Mega 2560)
- Libraries:
  - [`MCP7940`](https://github.com/Zanduino/MCP7940) — RTC driver

### Upload

1. Open `esky_sampler_me.ino` in the Arduino IDE
2. Select **Board → Arduino Mega 2560**
3. Edit the `#define` constants for your deployment (APN, site IDs, pump calibration)
4. Upload to the board

### First Boot

On first power-on, the firmware will:

1. Calibrate the Hall-effect sensor thresholds (~2 seconds of spinning)
2. Calibrate the tube-fill revolution count (pumps until XKC triggers)
3. Attempt to register on the cellular network
4. Begin the main sampling loop

---

## Operation

### Remote Control via Server

The device reads its operating parameters from `http://www.bosl.com.au/IoT/<SITE_DIR>/scripts/ReadMe_v2.php`. The server-side CSV holds:

| Key | Type | Description |
|-----|------|-------------|
| `state` | `0` or `1` | `0` = idle (sleep), `1` = active (sample) |
| `ml_per_sample` | float | Volume per sub-cycle |
| `ml_per_rev` | float | Pump calibration |
| `cycle` | int | Number of sub-cycles |
| `totalTargetVolume` | float | Total target volume (mL) |
| `xdelay` | int | Seconds between server checks |

### Alert Codes

| Code | Meaning |
|------|---------|
| `0` | Heartbeat / normal status |
| `200` | Rinse failure — pump blocked or XKC didn't trigger |
| `500` | ml_per_sample read failure |
| `1000` | Target volume reached — device entering permanent sleep |

### Terminal States

The device enters **permanent deep sleep** (only a power cycle / battery change can recover) when:
- The total target volume has been reached (`currentML ≥ totalTargetVolume`)
- All rinse retries are exhausted (pump blockage)

---

## Power Management

The firmware uses two complementary low-power techniques, documented in detail in [`sleep_functions_explained.md`](sleep_functions_explained.md):

### `xDelay(ms)` — Clock-Prescaled Idle

For short waits (hundreds of ms to a few seconds):
- CPU clock divided by 64× (16 MHz → 250 kHz)
- Current draw drops from ~20 mA to ~1–2 mA
- `millis()` manually corrected after restoring full speed

### `deepSleepSecs(seconds)` — Watchdog-Based Power Down

For long inter-cycle sleeps (minutes to hours):
- AVR enters `SLEEP_MODE_PWR_DOWN` — CPU halted, all clocks stopped
- Current draw: **~4 µA** (plus RTC quiescent)
- Watchdog timer wakes CPU in 8s / 4s / 1s intervals
- ADC disabled during sleep, re-enabled on wake
- `millis()` manually corrected to maintain accurate timekeeping

```
deepSleepSecs(30)
│
├─ ADC off
├─ 8s loop (30 → 22 → 14 → 6)    ← 3 × 8s sleeps
├─ 4s loop (6 → 2)                ← 1 × 4s sleep
├─ 1s loop (2 → 1 → 0)            ← 2 × 1s sleeps
├─ ADC on
└─ Return (total: 30s ✓)
```

---

## Project Structure

```
esky_sampler_me/
├── esky_sampler_me.ino            # Main firmware (single-file Arduino sketch)
├── sleep_functions_explained.md   # Detailed walkthrough of power-management code
└── README.md                      # This file
```

---

## Contributing

Contributions are welcome! Some ideas:

- 📦 Extract reusable modules (SIM7000 driver, pump controller) into separate libraries
- 🧪 Add Hardware-in-the-Loop (HIL) tests with simulated sensor inputs
- 📈 Build a web dashboard for real-time sampling status
- 🔒 Add HTTPS/TLS support for secure telemetry
- 📍 Integrate GPS fix from SIM7000 GNSS module for location-tagged samples
- 💾 Add SD card logging for offline data backup

---

## Acknowledgements

- **[BOSL (Building Our Smarter Labs)](https://www.bosl.com.au/)** — IoT platform and server infrastructure
- **[BaniDB MCP7940 Library](https://github.com/Zanduino/MCP7940)** — RTC driver
- **SIMCom SIM7000 Series** — LTE CAT-M1 module AT command reference

---

<p align="center">
  <em>Built for field science 🌿 — deploy it, forget it, collect your samples.</em>
</p>
