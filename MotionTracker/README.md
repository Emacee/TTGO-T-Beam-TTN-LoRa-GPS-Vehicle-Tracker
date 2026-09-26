# MotionTracker: TTGO T-Beam tracker that reports on motion

This firmware replaces the fixed-rate reporting of the original sketch with motion-triggered reporting:

| State | What the tracker does | Uplink interval |
|-------|-----------------------|-----------------|
| **MOVING** | GPS on, sends position, speed, battery | 30–120 s (the faster it goes, the shorter the interval) |
| **STOPPED** | No motion from the accelerometer *and* GPS speed below 5 km/h for `STOP_TIMEOUT_S` (180 s) | One final "stopped" uplink, then deep sleep |
| **PARKED** | Deep sleep, LoRa radio and GPS powered off | Heartbeat every `HEARTBEAT_S` (default 600 s, pick 300–900 s) |

The tracker wakes from deep sleep when:

* the **accelerometer detects motion**: it powers up the GPS and goes to MOVING,
* the **heartbeat timer** runs out: it sends the last known position and goes back to sleep in a few seconds,
* the **middle user button** (GPIO 38) is pressed: it sends an uplink straight away.

A short bump, such as a door being closed, wakes the tracker but doesn't make it send anything. The first
MOVING uplink only goes out when GPS speed confirms the movement, or when the accelerometer still reports
motion after `GPS_FIX_WAIT_S`. Otherwise the tracker goes back to sleep once `STOP_TIMEOUT_S` has passed.

## What changed compared to the original sketch

The original `TTGO-T-Beam-TTN-LoRa-GPS-Vehicle-Tracker.ino` has these problems, all fixed here:

1. **It never sleeps.** GPS and LoRa stay on all the time, sending every 600 s while parked. `esp_sleep.h` is
   included but never used.
2. **The send rate follows the speed from the previous packet.** The next interval is picked in
   `EV_TXCOMPLETE` from the last `kmph`, so a car that starts moving waits up to 10 minutes before the
   first "moving" packet.
3. **Without a GPS fix, nothing is sent.** `do_send()` retries every 15 s indefinitely.
4. **The scheduling chain can die.** If `do_send()` finds `OP_TXRXPEND`, it returns without scheduling
   another send, so the tracker stops reporting.
5. **LMIC settings are applied before `os_init()`/`LMIC_reset()`**, which wipes them. `LMIC_setDrTxpow`,
   `LMIC_setLinkCheckMode` and the frame counter restore have no effect.
6. **The session keys are saved but never restored**, so every reboot means a new OTAA join.
7. **GPS reading blocks for 1 s** inside the LMIC job, and the tracker needs an AXP192. It doesn't run on the
   T-Beam v1.2, which has an AXP2101.

## Hardware

The TTGO T-Beam (v1.0, v1.1 with AXP192, or v1.2 with AXP2101) **has no accelerometer on board**. You need to add one on the
I2C header. The driver detects which of these two you connected:

| Sensor | Sleep current | Notes |
|--------|---------------|-------|
| **LIS3DH** (recommended) | ~6 µA | Adafruit 2809, or generic LIS3DH boards |
| **MPU-6050 / GY-521** | ~20–70 µA | Cheap and common. It uses more power in sleep. |

Wiring:

```
Accelerometer    T-Beam
VCC / VIN   ->   3V3   (stays powered during deep sleep)
GND         ->   GND
SDA         ->   21
SCL         ->   22
INT / INT1  ->   13    (ACCEL_INT_PIN in config.h, must be an RTC GPIO)
```

If no accelerometer is found, the tracker still works: it wakes on the heartbeat timer, takes a GPS fix and
switches to MOVING if it has moved more than 100 m. Motion is then only noticed at the next heartbeat.

## Configuration: `config.h`

* `LORAWAN_DEVEUI`, `LORAWAN_APPEUI` (LSB first) and `LORAWAN_APPKEY` (MSB first), from the TTN console.
* `MOVING_TX_MIN_S` / `MOVING_TX_MAX_S`: the interval range while moving (default 30/120 s).
* `HEARTBEAT_S`: the heartbeat interval while parked (default 600 s).
* `STOP_TIMEOUT_S`: how long without motion before the tracker counts as parked (default 180 s).
* `LIS3DH_MOTION_THRESHOLD` / `MPU6050_MOTION_THRESHOLD`: wake-up sensitivity. Raise them if wind or
  passing trucks wake the tracker.
* `HEARTBEAT_REFRESH_GPS`: set to 1 to take a new fix on every heartbeat instead of sending the cached
  position.
* `#define DEBUG`: short intervals for testing on the desk.

## Building

### PlatformIO

`platformio.ini` sits in the repository root. Choose your LoRaWAN region in `build_flags` (EU868 is the default), then run:

```
pio run -t upload && pio device monitor
```

### Arduino IDE

1. Install the ESP32 board package and select **T-Beam** as the board.
2. Install these libraries: **MCCI LoRaWAN LMIC library**, **TinyGPSPlus**, **CayenneLPP** (it pulls in ArduinoJson), and **XPowersLib**.
3. Select your region in `Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`,
   for example `#define CFG_eu868 1`, and comment out `CFG_us915`.
4. Open `MotionTracker/MotionTracker.ino` and upload it.

## Payload (Cayenne LPP, port 1)

The channels match the original sketch, so existing Cayenne and TTN dashboards keep working:

| Channel | Type | Content |
|---------|------|---------|
| 1 | GPS | latitude, longitude, altitude. Left out when moving without a fix. |
| 5 | Analog | battery voltage (V) |
| 6 | Analog | speed (km/h) |
| 7 | Analog | satellites in use |
| 8 | Digital | **new:** 0 = parked heartbeat, 1 = moving, 2 = just stopped |

## Notes

* The LoRaWAN session (keys, frame counters, channels and duty-cycle timers) is kept in RTC memory. The
  tracker only joins after power-up or a reflash, not on every wake.
* ADR is disabled and the data rate is fixed (SF7 by default, `LORAWAN_DATARATE`), which suits a device that moves.
* **TTN fair use policy:** 30 s of airtime per device per day. At SF7 one uplink of this size takes about 70 ms, so that is roughly
  430 uplinks per day. Driving for hours at 30 s intervals will go over it. Raise `MOVING_TX_MIN_S` if you
  drive a lot.
