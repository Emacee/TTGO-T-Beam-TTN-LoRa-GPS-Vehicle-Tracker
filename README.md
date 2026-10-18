# TTGO T-Beam TTN LoRa GPS Vehicle Tracker

A GPS vehicle tracker for the **[TTGO T-Beam](https://github.com/LilyGO/TTGO-T-Beam)** (ESP32 + GPS + LoRa) that reports
over **The Things Network (TTN)** using LoRaWAN OTAA. It sends often while the vehicle moves, and sleeps
while it's parked, waking up for a heartbeat or as soon as an accelerometer feels motion.

The firmware is in [`MotionTracker/`](MotionTracker). The original fixed-rate sketches are still in the
repository root; see [Original sketches](#original-sketches).

## How it works

| State | What the tracker does | Uplink interval |
|-------|-----------------------|-----------------|
| **MOVING** | GPS on, sends position, speed, battery | 30–120 s (the faster it goes, the shorter the interval) |
| **STOPPED** | No motion from the accelerometer *and* GPS speed below 5 km/h for `STOP_TIMEOUT_S` (180 s) | One final "stopped" uplink, then deep sleep |
| **PARKED** | Deep sleep, LoRa radio and GPS powered off | Heartbeat every `HEARTBEAT_S` (default 600 s, pick 300–900 s) |

The tracker wakes from deep sleep when:

* the **accelerometer detects motion**: it powers up the GPS and goes to MOVING,
* the **heartbeat timer** runs out: it sends the last known position and goes back to sleep in a few seconds,
* the **middle user button** (GPIO 38) is pressed: it sends an uplink straight away (with `WAKE_ON_VBUS` the
  short press of the PWR button does this instead),
* **USB power comes on**, for example when the ignition switches on a USB port (optional, `WAKE_ON_VBUS`).

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

| Part | Needed | Notes |
|------|--------|-------|
| **TTGO T-Beam** v1.0, v1.1 (AXP192) or v1.2 (AXP2101) | Yes | Detected automatically |
| **LoRa antenna** and GPS antenna | Yes | Never transmit without the LoRa antenna |
| **Accelerometer**: LIS3DH or MPU-6050 | Optional, recommended | Wakes the tracker on motion; the T-Beam has none on board. See [Without an accelerometer](#without-an-accelerometer) |
| **BME280** temperature / humidity / pressure | Optional | Off by default, see [Optional BME280](#optional-bme280) |
| 18650 Li-ion cell | For use without USB | Goes in the holder on the back |

![TTGO-T-Beam](images/ttgo-t-beam.jpg)

### Accelerometer

The firmware detects which of these two is connected:

| Sensor | Sleep current | Notes |
|--------|---------------|-------|
| **LIS3DH** (recommended) | ~6 µA | Adafruit 2809, or generic LIS3DH boards |
| **MPU-6050 / GY-521** | ~20–70 µA | Cheap and common. It uses more power in sleep. |

Wire its interrupt pin to GPIO 13 (see [Wiring](#wiring)). If you don't fit one, set `USE_ACCEL 0` in
`config.h`, or leave it at 1: a missing sensor is detected and the tracker falls back to GPS.

### Without an accelerometer

The tracker works without an accelerometer, but it needs another way to notice that the vehicle starts moving:

| | With accelerometer | GPS only | GPS + USB power wake-up |
|---|---|---|---|
| Notices the vehicle starts moving | Immediately | At the next heartbeat (up to `HEARTBEAT_S`) | Immediately when the ignition comes on |
| Notices towing or theft without ignition | Immediately | At the next heartbeat | At the next heartbeat |
| Heartbeat while parked | A few seconds, GPS off | GPS on, up to `HEARTBEAT_GPS_WAIT_S` (45 s) | Same as GPS only |
| Extra hardware | Sensor + 5 wires | None | None, but needs a USB port that switches with the ignition |

**GPS only** (`USE_ACCEL 0`): at every heartbeat the tracker powers the GPS and switches to MOVING if it is
more than 100 m from the last position or GPS shows speed.

**USB power wake-up** (`WAKE_ON_VBUS 1`): power the T-Beam from a USB port that switches on with the
ignition, and keep the 18650 cell in the holder for when the ignition is off. The power chip on the T-Beam
then wakes the ESP32 (through GPIO 35) as soon as USB power appears. While USB power is present the
tracker stays in MOVING mode; after the ignition goes off it waits `STOP_TIMEOUT_S`, sends "stopped" and
goes to sleep. This also works together with an accelerometer.

Things to know about `WAKE_ON_VBUS`:

* With a USB supply that is **always on**, the tracker never sleeps. Only enable it on a switched port.
* For the same reason, test sleep on battery: unplug the USB cable after flashing.
* The middle button can no longer wake the tracker. A **short press of the PWR button** does that instead
  (the ESP32 has only one wake-up channel left for both).
* The power chip's interrupt line (GPIO 35) needs a pull-up resistor. The T-Beam should have one on board,
  but this hasn't been checked on real hardware yet. If the tracker prints `PMU interrupt line stuck low`
  before sleeping, the line stays low on your board and USB wake-up is switched off for that sleep.

### Optional BME280

The BME280 reports the temperature, humidity and air pressure inside the vehicle. Its code is a separate
module (`MotionTracker/climate.cpp`) that is **off by default**. With it off, the firmware doesn't use or
need the BME280 library, and you can leave the sensor out entirely.

* **Turn it on:** set `#define USE_BME280 1` in `MotionTracker/config.h` and install the
  **Adafruit BME280 Library** in the Arduino IDE (accept its dependencies).
* **Address:** most cheap boards use `0x76`; Adafruit boards use `0x77`. Set it with `BME280_ADDRESS`.
* **BME280, not BMP280:** the two look almost the same, but the BMP280 has no humidity sensor and isn't
  supported. The chip on a BME280 is square; on a BMP280 it is rectangular. With a BMP280 (or no sensor), the
  tracker logs `USE_BME280 is on but no BME280 found` and sends its uplinks without climate data.
* **Power:** the sensor runs in forced mode. It draws about 0.1 µA between readings, so it can stay powered
  in deep sleep. One reading is taken for every uplink.
* **Payload:** temperature, humidity and pressure go on channels 2, 3 and 4.
* The old BME280-I2C-ESP32 library from the original sketches isn't needed. MotionTracker uses the standard
  Adafruit library.

![BME280 module](images/bmp280.jpg)

### Wiring

The accelerometer and the BME280 share the I2C bus on the T-Beam header:

```
Sensor pin       T-Beam
VCC / VIN   ->   3V3   (stays powered during deep sleep)
GND         ->   GND
SDA         ->   21
SCL         ->   22
INT / INT1  ->   13    (accelerometer only; ACCEL_INT_PIN in config.h, must be an RTC GPIO)
```

Most sensor boards already have I2C pull-up resistors. With long wires, add 2.2k–10k pull-ups on SDA and SCL.

## Installing on a TTGO T-Beam

> **Always screw the LoRa antenna on before powering the board.** Transmitting without an antenna can
> damage the radio. The GPS antenna (the small ceramic patch or the U.FL cable) should also be connected.

### 1. Find out which board you have

Look at the chip next to the battery holder, or check the print on the board:

* **v1.0 / v1.1**: AXP192 power chip, CP2104 USB chip.
* **v1.2**: AXP2101 power chip, CH9102 USB chip.

The firmware detects both. You only need to know this for the USB driver.

### 2. Register the device on The Things Network

1. Sign in to the [TTN console](https://console.cloud.thethings.network/) and pick your cluster
   (Europe 1 for the Netherlands).
2. **Applications → Create application**, give it an ID (for example `my-tracker`).
3. **Register end device → Enter end device specifics manually**:
   * Frequency plan: **Europe 863-870 MHz (SF9 for RX2 - recommended)** (or the plan for your region)
   * LoRaWAN version: **LoRaWAN Specification 1.0.3**
   * JoinEUI: all zeros is fine (`00 00 00 00 00 00 00 00`), then **Confirm**
   * DevEUI and AppKey: click **Generate** for both
4. Open the device page. For **DevEUI** and **JoinEUI**, click the `<>` button, switch to **lsb**, and copy the
   `{ 0x.., ... }` arrays into `LORAWAN_DEVEUI` and `LORAWAN_APPEUI` in `MotionTracker/config.h`.
   For **AppKey**, use `<>` with **msb** and paste it into `LORAWAN_APPKEY`.
5. **Payload formatters → Uplink → Formatter type: CayenneLPP → Save.** TTN then shows the decoded values
   (location, battery, speed, state) for every message.

### 3. Set up the Arduino IDE

1. Install [Arduino IDE 2](https://www.arduino.cc/en/software).
2. **File → Preferences → Additional boards manager URLs**, add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
3. **Tools → Board → Boards Manager**, search for **esp32** and install **esp32 by Espressif Systems**.
   Versions 2.0.17 and 3.3.x are tested.
4. **Tools → Manage Libraries**, install:
   * **MCCI LoRaWAN LMIC library** (by IBM, Matthijs Kooijman, Terry Moore, ...)
   * **TinyGPSPlus** (by Mikal Hart)
   * **CayenneLPP** (by Electronic Cats). Click **Install all** to add ArduinoJson as well.
   * **XPowersLib** (by Lewis He)
   * **Adafruit BME280 Library**, only if you use the BME280. Click **Install all**.
5. **Set the LoRaWAN region.** The LMIC library has its own settings file and defaults to the US (915 MHz).
   Open it in a text editor:
   * Windows: `Documents\Arduino\libraries\MCCI_LoRaWAN_LMIC_library\project_config\lmic_project_config.h`
   * macOS: `~/Documents/Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`
   * Linux: `~/Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`

   Change the first lines to:
   ```c
   #define CFG_eu868 1
   //#define CFG_us915 1
   ```
   Updating the library overwrites this file, so repeat this step after every LMIC update. If you forget,
   the build stops with `The MCCI LMIC library is not set to EU868`. Outside Europe, pick your region
   here and set `REQUIRE_EU868` to 0 in `config.h`.

### 4. Configure, build and flash

1. Download this repository (**Code → Download ZIP** on GitHub) and unzip it.
2. In the Arduino IDE, **File → Open** `MotionTracker/MotionTracker.ino`. All the other files open as tabs.
3. In the `config.h` tab:
   * paste your TTN keys (step 2),
   * set `USE_ACCEL`, `WAKE_ON_VBUS` and `USE_BME280` for your hardware (see [Configuration](#configuration-configh)).
4. **Tools → Board → esp32 → T-Beam.** The other Tools settings can stay at their defaults.
5. Install the USB driver if your computer doesn't see a new serial port: **CP210x** (Silicon Labs) for
   v1.0/v1.1, **CH9102/CH343** (WCH) for v1.2. Recent macOS and Linux usually need nothing.
6. Connect the T-Beam with a USB **data** cable (not a charge-only cable). Switch it on with the PWR button
   if needed.
7. **Tools → Port** → the new serial port (`COMx` on Windows, `/dev/cu.usbserial-...` or `/dev/cu.wchusbserial...`
   on macOS, `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux).
8. Click **Upload** (→). The first build takes a few minutes.

   If the upload hangs at `Connecting...`, hold the **BOOT** button (GPIO 0, on the side) until the upload starts.

The `platformio.ini` in the repository is for PlatformIO users; the Arduino IDE ignores it.

### 5. Check that it works

1. Open **Tools → Serial Monitor** and set it to **115200 baud**. Press the RST button. You should see:
   ```
   MotionTracker boot #1, PMU AXP192, accel LIS3DH, USB power yes, woken by power-up
   State -> MOVING
   EV_JOINING
   EV_JOINED, devaddr=0x260B....
   Uplink queued: state=1 fix=1 ...
   EV_TXCOMPLETE
   ```
2. In the TTN console, the device's **Live data** tab shows the join and the uplinks.
3. The first GPS fix can take a few minutes outdoors. Indoors it may not come at all; the tracker then
   sends without position after `GPS_FIX_WAIT_S`.
4. Leave it still for `STOP_TIMEOUT_S` (3 minutes). It sends a "stopped" message (state 2) and prints
   `Deep sleep for up to 600 s`. Shake it and it wakes up straight away (`woken by motion`).
   * With `WAKE_ON_VBUS 1`, unplug USB first, otherwise it doesn't sleep. Plug USB back in to wake it
     (`woken by USB power`).
   * Without an accelerometer, it only wakes at the heartbeat (`woken by heartbeat`).

The serial monitor disconnects while the tracker sleeps; that's normal.

For desk testing, uncomment `#define DEBUG` in `config.h` to get shorter intervals.

## Configuration: `config.h`

* `LORAWAN_DEVEUI`, `LORAWAN_APPEUI` (LSB first) and `LORAWAN_APPKEY` (MSB first), from the TTN console.
* `MOVING_TX_MIN_S` / `MOVING_TX_MAX_S`: the interval range while moving (default 30/120 s).
* `HEARTBEAT_S`: the heartbeat interval while parked (default 600 s).
* `STOP_TIMEOUT_S`: how long without motion before the tracker counts as parked (default 180 s).
* `LIS3DH_MOTION_THRESHOLD` / `MPU6050_MOTION_THRESHOLD`: wake-up sensitivity. Raise them if wind or
  passing trucks wake the tracker.
* `HEARTBEAT_REFRESH_GPS`: set to 1 to take a new fix on every heartbeat instead of sending the cached
  position.
* `USE_ACCEL`: 1 to use an accelerometer if one is connected, 0 for GPS-only motion detection.
* `WAKE_ON_VBUS`: 1 to wake up and stay in MOVING mode while USB power is present (ignition-switched USB), 0 to ignore USB power.
* `USE_BME280`: 1 to include the optional BME280 module, 0 to leave it out.
* `REQUIRE_EU868`: stops the build if the LMIC library isn't set to EU868. Set it to 0 for other regions.
* `#define DEBUG`: short intervals for testing on the desk.

## Payload (Cayenne LPP, port 1)

The channels are the same as in the original sketches, so existing dashboards keep working. In the TTN
console, set **Payload formatters → Uplink** to **CayenneLPP** to decode them.

| Channel | Type | Content |
|---------|------|---------|
| 1 | GPS | latitude, longitude, altitude. Left out when moving without a fix. |
| 2 | Temperature | °C (only with `USE_BME280`) |
| 3 | Humidity | % (only with `USE_BME280`) |
| 4 | Barometer | hPa (only with `USE_BME280`) |
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

### Home Assistant and ESPHome

ESPHome can't be used for this tracker: it has no LoRaWAN support, so it can't join TTN. To get the data
into Home Assistant, keep this firmware and use Home Assistant's **The Things Network** integration, or TTN's
MQTT server with the MQTT integration.

## Original sketches

These are the original fixed-rate versions, kept for reference. They don't sleep.

* `TTGO-T-Beam-TTN-LoRa-GPS-Vehicle-Tracker.ino`: T-Beam v1.1 (AXP192). It reads a BME280 if one is
  connected, but the lines that add the climate values to the payload are commented out, so they aren't
  sent. Building it needs the [BME280-I2C-ESP32](https://github.com/Takatsuki0204/BME280-I2C-ESP32) library;
  uninstall the Adafruit BME280 library while using it, because the two conflict.
* `TTGO-T-Beam-TTN-LoRa-GPS-Vehicle-Trackerv07.ino`: T-Beam v0.7, without the BME280.

Both send every 10–15 minutes, or every 30–60 s when the last GPS speed was above 3.5–4 km/h. The keys go in
the `.ino` file itself.

The Arduino IDE builds every `.ino` file in a folder into one program, so copy the sketch you want into its
own folder, together with `gps.h` and `gps.cpp`, before you build it.

## TTGO T-Beam specifications

```
ESP32
  ESP32 Version REV1
  WiFi
  Bluetooth 
  4MB Flash
  3D Antenna
 
LORA
  Working voltage:      1.8 ~ 3.7v
  Acceptable current:   10 ~ 14mA
  Transmit current:     120mA @ +20dBm
                         90mA @ +17dBm
                         29mA @ +13dBm
  Operating frequency:  433MHz / 868MHz / 915MHz
  Transmit power:       +20dBm
  Receive sensitivity:  -139dBm @ LoRa &  62.5 KHz & SF=12 &  146bps
                        -136dBm @ LoRa & 125 KHz   & SF=12 &  293bps
                        -118dBm @ LoRa & 125 KHz   & SF=6  & 9380bps
                        -123dBm @ FSK  &   5 KHz   & 1.2Kbps
  Frequency error:       +/-15KHz
  FIFO space:            64 byte
  Data rate:             1.2K ~ 300Kbps @ FSK
                         0.018K ~ 37.5Kbps @ LoRa        
  Modulation Mode:       FSK, GFSK, MSK, GMSK, LoRa TM, OOK
  Interface form:        SPI
  Sleep current:         0.2uA @ SLEEP
                         1.5uA @ IDLE
  Operating temperature: -40? - +85?
  Digital RSSI function
  Automatic frequency correction
  Automatic gain control 
  RF wake-up function
  Low voltage detection and temperature sensor
  Fast wake-up and frequency hopping
  Highly configurable data packet handler

GPS
  GPS modules NEO-6M, 3V-5V power supply Universal
  Destined module with ceramic antenna, signal super
  Save the configuration parameter data EEPROM Down
  With data backup battery
  There are LED signal indicator
  Default Baud Rate: 9600

Power
  IP5306 2A Battery PMIC
  LED, Blue - User controller
  LED, Red - GPS 1PPS
  LED, Red/green - battery charged/power on
  Button, reset switch
  Button, user readable
  Switch, power on/battery charge
  USB
  CP2104-GMR
```

These specifications are for the early T-Beam. On v1.0 and later, the IP5306 is replaced by the AXP192
(AXP2101 on v1.2), and the GPS is usually a NEO-M8N.

## TODO

- Debug TTN downlink messages, there are a lot of frames down. Seems to come from the library but need to test this first
- Test MotionTracker on hardware and tune the default motion thresholds

## Credits

- Thanks to [Tekk](https://github.com/tekk) for his [TTGO-T-Beam-Car-Tracker](https://github.com/tekk/TTGO-T-Beam-Car-Tracker) which came as an inspiration for this project
- Thanks [Matthijs Kooijman](https://github.com/matthijskooijman/arduino-lmic/tree/master/examples/ttn-otaa) for his OTAA example for The Things Network
- Thanks to [DeuxVis](https://github.com/DeuxVis) for his [Lora-TTNMapper-T-Beam](https://github.com/DeuxVis/Lora-TTNMapper-T-Beam) which formed inspiration for Tekk's repository
- GPS power management based on the code provided by [LilyGO](https://github.com/LilyGO/TTGO-T-Beam/blob/master/GPS-T22_v1.0-20190612/GPS-T22_v1.0-20190612.ino)

## Further improvement

You are welcome to contribute to this project in any way. (Submit an issue, bug report, fork and improve...)
Suggestions & feedback is much appreciated.
