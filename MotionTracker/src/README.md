# Bundled libraries

The Arduino IDE compiles everything in a sketch's `src/` folder, so the libraries below are built together
with MotionTracker. Nothing needs to be installed with the Library Manager. Libraries you have installed
yourself are not used by this sketch, and their settings don't affect it.

| Folder | Library | Version | Source | Licence |
|--------|---------|---------|--------|---------|
| `MCCI_LMIC` | MCCI LoRaWAN LMIC library | 6.0.1 (`e4c3711`) | [mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic) | MIT (see `MCCI_LMIC/LICENSE`) |
| `TinyGPSPlus` | TinyGPSPlus | 1.1.0 (`0a20575`) | [mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) | LGPL 2.1 or later (see the header of `TinyGPS++.h`) |
| `XPowersLib` | XPowersLib | 0.3.4 (`d699758`) | [lewisxhe/XPowersLib](https://github.com/lewisxhe/XPowersLib) | MIT (see `XPowersLib/LICENSE`) |
| `Adafruit_BME280` | Adafruit BME280 Library | 2.3.0 (`ef712e9`) | [adafruit/Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library) | BSD (see `Adafruit_BME280/LICENSE`) |
| `Adafruit_Sensor` | Adafruit Unified Sensor | 1.1.15 (`0a9127a`) | [adafruit/Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor) | Apache 2.0 (see `Adafruit_Sensor/LICENSE.txt`) |
| `Adafruit_BusIO` | Adafruit BusIO | 1.17.4 (`3b83642`) | [adafruit/Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO) | MIT (see `Adafruit_BusIO/LICENSE`) |

Only the source files and licences are included, not the examples. The Adafruit libraries are only used
when `USE_BME280` is 1; otherwise the linker leaves them out.

## Changes compared to the originals

* `#include` lines between these libraries are rewritten to relative paths (for example
  `#include "../Adafruit_BusIO/Adafruit_I2CDevice.h"`), because a sketch's `src/` folder isn't on the
  compiler's include path. The code itself is unchanged.
* `MCCI_LMIC/project_config/lmic_project_config.h` is set to **EU868** (`#define CFG_eu868 1`) instead of
  the library's default US915.

The Cayenne LPP payload is built by `lpp.h` in the sketch folder, so the CayenneLPP and ArduinoJson
libraries aren't needed.

## Changing the LoRaWAN region

Edit `MCCI_LMIC/project_config/lmic_project_config.h` in this folder: comment out `#define CFG_eu868 1` and
enable your region, for example `#define CFG_us915 1`. Then set `REQUIRE_EU868` to 0 in `config.h`.

## Updating a library

`tools/vendor_libraries.py` in the repository root copies the libraries from an Arduino `libraries`
folder into this folder again and rewrites the includes:

```
python3 tools/vendor_libraries.py ~/Arduino/libraries MotionTracker
```

It expects the library folder names used by `git clone` (`arduino-lmic`, `TinyGPSPlus`, `XPowersLib`,
`Adafruit_BME280_Library`, `Adafruit_Sensor`, `Adafruit_BusIO`). Check the region setting in
`MCCI_LMIC/project_config/lmic_project_config.h` afterwards, and update the versions in this table.
