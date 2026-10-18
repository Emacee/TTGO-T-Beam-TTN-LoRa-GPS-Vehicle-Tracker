#include "config.h"
#include "climate.h"

#if USE_BME280

#include <Wire.h>
#include <Adafruit_BME280.h>

static Adafruit_BME280 bme;
static bool found = false;

bool climate::begin()
{
  found = bme.begin(BME280_ADDRESS, &Wire);
  if (!found) return false;
  // Forced mode: the sensor sleeps (~0.1 uA) and only measures when asked,
  // so it can stay powered through deep sleep.
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,  // temperature
                  Adafruit_BME280::SAMPLING_X1,  // pressure
                  Adafruit_BME280::SAMPLING_X1,  // humidity
                  Adafruit_BME280::FILTER_OFF);
  return true;
}

void climate::addToPayload(CayenneLPP& lpp)
{
  if (!found || !bme.takeForcedMeasurement()) return;
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0F;
  lpp.addTemperature(2, t);
  lpp.addRelativeHumidity(3, h);
  lpp.addBarometricPressure(4, p);
  Serial.printf("BME280: %.1f C, %.0f %%, %.1f hPa\n", t, h, p);
}

#else

bool climate::begin() { return false; }
void climate::addToPayload(CayenneLPP&) {}

#endif
