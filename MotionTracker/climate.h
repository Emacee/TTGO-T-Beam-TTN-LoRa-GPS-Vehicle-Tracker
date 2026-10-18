#ifndef MOTIONTRACKER_CLIMATE_H
#define MOTIONTRACKER_CLIMATE_H

#include <CayenneLPP.h>

// Optional BME280 temperature / humidity / pressure sensor.
// Enabled with USE_BME280 in config.h. When disabled, these functions do
// nothing and the Adafruit BME280 library is not needed to build.
namespace climate
{
    bool begin();                 // false when disabled or no sensor found
    void addToPayload(CayenneLPP& lpp);
}

#endif // MOTIONTRACKER_CLIMATE_H
