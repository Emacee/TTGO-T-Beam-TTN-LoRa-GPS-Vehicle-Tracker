#ifndef MOTIONTRACKER_POWER_H
#define MOTIONTRACKER_POWER_H

#include <Arduino.h>

// Wraps the T-Beam power management chip: AXP192 (T-Beam v1.0/v1.1) or
// AXP2101 (T-Beam v1.2), detected automatically.
namespace power
{
    bool begin();                 // enables LoRa, GPS and 3V3 header rails
    const char* pmuName();
    void gps(bool on);            // switch the GPS supply rail
    float batteryVoltage();       // volts, 0 when unknown
    void led(bool on);            // charge LED (AXP192 only; no-op on AXP2101)
    void prepareSleep();          // LoRa + GPS off; header 3V3 (accelerometer) stays on
}

#endif // MOTIONTRACKER_POWER_H
