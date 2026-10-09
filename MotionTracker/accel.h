#ifndef MOTIONTRACKER_ACCEL_H
#define MOTIONTRACKER_ACCEL_H

#include <Arduino.h>

// Minimal register-level driver for a motion-detecting accelerometer.
// The sensor raises a latched, active-high interrupt on ACCEL_INT_PIN when it
// detects motion; that pin both wakes the ESP32 from deep sleep (ext0) and is
// polled while awake.
class Accel
{
    public:
        enum Chip { NONE, LIS3DH, MPU6050 };

        // Detect the sensor, false if none found. configure=false keeps the settings it
        // kept while the ESP32 was in deep sleep (reconfiguring can fire a false motion event).
        bool begin(bool configure);
        bool present() const { return chip != NONE; }
        const char* name() const;

        // Returns true if motion was latched since the last call, and clears the latch.
        bool motionDetected();

        // Clear any pending latch so the INT line is low before entering deep sleep.
        void clear();

    private:
        Chip chip = NONE;
        uint8_t addr = 0;

        bool probe(uint8_t address, uint8_t whoAmIReg, uint8_t expected);
        void write(uint8_t reg, uint8_t value);
        uint8_t read(uint8_t reg);
        void setupLis3dh();
        void setupMpu6050();
};

#endif // MOTIONTRACKER_ACCEL_H
