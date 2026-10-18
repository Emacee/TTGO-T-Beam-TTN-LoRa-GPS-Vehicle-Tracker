#ifndef MOTIONTRACKER_LPP_H
#define MOTIONTRACKER_LPP_H

#include <stdint.h>
#include <math.h>

// Minimal Cayenne Low Power Payload encoder for the fields this tracker sends.
// Same byte format as the CayenneLPP library (and TTN's CayenneLPP formatter):
// per field one channel byte, one type byte, then the value big-endian.
class LppEncoder
{
    public:
        explicit LppEncoder(uint8_t maxSize) : max(maxSize > sizeof(buf) ? sizeof(buf) : maxSize) {}

        void reset() { len = 0; }
        uint8_t* getBuffer() { return buf; }
        uint8_t getSize() const { return len; }

        void addDigitalInput(uint8_t ch, uint8_t value)    { if (start(ch, 0, 1)) put(value, 1); }
        void addAnalogInput(uint8_t ch, float value)       { if (start(ch, 2, 2)) put(lround(value * 100), 2); }
        void addTemperature(uint8_t ch, float celsius)     { if (start(ch, 103, 2)) put(lround(celsius * 10), 2); }
        void addRelativeHumidity(uint8_t ch, float pct)    { if (start(ch, 104, 1)) put(lround(pct * 2), 1); }
        void addBarometricPressure(uint8_t ch, float hpa)  { if (start(ch, 115, 2)) put(lround(hpa * 10), 2); }
        void addGPS(uint8_t ch, float lat, float lon, float meters)
        {
            if (!start(ch, 136, 9)) return;
            put(lround(lat * 10000), 3);
            put(lround(lon * 10000), 3);
            put(lround(meters * 100), 3);
        }

    private:
        uint8_t buf[51];
        uint8_t max;
        uint8_t len = 0;

        bool start(uint8_t ch, uint8_t type, uint8_t size)
        {
            if (len + 2 + size > max) return false;
            buf[len++] = ch;
            buf[len++] = type;
            return true;
        }

        void put(long value, uint8_t bytes)
        {
            for (int i = bytes - 1; i >= 0; i--) buf[len++] = (uint8_t)(value >> (8 * i));
        }
};

#endif // MOTIONTRACKER_LPP_H
