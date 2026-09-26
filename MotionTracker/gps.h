#ifndef MOTIONTRACKER_GPS_H
#define MOTIONTRACKER_GPS_H

#include <TinyGPS++.h>

struct GpsFix
{
    double lat, lon, alt, kmph;
    int sats;
};

class gps
{
    public:
        void init();
        void feed();                  // non-blocking: consume whatever the GPS sent so far
        bool hasFix();                // fresh, reasonably accurate 3D fix
        GpsFix read();
        double distanceTo(double lat, double lon); // metres from the current fix

    private:
        TinyGPSPlus tGps;
};

#endif // MOTIONTRACKER_GPS_H
