#include <HardwareSerial.h>
#include "config.h"
#include "gps.h"

HardwareSerial GPSSerial(1);

void gps::init()
{
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void gps::feed()
{
  while (GPSSerial.available()) {
    tGps.encode(GPSSerial.read());
  }
}

bool gps::hasFix()
{
  return tGps.location.isValid() &&
         tGps.location.age() < 2000 &&
         tGps.hdop.isValid() &&
         tGps.hdop.value() <= 300 &&
         tGps.altitude.isValid();
}

GpsFix gps::read()
{
  GpsFix fix;
  fix.lat = tGps.location.lat();
  fix.lon = tGps.location.lng();
  fix.alt = tGps.altitude.meters();
  fix.kmph = tGps.speed.isValid() ? tGps.speed.kmph() : 0.0;
  fix.sats = tGps.satellites.value();
  return fix;
}

double gps::distanceTo(double lat, double lon)
{
  return TinyGPSPlus::distanceBetween(tGps.location.lat(), tGps.location.lng(), lat, lon);
}
