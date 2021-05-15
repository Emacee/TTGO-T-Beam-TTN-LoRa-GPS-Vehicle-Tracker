/*******************************************************************************
 * Based on the LMIC example by 2015 Thomas Telkamp and Matthijs Kooijman Copyright (c)
 *
 * This uses OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 * 
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <esp_sleep.h>


#include "gps.h"

// #define DEBUG 1 // for real use comment this out

#define BUILTIN_LED 14 // T-Beam blue LED, see: http://tinymicros.com/wiki/TTGO_T-Beam
#define BATTERY_PIN 35 // battery level measurement pin, here is the voltage divider connected

CayenneLPP lpp(51); // here we will construct Cayenne Low Power Payload (LPP) - see https://community.mydevices.com/t/cayenne-lpp-2-0/7510
gps gps; // class that is encapsulating additional GPS functionality

double lat, lon, alt, kmph; // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
int sats; // GPS satellite count
char s[32]; // used to sprintf for Serial output
float vBat; // battery voltage
long nextPacketTime;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xD1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob; // callback to LoRa send packet 
void do_send(osjob_t* j); // declaration of send function

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
#ifdef DEBUG
const unsigned int TX_INTERVAL = 30;
#else
const unsigned int TX_INTERVAL = 900;
#endif

// Those variables keep their values after software restart or wakeup from sleep, not after power loss or hard reset !
RTC_NOINIT_ATTR int RTCseqnoUp, RTCseqnoDn;
RTC_NOINIT_ATTR u4_t otaaDevAddr;
RTC_NOINIT_ATTR u1_t otaaNetwKey[16];
RTC_NOINIT_ATTR u1_t otaaApRtKey[16];

const unsigned int GPS_FIX_RETRY_DELAY = 15; // Wait this many seconds when no GPS fix is received to retry
const unsigned int SHORT_TX_INTERVAL = 60; // When moving, send packets every SHORT_TX_INTERVAL seconds
const double MOVING_KMPH = 3.5; // If speed in km/h is higher than MOVING_HMPH, we assume that car is moving

// Pin mapping for REV v0.7 TTGO T-Beam
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

void getBatteryVoltage() {
  // we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
  vBat = analogRead(BATTERY_PIN) * 2.0 * (3.3 / 3500.0);
  Serial.print("Battery voltage: ");
  Serial.print(vBat);
  Serial.println("V");  
}

void storeFrameCounters()
{
  RTCseqnoUp = LMIC.seqnoUp;
  RTCseqnoDn = LMIC.seqnoDn;
  sprintf(s, "Counters stored as %d/%d", LMIC.seqnoUp, LMIC.seqnoDn);
  Serial.println(s);
}

void restoreFrameCounters()
{
  LMIC.seqnoUp = RTCseqnoUp;
  LMIC.seqnoDn = RTCseqnoDn;
  sprintf(s, "Restored counters as %d/%d", LMIC.seqnoUp, LMIC.seqnoDn);
  Serial.println(s);
}

void setOrRestorePersistentCounters()
{
  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason != ESP_RST_DEEPSLEEP) && (reason != ESP_RST_SW))
  {
    Serial.println(F("Counters both set to 0"));
    LMIC.seqnoUp = 0;
    LMIC.seqnoDn = 0;
  }
  else
  {
    restoreFrameCounters();
  }
}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            otaaDevAddr = LMIC.devaddr;
            memcpy_P(otaaNetwKey, LMIC.nwkKey, 16);
            memcpy_P(otaaApRtKey, LMIC.artKey, 16);
            LMIC_setDrTxpow(DR_SF8,14);
            sprintf(s, "got devaddr = 0x%X", LMIC.devaddr);
            Serial.println(s);
 //           digitalWrite(BUILTIN_LED, LOW); // LED on after join
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
  //          digitalWrite(BUILTIN_LED, HIGH); // LED off after failed join
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
 //           digitalWrite(BUILTIN_LED, HIGH); // LED off after failed rejoin
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            nextPacketTime = (kmph > MOVING_KMPH ? SHORT_TX_INTERVAL : TX_INTERVAL); // depend on current GPS speed
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(nextPacketTime), do_send);
            Serial.print(F("Next LoRa packet scheduled in "));
            Serial.print(nextPacketTime);
            Serial.println(F(" seconds!"));
            storeFrameCounters();
            digitalWrite(BUILTIN_LED, LOW); // LED on after first complete message transmission
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j) {  

  getBatteryVoltage();
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  { 
    if (gps.checkGpsFix())
    {
      // Prepare upstream data transmission at the next possible time.
      gps.getLatLon(&lat, &lon, &alt, &kmph, &sats);

      // we have all the data that we need, let's construct LPP packet for Cayenne
      lpp.reset();
      lpp.addGPS(1, lat, lon, alt);
      lpp.addAnalogInput(5, vBat);
      lpp.addAnalogInput(6, kmph);
      lpp.addAnalogInput(7, sats);
            
      // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send to TTN
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
      
      Serial.print(lpp.getSize());
      Serial.println(F(" bytes long LPP packet queued."));
//      digitalWrite(BUILTIN_LED, LOW); // LED on if fix and packet queued
    }
    else
    {
      // try again in a few 'GPS_FIX_RETRY_DELAY' seconds...
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), do_send);

    }
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting GPS"));
    gps.init();
    Serial.println(F("Initializing LoRa module"));

    // Don't ask for ACKs: Disable link check validation
    LMIC_setLinkCheckMode(0);
    // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
    // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
    LMIC_setDrTxpow(DR_SF8,14);
    // This must be done AFTER calling LMIC_setSession !
    setOrRestorePersistentCounters();
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    pinMode(BUILTIN_LED, OUTPUT);
    Serial.println(F("Ready to track"));

    // Start job (sending automatically starts OTAA too)
//    axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ); // 4blink/sec, high rate
    do_send(&sendjob);
}

 void loop() {
     os_runloop_once();
//     digitalWrite(14, LOW);   // turn the LED on (HIGH is the voltage level)
//     delay(500);              // wait for half a second
//     digitalWrite(14, HIGH);  // turn the LED off by making the voltage LOW
//     delay(500);              // wait for half a second
 }
