/*******************************************************************************
 * TTGO T-Beam LoRaWAN GPS tracker with motion-triggered reporting.
 *
 * Based on TTGO-T-Beam-TTN-LoRa-GPS-Vehicle-Tracker (LMIC OTAA example by
 * Thomas Telkamp and Matthijs Kooijman).
 *
 * Behaviour:
 *   MOVING  - GPS on, uplink every 30-120 s (shorter the faster you go).
 *   STOPPED - no motion (accelerometer and GPS speed) for STOP_TIMEOUT_S:
 *             one final "stopped" uplink, then deep sleep.
 *   PARKED  - deep sleep with LoRa + GPS powered off. The ESP32 wakes up
 *             every HEARTBEAT_S (300-900 s) to send a heartbeat with the last
 *             known position, or immediately when the accelerometer detects
 *             motion (or the user button is pressed).
 *
 * The LoRaWAN session (keys, frame counters, channels, duty-cycle state) is
 * kept in RTC memory across deep sleep, so the tracker joins only once after
 * power-up instead of on every wake.
 *
 * Uplink: Cayenne LPP on port LORAWAN_PORT
 *   ch 1 GPS (lat, lon, alt)   ch 5 battery voltage   ch 6 speed km/h
 *   ch 7 satellites            ch 8 state: 0 parked (heartbeat), 1 moving, 2 stopped
 *
 * Requires the MCCI LoRaWAN LMIC library with the right region selected
 * (see README), TinyGPSPlus, CayenneLPP and XPowersLib.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <CayenneLPP.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <sys/time.h>

#include "config.h"
#include "accel.h"
#include "gps.h"
#include "power.h"

enum TrackerState : uint8_t { STATE_PARKED = 0, STATE_MOVING = 1, STATE_STOPPED = 2 };

CayenneLPP lpp(51);
gps gps;
Accel accel;

// ---------------------------------------------------------------------------
// LoRaWAN keys and pins
// ---------------------------------------------------------------------------
static const u1_t PROGMEM APPEUI[8] = LORAWAN_APPEUI;
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = LORAWAN_DEVEUI;
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = LORAWAN_APPKEY;
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping for TTGO T-Beam v1.x
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

// ---------------------------------------------------------------------------
// State kept in RTC memory: survives deep sleep, reset on power loss / reflash
// ---------------------------------------------------------------------------
RTC_DATA_ATTR lmic_t rtcLmic;
RTC_DATA_ATTR bool rtcSessionValid = false;
RTC_DATA_ATTR int64_t rtcSleepStartUs;
RTC_DATA_ATTR ostime_t rtcOsTimeAtSleep;
RTC_DATA_ATTR bool rtcHaveFix = false;
RTC_DATA_ATTR double rtcLat, rtcLon, rtcAlt;
RTC_DATA_ATTR uint32_t rtcBootCount = 0;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
TrackerState state;
bool gpsPowered = false;
bool forceReport = false;       // power-up / button: report without waiting for confirmed movement
bool uplinkQueued = false;      // waiting for EV_TXCOMPLETE
TrackerState queuedState;       // state reported by the queued uplink
bool reportedMoving = false;    // at least one MOVING uplink went out during this wake
bool finalUplinkDone = false;   // heartbeat / stopped uplink completed -> may sleep
uint32_t stateSinceMs = 0;
uint32_t lastUplinkMs = 0;
uint32_t lastMotionMs = 0;      // accelerometer or GPS speed
uint32_t lastAccelMs = 0;
bool accelSeen = false;
uint32_t lastEvalMs = 0;

bool haveFix = false;
GpsFix fix;

void onEvent(ev_t ev);
void goToSleep();

// ---------------------------------------------------------------------------
// LoRaWAN session persistence across deep sleep
// ---------------------------------------------------------------------------
static int64_t rtcNowUs()
{
  struct timeval tv;
  gettimeofday(&tv, NULL); // backed by the RTC timer, keeps counting in deep sleep
  return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

void saveSession()
{
  rtcLmic = LMIC;
  // Normally LMIC is idle here. If we gave up waiting (e.g. duty cycle), drop the
  // pending uplink; the frame counter already moved on, so it is never reused.
  rtcLmic.opmode &= ~(OP_TXDATA | OP_TXRXPEND | OP_POLL);
  rtcOsTimeAtSleep = os_getTime();
  rtcSleepStartUs = rtcNowUs();
  rtcSessionValid = true;
  Serial.printf("Session saved, FCntUp=%u\n", LMIC.seqnoUp);
}

void restoreSession()
{
  int64_t sleptUs = rtcNowUs() - rtcSleepStartUs;
  if (sleptUs < 0) sleptUs = 0;

  LMIC = rtcLmic;

  // The LMIC clock restarted at 0 on boot. Shift the stored duty-cycle
  // timestamps by the time that really passed so we neither violate the
  // duty cycle nor wait for time that already elapsed.
  ostime_t shift = rtcOsTimeAtSleep + us2osticks(sleptUs) - os_getTime();
#if CFG_LMIC_EU_like
  for (int i = 0; i < MAX_BANDS; i++) {
    LMIC.bands[i].avail -= shift;
  }
#endif
  LMIC.globalDutyAvail -= shift;

  Serial.printf("Session restored after %lld s asleep, FCntUp=%u\n", sleptUs / 1000000LL, LMIC.seqnoUp);
}

bool lmicIdle()
{
  return !(LMIC.opmode & (OP_JOINING | OP_TXRXPEND | OP_TXDATA | OP_POLL)) &&
         !os_queryTimeCriticalJobs(ms2osticksRound(2000));
}

// ---------------------------------------------------------------------------
// Uplinks
// ---------------------------------------------------------------------------
uint32_t movingIntervalS(double kmph)
{
  if (kmph <= MOVING_KMPH) return MOVING_TX_MAX_S;
  if (kmph >= FAST_KMPH) return MOVING_TX_MIN_S;
  double f = (kmph - MOVING_KMPH) / (FAST_KMPH - MOVING_KMPH);
  return MOVING_TX_MAX_S - (uint32_t)(f * (MOVING_TX_MAX_S - MOVING_TX_MIN_S));
}

void queueUplink(TrackerState reportState)
{
  float vBat = power::batteryVoltage();

  lpp.reset();
  if (haveFix) {
    lpp.addGPS(1, fix.lat, fix.lon, fix.alt);
  } else if (reportState != STATE_MOVING && rtcHaveFix) {
    lpp.addGPS(1, rtcLat, rtcLon, rtcAlt); // not moving: last known position is still correct
  }
  lpp.addAnalogInput(5, vBat);
  lpp.addAnalogInput(6, haveFix ? fix.kmph : 0);
  lpp.addAnalogInput(7, haveFix ? fix.sats : 0);
  lpp.addDigitalInput(8, reportState);

  if (LMIC_setTxData2(LORAWAN_PORT, lpp.getBuffer(), lpp.getSize(), 0) == 0) {
    uplinkQueued = true;
    queuedState = reportState;
    lastUplinkMs = millis();
    if (reportState == STATE_MOVING) reportedMoving = true;
    Serial.printf("Uplink queued: state=%u fix=%d speed=%.1f km/h vbat=%.2f V (%u bytes)\n",
                  reportState, haveFix, haveFix ? fix.kmph : 0.0, vBat, lpp.getSize());
  } else {
    Serial.println(F("LMIC_setTxData2 failed, will retry"));
  }
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
void setGps(bool on)
{
  if (on == gpsPowered) return;
  power::gps(on);
  gpsPowered = on;
  if (!on) haveFix = false;
  Serial.printf("GPS %s\n", on ? "on" : "off");
}

void enterState(TrackerState s)
{
  state = s;
  stateSinceMs = millis();
  finalUplinkDone = false;
  if (s == STATE_MOVING) {
    setGps(true);
    lastMotionMs = millis();
  }
  const char* names[] = { "PARKED", "MOVING", "STOPPED" };
  Serial.printf("State -> %s\n", names[s]);
}

void evaluate()
{
  uint32_t now = millis();

  if (accel.present() && accel.motionDetected()) {
    lastAccelMs = now;
    accelSeen = true;
  }
  bool accelMoving = accelSeen && (now - lastAccelMs) < ACCEL_MOTION_HOLD_S * 1000UL;

  haveFix = gpsPowered && gps.hasFix();
  if (haveFix) fix = gps.read();
  bool gpsMoving = haveFix && fix.kmph > MOVING_KMPH;

  if (accelMoving || gpsMoving) lastMotionMs = now;

  switch (state) {
    case STATE_PARKED: {
      // Heartbeat wake. Without an accelerometer the GPS has to tell us whether we moved.
      bool movedWithoutAccel = !accel.present() && haveFix && rtcHaveFix &&
                               (gpsMoving || gps.distanceTo(rtcLat, rtcLon) > 100);
      if (accelMoving || movedWithoutAccel) {
        enterState(STATE_MOVING);
        break;
      }
      if (!uplinkQueued && !finalUplinkDone) {
        bool gpsWaitOver = !gpsPowered || haveFix || (now - stateSinceMs) > HEARTBEAT_GPS_WAIT_S * 1000UL;
        if (gpsWaitOver) {
          setGps(false);
          queueUplink(STATE_PARKED);
        }
      }
      break;
    }

    case STATE_MOVING: {
      if ((now - lastMotionMs) > STOP_TIMEOUT_S * 1000UL) {
        enterState(STATE_STOPPED);
        if (!reportedMoving) finalUplinkDone = true; // only a bump (door, wind): nothing to report
        break;
      }
      if (uplinkQueued) break;

      if (!reportedMoving && !forceReport) {
        // First report after a wake: wait until the movement is confirmed, either by GPS speed
        // or by the accelerometer still reporting motion once the fix wait is over.
        bool fixWaitOver = (now - stateSinceMs) > GPS_FIX_WAIT_S * 1000UL;
        if (gpsMoving || (fixWaitOver && accelMoving)) queueUplink(STATE_MOVING);
      } else if (forceReport) {
        if (haveFix || (now - stateSinceMs) > GPS_FIX_WAIT_S * 1000UL) {
          forceReport = false;
          queueUplink(STATE_MOVING);
        }
      } else if ((now - lastUplinkMs) >= movingIntervalS(haveFix ? fix.kmph : 0) * 1000UL) {
        queueUplink(STATE_MOVING);
      }
      break;
    }

    case STATE_STOPPED:
      if (accelMoving || gpsMoving) {
        enterState(STATE_MOVING);
        break;
      }
      if (!uplinkQueued && !finalUplinkDone) queueUplink(STATE_STOPPED);
      break;
  }

  // Parked or stopped: sleep once the last uplink is done and LMIC is idle. Give up after
  // JOIN_GIVEUP_S (e.g. no network coverage) instead of draining the battery.
  if (state != STATE_MOVING) {
    bool done = finalUplinkDone && lmicIdle();
    bool giveUp = (now - stateSinceMs) > JOIN_GIVEUP_S * 1000UL;
    if (done || giveUp) {
      if (giveUp && !done) Serial.println(F("Uplink not completed in time, sleeping anyway"));
      goToSleep();
    }
  }
}

// ---------------------------------------------------------------------------
// Deep sleep
// ---------------------------------------------------------------------------
void goToSleep()
{
  if (LMIC.devaddr != 0) {
    saveSession();
  } else {
    rtcSessionValid = false; // join never completed: join again on the next wake
  }

  // Motion that arrived just now must not be lost: clear the latch, and if the
  // sensor re-triggers immediately stay awake instead.
  accel.clear();
  if (accel.present() && digitalRead(ACCEL_INT_PIN)) {
    Serial.println(F("Motion while going to sleep, staying awake"));
    enterState(STATE_MOVING);
    return;
  }

  power::prepareSleep();

  if (accel.present()) {
    rtc_gpio_pullup_dis(ACCEL_INT_PIN);
    rtc_gpio_pulldown_en(ACCEL_INT_PIN);
    esp_sleep_enable_ext0_wakeup(ACCEL_INT_PIN, 1);
  }
#if WAKE_ON_BUTTON
  esp_sleep_enable_ext1_wakeup(1ULL << USER_BUTTON_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
#endif
  esp_sleep_enable_timer_wakeup((uint64_t)HEARTBEAT_S * 1000000ULL);

  Serial.printf("Deep sleep for up to %u s\n", HEARTBEAT_S);
  Serial.flush();
  esp_deep_sleep_start();
}

// ---------------------------------------------------------------------------
// LMIC events
// ---------------------------------------------------------------------------
void onEvent(ev_t ev)
{
  switch (ev) {
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      power::led(true);
      break;
    case EV_JOINED:
      Serial.printf("EV_JOINED, devaddr=0x%08X\n", LMIC.devaddr);
      power::led(false);
      LMIC_setLinkCheckMode(0);
      LMIC_setAdrMode(0);
      LMIC_setDrTxpow(LORAWAN_DATARATE, LORAWAN_TXPOWER);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE"));
      if (LMIC.dataLen) Serial.printf("Received %u bytes of downlink\n", LMIC.dataLen);
      uplinkQueued = false;
      if (queuedState != STATE_MOVING && state == queuedState) finalUplinkDone = true;
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      uplinkQueued = false;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  rtcBootCount++;

  WiFi.mode(WIFI_OFF);
  btStop();

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!power::begin()) Serial.println(F("No AXP192/AXP2101 found"));

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool fromDeepSleep = esp_reset_reason() == ESP_RST_DEEPSLEEP;

  if (!accel.begin(!fromDeepSleep)) Serial.println(F("No accelerometer found: motion wake-up disabled, using GPS only"));
  Serial.printf("\nMotionTracker boot #%u, PMU %s, accel %s, wake cause %d\n",
                rtcBootCount, power::pmuName(), accel.name(), cause);

  gpsPowered = true; // power::begin() switched the GPS rail on
  gps.init();
  if (fromDeepSleep && cause == ESP_SLEEP_WAKEUP_TIMER) {
    // Heartbeat. Keep the GPS powered only when we need a fresh position.
    setGps(HEARTBEAT_REFRESH_GPS || !rtcHaveFix || !accel.present());
    enterState(STATE_PARKED);
  } else {
    // Power-up, motion or button: start tracking.
    forceReport = !fromDeepSleep || cause == ESP_SLEEP_WAKEUP_EXT1;
    enterState(STATE_MOVING);
  }

  os_init();
  LMIC_reset();
  if (fromDeepSleep && rtcSessionValid) {
    restoreSession();
  } else {
    rtcSessionValid = false;
    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_startJoining();
  }
}

void loop()
{
  os_runloop_once();
  if (gpsPowered) gps.feed();

  // LMIC needs frequent attention; evaluate the tracker logic a few times per second.
  if (millis() - lastEvalMs >= 250) {
    lastEvalMs = millis();
    evaluate();
  }

  if (haveFix) {
    rtcHaveFix = true;
    rtcLat = fix.lat;
    rtcLon = fix.lon;
    rtcAlt = fix.alt;
  }
}
