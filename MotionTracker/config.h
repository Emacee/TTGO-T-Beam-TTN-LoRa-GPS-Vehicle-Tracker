#ifndef MOTIONTRACKER_CONFIG_H
#define MOTIONTRACKER_CONFIG_H

/*******************************************************************************
 * MotionTracker configuration
 *
 * Everything you are expected to change lives in this file.
 *******************************************************************************/

// #define DEBUG 1 // shorter intervals for testing on the desk; comment out for real use

// ---------------------------------------------------------------------------
// LoRaWAN OTAA keys (from the TTN console)
// ---------------------------------------------------------------------------
// DevEUI and JoinEUI (AppEUI) in LITTLE-endian (LSB first) - in the TTN console
// click the "<>" button and choose "lsb".
#define LORAWAN_DEVEUI  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define LORAWAN_APPEUI  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
// AppKey in BIG-endian (MSB first) - copy it as shown in the TTN console.
#define LORAWAN_APPKEY  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define LORAWAN_PORT        1       // FPort used for the Cayenne LPP uplinks
#define LORAWAN_DATARATE    DR_SF7  // fixed data rate; ADR is disabled because the device moves
#define LORAWAN_TXPOWER     14      // dBm

// ---------------------------------------------------------------------------
// Reporting intervals (seconds)
// ---------------------------------------------------------------------------
// While moving the interval scales with GPS speed between these two bounds:
// <= MOVING_KMPH -> MOVING_TX_MAX_S, >= FAST_KMPH -> MOVING_TX_MIN_S.
#ifdef DEBUG
const uint32_t MOVING_TX_MIN_S  = 30;
const uint32_t MOVING_TX_MAX_S  = 60;
const uint32_t HEARTBEAT_S      = 120;   // parked: deep sleep, wake up and send a heartbeat
const uint32_t STOP_TIMEOUT_S   = 60;    // no motion for this long -> parked
#else
const uint32_t MOVING_TX_MIN_S  = 30;
const uint32_t MOVING_TX_MAX_S  = 120;
const uint32_t HEARTBEAT_S      = 600;   // pick anything between 300 and 900
const uint32_t STOP_TIMEOUT_S   = 180;   // no motion for this long -> parked
#endif

const double   MOVING_KMPH      = 5.0;   // GPS speed above this counts as moving
const double   FAST_KMPH        = 80.0;  // at or above this speed use MOVING_TX_MIN_S
const uint32_t GPS_FIX_WAIT_S   = 60;    // after a motion wake, wait this long for a fix before sending without one
const uint32_t JOIN_GIVEUP_S    = 300;   // parked but not joined/not done after this long -> sleep anyway

// Heartbeat uplinks normally reuse the last known position (the accelerometer
// guarantees the tracker has not moved). Set to 1 to power the GPS and take a
// fresh fix on every heartbeat instead (costs considerably more battery).
#define HEARTBEAT_REFRESH_GPS 0
const uint32_t HEARTBEAT_GPS_WAIT_S = 45; // max. time to wait for that fix

// ---------------------------------------------------------------------------
// Accelerometer (the T-Beam has none on board - wire one to the I2C header)
// ---------------------------------------------------------------------------
// Supported: LIS3DH (recommended, ~6 uA) or MPU6050 / GY-521 (~20-70 uA).
// The driver auto-detects which one is connected.
//   SDA -> 21, SCL -> 22, VCC -> 3V3, GND -> GND, INT/INT1 -> ACCEL_INT_PIN
#define ACCEL_INT_PIN   GPIO_NUM_13 // must be an RTC GPIO: 0,2,4,12-15,25-27,32-39
// Wake/motion threshold. Units differ per sensor:
//   LIS3DH : 16 mg per step at +-2 g  (6 = ~96 mg)
//   MPU6050: ~2 mg per step           (20 = ~40 mg)
#define LIS3DH_MOTION_THRESHOLD   6
#define MPU6050_MOTION_THRESHOLD  20
// Motion seen by the accelerometer keeps the tracker in "moving" mode for this long.
const uint32_t ACCEL_MOTION_HOLD_S = 30;

// ---------------------------------------------------------------------------
// Board (TTGO T-Beam v1.0 / v1.1 with AXP192 and v1.2 with AXP2101)
// ---------------------------------------------------------------------------
#define I2C_SDA         21
#define I2C_SCL         22
#define GPS_RX_PIN      34  // ESP32 RX <- GPS TX
#define GPS_TX_PIN      12  // ESP32 TX -> GPS RX
#define USER_BUTTON_PIN GPIO_NUM_38 // middle button; wakes the tracker and forces an uplink
#define WAKE_ON_BUTTON  1

#endif // MOTIONTRACKER_CONFIG_H
