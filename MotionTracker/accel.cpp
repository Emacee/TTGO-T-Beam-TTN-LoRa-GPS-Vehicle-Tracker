#include <Wire.h>
#include "config.h"
#include "accel.h"

// LIS3DH registers
#define LIS3DH_WHO_AM_I     0x0F // = 0x33
#define LIS3DH_CTRL_REG1    0x20
#define LIS3DH_CTRL_REG2    0x21
#define LIS3DH_CTRL_REG3    0x22
#define LIS3DH_CTRL_REG4    0x23
#define LIS3DH_CTRL_REG5    0x24
#define LIS3DH_CTRL_REG6    0x25
#define LIS3DH_REFERENCE    0x26
#define LIS3DH_INT1_CFG     0x30
#define LIS3DH_INT1_SRC     0x31
#define LIS3DH_INT1_THS     0x32
#define LIS3DH_INT1_DUR     0x33

// MPU6050 registers
#define MPU6050_CONFIG          0x1A
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_MOT_THR         0x1F
#define MPU6050_MOT_DUR         0x20
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_MOT_DETECT_CTRL 0x69
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_WHO_AM_I        0x75 // = 0x68

bool Accel::begin(bool configure)
{
  if (probe(0x18, LIS3DH_WHO_AM_I, 0x33) || probe(0x19, LIS3DH_WHO_AM_I, 0x33)) {
    chip = LIS3DH;
    if (configure) setupLis3dh();
  } else if (probe(0x68, MPU6050_WHO_AM_I, 0x68) || probe(0x69, MPU6050_WHO_AM_I, 0x68)) {
    chip = MPU6050;
    if (configure) setupMpu6050();
  } else {
    chip = NONE;
    return false;
  }
  pinMode(ACCEL_INT_PIN, INPUT_PULLDOWN);
  if (configure) clear();
  return true;
}

const char* Accel::name() const
{
  switch (chip) {
    case LIS3DH:  return "LIS3DH";
    case MPU6050: return "MPU6050";
    default:      return "none";
  }
}

bool Accel::motionDetected()
{
  switch (chip) {
    case LIS3DH:  return read(LIS3DH_INT1_SRC) & 0x40;     // IA bit, reading clears the latch
    case MPU6050: return read(MPU6050_INT_STATUS) & 0x40;  // MOT_INT bit, reading clears the latch
    default:      return false;
  }
}

void Accel::clear()
{
  motionDetected();
}

void Accel::setupLis3dh()
{
  write(LIS3DH_CTRL_REG5, 0x80);  // BOOT: reload trimming values
  delay(10);
  write(LIS3DH_CTRL_REG1, 0x2F);  // 10 Hz, low-power mode, X/Y/Z enabled
  write(LIS3DH_CTRL_REG2, 0x01);  // high-pass filter on interrupt 1 (ignores gravity / static tilt)
  write(LIS3DH_CTRL_REG3, 0x40);  // IA1 interrupt routed to INT1 pin
  write(LIS3DH_CTRL_REG4, 0x00);  // +-2 g
  write(LIS3DH_CTRL_REG5, 0x08);  // latch interrupt 1 until INT1_SRC is read
  write(LIS3DH_CTRL_REG6, 0x00);  // INT pins active high
  write(LIS3DH_INT1_THS, LIS3DH_MOTION_THRESHOLD);
  write(LIS3DH_INT1_DUR, 0x00);
  read(LIS3DH_REFERENCE);         // reset the high-pass filter to the current orientation
  write(LIS3DH_INT1_CFG, 0x2A);   // OR of X-high, Y-high, Z-high events
}

void Accel::setupMpu6050()
{
  write(MPU6050_PWR_MGMT_1, 0x80);      // device reset
  delay(100);
  write(MPU6050_PWR_MGMT_1, 0x00);      // wake up
  write(MPU6050_CONFIG, 0x00);
  write(MPU6050_ACCEL_CONFIG, 0x01);    // +-2 g, high-pass filter 5 Hz (required for motion detection)
  write(MPU6050_MOT_THR, MPU6050_MOTION_THRESHOLD);
  write(MPU6050_MOT_DUR, 1);
  write(MPU6050_MOT_DETECT_CTRL, 0x15); // accelerometer power-on delay + decrement rates
  write(MPU6050_INT_PIN_CFG, 0x30);     // active high, push-pull, latched, cleared by any read
  write(MPU6050_INT_ENABLE, 0x40);      // motion interrupt
  delay(50);
  write(MPU6050_ACCEL_CONFIG, 0x07);    // high-pass filter "hold" so cycle mode compares against this reference
  write(MPU6050_PWR_MGMT_2, 0x47);      // 5 Hz wake-up rate, gyroscopes in standby
  write(MPU6050_PWR_MGMT_1, 0x28);      // cycle mode, temperature sensor off
}

bool Accel::probe(uint8_t address, uint8_t whoAmIReg, uint8_t expected)
{
  addr = address;
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) return false;
  return read(whoAmIReg) == expected;
}

void Accel::write(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t Accel::read(uint8_t reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return 0;
  return Wire.read();
}
