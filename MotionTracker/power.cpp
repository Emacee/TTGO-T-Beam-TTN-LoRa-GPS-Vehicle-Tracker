#include <Wire.h>
#include <XPowersLib.h> // no XPOWERS_CHIP_* define: pulls in both the AXP192 and AXP2101 drivers
#include "config.h"
#include "power.h"

// Power rails (same function, different names on both chips):
//                 AXP192 (v1.0/v1.1)   AXP2101 (v1.2)
//   LoRa radio        LDO2                 ALDO2
//   GPS               LDO3                 ALDO3
//   3V3 header        DCDC1                DCDC1 (also ESP32, never switched off)
static XPowersLibInterface* pmu = nullptr;
static uint8_t railLora, railGps;

bool power::begin()
{
  XPowersLibInterface* candidate = new XPowersAXP2101(Wire, I2C_SDA, I2C_SCL, AXP2101_SLAVE_ADDRESS);
  if (candidate->init()) {
    pmu = candidate;
    railLora = XPOWERS_ALDO2;
    railGps = XPOWERS_ALDO3;
  } else {
    delete candidate;
    candidate = new XPowersAXP192(Wire, I2C_SDA, I2C_SCL, AXP192_SLAVE_ADDRESS);
    if (candidate->init()) {
      pmu = candidate;
      railLora = XPOWERS_LDO2;
      railGps = XPOWERS_LDO3;
    } else {
      delete candidate;
      return false;
    }
  }

  pmu->setPowerChannelVoltage(railLora, 3300);
  pmu->enablePowerOutput(railLora);
  pmu->setPowerChannelVoltage(railGps, 3300);
  pmu->enablePowerOutput(railGps);
  pmu->enablePowerOutput(XPOWERS_DCDC1);
  pmu->enableBattVoltageMeasure();
  pmu->enableBattDetection();
  return true;
}

const char* power::pmuName()
{
  if (!pmu) return "none";
  return pmu->getChipModel() == XPOWERS_AXP2101 ? "AXP2101" : "AXP192";
}

void power::gps(bool on)
{
  if (!pmu) return;
  if (on) pmu->enablePowerOutput(railGps);
  else pmu->disablePowerOutput(railGps);
}

float power::batteryVoltage()
{
  if (!pmu || !pmu->isBatteryConnect()) return 0.0f;
  return pmu->getBattVoltage() / 1000.0f;
}

void power::led(bool on)
{
  if (pmu && pmu->getChipModel() == XPOWERS_AXP192) {
    pmu->setChargingLedMode(on ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
  }
}

void power::prepareSleep()
{
  if (!pmu) return;
  led(false);
  pmu->disablePowerOutput(railGps);
  pmu->disablePowerOutput(railLora);
}
