#include <Wire.h>
#include "src/XPowersLib/src/XPowersLib.h" // no XPOWERS_CHIP_* define: pulls in both the AXP192 and AXP2101 drivers
#include "config.h"
#include "power.h"

// Power rails (same function, different names on both chips):
//                 AXP192 (v1.0/v1.1)   AXP2101 (v1.2)
//   LoRa radio        LDO2                 ALDO2
//   GPS               LDO3                 ALDO3
//   3V3 header        DCDC1                DCDC1 (also ESP32, never switched off)
static XPowersLibInterface* pmu = nullptr;
static uint8_t railLora, railGps;
static bool irqButton = false;

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

  // Remember why the PMU raised its interrupt (it may have woken us), then only
  // keep the interrupts we use, so the IRQ line is idle while we sleep.
  pmu->getIrqStatus();
  irqButton = pmu->isPekeyShortPressIrq();
  pmu->disableInterrupt(XPOWERS_ALL_INT);
#if WAKE_ON_VBUS
  pmu->enableInterrupt(XPOWERS_USB_INSERT_INT | XPOWERS_PWR_BTN_CLICK_INT);
#endif
  pmu->clearIrqStatus();
  pinMode(PMU_IRQ_PIN, INPUT); // GPIO 35 has no internal pull-up; relies on the one on the board
  return true;
}

bool power::vbusPresent()
{
  return pmu && pmu->isVbusIn();
}

bool power::wokeByButton() { return irqButton; }

bool power::irqPending()
{
  return digitalRead(PMU_IRQ_PIN) == LOW;
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
  pmu->clearIrqStatus();
}
