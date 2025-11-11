// PCA9685Emulator.cpp - Full implementation for PCA9685 compatible emulator

#include "PCA9685Emulator.h"
#include <Wire.h>
#include "PCA9685Emulator.h"

uint8_t PCA9685Emulator::i2c_address = 0x40;
uint8_t PCA9685Emulator::registers[256] = {0};
uint32_t PCA9685Emulator::lastUpdate = 0;

void PCA9685Emulator::begin(uint8_t addr) {
  i2c_address = addr;
  Wire.begin(addr);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  lastUpdate = millis();
}

void PCA9685Emulator::loop() {
  // No active output simulation needed for this emulator.
  // Users should call readChannelUs() to get current virtual PWM signal.
}

void PCA9685Emulator::onReceive(int numBytes) {
  static uint8_t reg_addr = 0;
  static bool addr_set = false;
  while (numBytes-- > 0) {
    uint8_t val = Wire.read();
    if (!addr_set) {
      reg_addr = val;
      addr_set = true;
    } else {
      applyRegister(reg_addr, val);
      reg_addr++;
    }
  }
  addr_set = false;
}

void PCA9685Emulator::onRequest() {
  Wire.write(registers[0]);
}

void PCA9685Emulator::applyRegister(uint8_t reg, uint8_t value) {
  registers[reg] = value;
}

uint8_t PCA9685Emulator::getRegister(uint8_t reg) {
  return registers[reg];
}

float getPwmPeriodUs() {
  uint8_t prescale = PCA9685Emulator::getRegister(PCA9685Emulator::getPreScaleReg());
  float freq = 25000000.0f / (4096.0f * (prescale + 1));
  return 1000000.0f / freq;
}

uint16_t PCA9685Emulator::readChannelUs(uint8_t ch) {
  uint8_t on_l = registers[LED0_ON_L + 4 * ch];
  uint8_t on_h = registers[LED0_ON_H + 4 * ch];
  uint8_t off_l = registers[LED0_OFF_L + 4 * ch];
  uint8_t off_h = registers[LED0_OFF_H + 4 * ch];

  uint16_t on = (on_h << 8) | on_l;
  uint16_t off = (off_h << 8) | off_l;
  uint16_t width = (off - on) & 0x0FFF;

  float period_us = getPwmPeriodUs();
  return (uint16_t)((width / 4096.0f) * period_us);
}
