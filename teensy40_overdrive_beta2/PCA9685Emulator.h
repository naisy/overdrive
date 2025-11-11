// PCA9685Emulator.h - Fully compatible emulator header for PCA9685
#ifndef PCA9685_EMULATOR_H
#define PCA9685_EMULATOR_H

#include <Arduino.h>
#include <Wire.h>

class PCA9685Emulator {
public:
  static void begin(uint8_t addr);
  static void loop();
  static uint16_t readChannelUs(uint8_t ch);

  // I2C emulation handlers
  static void onReceive(int numBytes);
  static void onRequest();

  // Accessors for internal state (for testing and calculations)
  static uint8_t getRegister(uint8_t reg);
  static constexpr uint8_t getPreScaleReg() { return PRE_SCALE; }

private:
  static void applyRegister(uint8_t reg, uint8_t value);

  // PCA9685 register map (full set)
  static constexpr uint8_t MODE1              = 0x00;
  static constexpr uint8_t MODE2              = 0x01;
  static constexpr uint8_t SUBADR1            = 0x02;
  static constexpr uint8_t SUBADR2            = 0x03;
  static constexpr uint8_t SUBADR3            = 0x04;
  static constexpr uint8_t FAKE_PRESCALE      = 0x05;
  static constexpr uint8_t LED0_ON_L          = 0x06;
  static constexpr uint8_t LED0_ON_H          = 0x07;
  static constexpr uint8_t LED0_OFF_L         = 0x08;
  static constexpr uint8_t LED0_OFF_H         = 0x09;
  static constexpr uint8_t ALL_LED_ON_L       = 0xFA;
  static constexpr uint8_t ALL_LED_ON_H       = 0xFB;
  static constexpr uint8_t ALL_LED_OFF_L      = 0xFC;
  static constexpr uint8_t ALL_LED_OFF_H      = 0xFD;
  static constexpr uint8_t PRE_SCALE          = 0xFE;

  // Bit masks for MODE1 and MODE2
  static constexpr uint8_t RESTART            = 0x80;
  static constexpr uint8_t SLEEP              = 0x10;
  static constexpr uint8_t ALLCALL            = 0x01;
  static constexpr uint8_t OUTDRV             = 0x04;

  // Internal storage
  static uint8_t i2c_address;
  static uint8_t registers[256];
  static uint32_t lastUpdate;
};

#endif // PCA9685_EMULATOR_H
