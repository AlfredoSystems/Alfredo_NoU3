#pragma once

// I2C driver for the NXP PCA9685 16-channel PWM controller that drives the
// NoU3's motor H-bridges.
//
// Original implementation for Alfredo Systems, written against the NXP
// datasheet register descriptions. Supports the single device on the NoU3
// (default address 0x40).

#include <Arduino.h>
#include <Wire.h>

class PCA9685 {
  public:
    // Resets the device (I2C general-call SWRST), then wakes it with
    // register auto-increment enabled.
    void setupSingleDevice(TwoWire &wire, uint8_t address = 0x40) {
      _wire = &wire;
      _address = address;

      _wire->beginTransmission(GENERAL_CALL_ADDRESS);
      _wire->write(SWRST_COMMAND);
      _wire->endTransmission();
      delay(10);

      wake();
    }

    // The output-enable pin is active low; start disabled.
    void setupOutputEnablePin(uint8_t pin) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    }

    void enableOutputs(uint8_t pin) { digitalWrite(pin, LOW); }
    void disableOutputs(uint8_t pin) { digitalWrite(pin, HIGH); }

    // PWM frequency for all channels, ~24 to ~1526 Hz.
    void setToFrequency(uint16_t frequency) {
      if (frequency == 0)
        return;
      // The prescale value is linear in the PWM period. Interpolate between
      // the two calibrated endpoints (the internal oscillator runs slightly
      // fast, so the measured periods are shorter than the datasheet's
      // nominal 655 and 41666 us).
      uint32_t period_us = 1000000UL / frequency;
      period_us = constrain(period_us, PERIOD_MIN_US, PERIOD_MAX_US);
      uint8_t prescale = map(period_us, PERIOD_MIN_US, PERIOD_MAX_US,
                             PRESCALE_MIN, PRESCALE_MAX);

      // The prescale register can only be written while the device sleeps.
      sleep();
      writeRegister(_address, REG_PRESCALE, prescale);
      wake();
    }

    // Duty cycle in percent (0-100) for one channel (0-15). percentDelay
    // phase-shifts the pulse within the PWM period.
    void setChannelDutyCycle(uint8_t channel, float dutyCycle, float percentDelay = 0) {
      if (channel >= CHANNEL_COUNT)
        return;
      writeOnOffTimes(_address, REG_LED0_ON_L + 4 * channel, dutyCycle, percentDelay);
    }

    // Duty cycle in percent for every channel at once, via the ALL_LED
    // registers at the LED All Call address.
    void setAllChannelsDutyCycle(float dutyCycle, float percentDelay = 0) {
      writeOnOffTimes(ALL_CALL_ADDRESS, REG_ALL_LED_ON_L, dutyCycle, percentDelay);
    }

  private:
    enum : uint8_t {
      REG_MODE1        = 0x00,
      REG_LED0_ON_L    = 0x06,
      REG_ALL_LED_ON_L = 0xFA,
      REG_PRESCALE     = 0xFE,
    };

    static const uint8_t MODE1_SLEEP = 0x10;
    static const uint8_t MODE1_AI = 0x20;      // register auto-increment
    static const uint8_t MODE1_RESTART = 0x80; // write 1 to resume PWM after sleep

    static const uint8_t GENERAL_CALL_ADDRESS = 0x00;
    static const uint8_t SWRST_COMMAND = 0x06;
    static const uint8_t ALL_CALL_ADDRESS = 0x70;

    static const uint8_t CHANNEL_COUNT = 16;
    static const uint16_t COUNTS_PER_PERIOD = 4096;

    static const uint8_t PRESCALE_MIN = 0x03;
    static const uint16_t PERIOD_MIN_US = 617; // measured; nominal 655 (1526 Hz)
    static const uint8_t PRESCALE_MAX = 0xFF;
    static const uint16_t PERIOD_MAX_US = 39525; // measured; nominal 41666 (24 Hz)

    void sleep() {
      uint8_t mode1;
      if (!readRegister(REG_MODE1, mode1))
        return;
      writeRegister(_address, REG_MODE1, mode1 | MODE1_SLEEP);
    }

    void wake() {
      uint8_t mode1;
      if (!readRegister(REG_MODE1, mode1))
        return;
      mode1 = (mode1 & ~MODE1_SLEEP) | MODE1_AI;
      writeRegister(_address, REG_MODE1, mode1);
      // If PWM was interrupted by sleep, the restart flag reads 1; writing
      // it back after the oscillator settles resumes the outputs.
      if (mode1 & MODE1_RESTART) {
        delay(1);
        writeRegister(_address, REG_MODE1, mode1);
      }
    }

    // Convert a duty cycle and phase delay to the chip's on/off counts and
    // write them as one auto-incremented 4-byte transfer.
    void writeOnOffTimes(uint8_t address, uint8_t startRegister,
                         float dutyCycle, float percentDelay) {
      uint16_t pulse = (uint16_t)round(COUNTS_PER_PERIOD * (double)dutyCycle / 100.0);
      uint16_t phase = (uint16_t)round(COUNTS_PER_PERIOD * (double)percentDelay / 100.0);

      uint16_t onTime;
      uint16_t offTime;
      if (pulse == 0) {
        // Bit 12 of the off time is the full-off flag
        onTime = 0;
        offTime = COUNTS_PER_PERIOD;
      } else if (pulse >= COUNTS_PER_PERIOD) {
        // Bit 12 of the on time is the full-on flag
        onTime = COUNTS_PER_PERIOD;
        offTime = 0;
      } else {
        onTime = phase % COUNTS_PER_PERIOD;
        offTime = (onTime + pulse) % COUNTS_PER_PERIOD;
      }

      if (_wire == nullptr)
        return;
      _wire->beginTransmission(address);
      _wire->write(startRegister);
      _wire->write(onTime & 0xFF);
      _wire->write(onTime >> 8);
      _wire->write(offTime & 0xFF);
      _wire->write(offTime >> 8);
      _wire->endTransmission();
    }

    void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
      if (_wire == nullptr)
        return;
      _wire->beginTransmission(address);
      _wire->write(reg);
      _wire->write(value);
      _wire->endTransmission();
    }

    bool readRegister(uint8_t reg, uint8_t &value) {
      value = 0;
      if (_wire == nullptr)
        return false;
      _wire->beginTransmission(_address);
      _wire->write(reg);
      if (_wire->endTransmission() != 0)
        return false;
      if (_wire->requestFrom(_address, (uint8_t)1) != 1)
        return false;
      value = _wire->read();
      return true;
    }

    TwoWire *_wire = nullptr;
    uint8_t _address = 0x40;
};
