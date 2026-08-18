#pragma once

// I2C driver for the MEMSIC MMC5983MA magnetometer on the NoU3.
//
// Original implementation for Alfredo Systems, written against the MEMSIC
// datasheet register descriptions.
//
// The chip's control registers are write-only, so the driver keeps a local
// copy of each and always writes through it, preserving previously set bits.

#include <Arduino.h>
#include <Wire.h>

class MMC5983MAClass {
  public:
    bool begin(TwoWire &wire) {
      _wire = &wire;
      return isConnected();
    }

    bool isConnected() {
      uint8_t id = 0;
      return readRegisters(REG_PRODUCT_ID, &id, 1) && id == 0x30;
    }

    // Software reset; takes ~10 ms and zeroes all control registers.
    bool softReset() {
      bool ok = writeRegister(REG_CONTROL_1, 0x80);
      _control0 = _control1 = _control2 = 0;
      delay(15);
      return ok;
    }

    // Measurement filter bandwidth in Hz: 100, 200, 400, or 800.
    bool setFilterBandwidth(uint16_t bandwidth) {
      uint8_t bits;
      switch (bandwidth) {
        case 100: bits = 0x00; break;
        case 200: bits = 0x01; break;
        case 400: bits = 0x02; break;
        case 800: bits = 0x03; break;
        default: return false;
      }
      _control1 = (_control1 & ~0x03) | bits;
      return writeRegister(REG_CONTROL_1, _control1);
    }

    // Continuous measurement rate in Hz: 1000, 200, 100, 50, 20, 10, 1,
    // or 0 (off).
    bool setContinuousModeFrequency(uint16_t frequency) {
      uint8_t bits;
      switch (frequency) {
        case 0:    bits = 0x00; break;
        case 1:    bits = 0x01; break;
        case 10:   bits = 0x02; break;
        case 20:   bits = 0x03; break;
        case 50:   bits = 0x04; break;
        case 100:  bits = 0x05; break;
        case 200:  bits = 0x06; break;
        case 1000: bits = 0x07; break;
        default: return false;
      }
      _control2 = (_control2 & ~0x07) | bits;
      return writeRegister(REG_CONTROL_2, _control2);
    }

    bool enableContinuousMode() {
      _control2 |= 0x08;
      return writeRegister(REG_CONTROL_2, _control2);
    }

    bool disableContinuousMode() {
      _control2 &= ~0x08;
      return writeRegister(REG_CONTROL_2, _control2);
    }

    // Automatic set/reset cancels the sensor bridge's offset drift
    // between measurements.
    bool enableAutomaticSetReset() {
      _control0 |= 0x20;
      return writeRegister(REG_CONTROL_0, _control0);
    }

    bool disableAutomaticSetReset() {
      _control0 &= ~0x20;
      return writeRegister(REG_CONTROL_0, _control0);
    }

    // Assert the INT pin when a measurement completes.
    bool enableInterrupt() {
      _control0 |= 0x04;
      return writeRegister(REG_CONTROL_0, _control0);
    }

    bool disableInterrupt() {
      _control0 &= ~0x04;
      return writeRegister(REG_CONTROL_0, _control0);
    }

    // Clear the measurement-done flags (and release the INT pin) by
    // writing 1s to them in the status register.
    bool clearMeasDoneInterrupt() { return writeRegister(REG_STATUS, 0x03); }

    // Raw 18-bit field measurements; 131072 counts = zero field.
    bool readFieldsXYZ(uint32_t *x, uint32_t *y, uint32_t *z) {
      uint8_t b[7];
      if (!readRegisters(REG_XOUT_0, b, sizeof(b)))
        return false;
      // Each axis: two full bytes, plus its lowest 2 bits packed into b[6].
      *x = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | (b[6] >> 6);
      *y = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x03);
      *z = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x03);
      return true;
    }

    // Field in microteslas (+/-800 uT full scale), NoU3 axis convention
    // (all axes negated). Outputs are only meaningful when true is returned.
    bool readMagnetometer(float *x, float *y, float *z) {
      uint32_t rawX = 0, rawY = 0, rawZ = 0;
      bool ok = readFieldsXYZ(&rawX, &rawY, &rawZ);
      const float countsPerMicrotesla = 131072.0 / 800.0;
      *x = ((float)rawX - 131072.0) / countsPerMicrotesla * -1.0;
      *y = ((float)rawY - 131072.0) / countsPerMicrotesla * -1.0;
      *z = ((float)rawZ - 131072.0) / countsPerMicrotesla * -1.0;
      return ok;
    }

  private:
    enum : uint8_t {
      REG_XOUT_0     = 0x00,
      REG_STATUS     = 0x08,
      REG_CONTROL_0  = 0x09,
      REG_CONTROL_1  = 0x0A,
      REG_CONTROL_2  = 0x0B,
      REG_PRODUCT_ID = 0x2F,
    };

    static const uint8_t I2C_ADDRESS = 0x30;

    bool readRegisters(uint8_t reg, uint8_t *data, size_t length) {
      if (_wire == nullptr)
        return false;
      _wire->beginTransmission(I2C_ADDRESS);
      _wire->write(reg);
      if (_wire->endTransmission(false) != 0)
        return false;
      if (_wire->requestFrom(I2C_ADDRESS, length) != length)
        return false;
      while (length-- > 0)
        *data++ = _wire->read();
      return true;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
      if (_wire == nullptr)
        return false;
      _wire->beginTransmission(I2C_ADDRESS);
      _wire->write(reg);
      _wire->write(value);
      return _wire->endTransmission() == 0;
    }

    TwoWire *_wire = nullptr;
    // Local copies of the write-only control registers
    uint8_t _control0 = 0, _control1 = 0, _control2 = 0;
};
