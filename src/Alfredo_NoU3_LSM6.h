#pragma once

// I2C driver for the ST LSM6-series accelerometer/gyroscope on the NoU3.
// Supports the LSM6DSOX, LSM6DSOW, LSM6DS3, and LSM6DSD variants.
//
// Original implementation for Alfredo Systems, written against the ST
// datasheet register descriptions.
//
// Configuration: accelerometer at 104 Hz, +/-4 g, with the ODR/4 low-pass
// filter; gyroscope at 104 Hz, +/-500 dps. Axes are remapped to the NoU3
// board convention (X/Y swapped, Y negated).

#include <Arduino.h>
#include <Wire.h>

class LSM6Class {
  public:
    // Detects the chip, then resets and configures it. Returns 1 on success.
    int begin(TwoWire &wire) {
      _wire = &wire;

      // {I2C address, expected WHO_AM_I} for each supported variant:
      // LSM6DSOX/LSM6DSOW, LSM6DS3, LSM6DSD.
      const uint8_t candidates[][2] = {
          {0x6B, 0x6C},
          {0x6A, 0x69},
          {0x6A, 0x6A},
      };
      for (const auto &candidate : candidates) {
        _address = candidate[0];
        if (readRegister(REG_WHO_AM_I) == candidate[1])
          return configure();
      }
      _address = 0;
      return 0;
    }

    // Acceleration in g, NoU3 axis convention. Returns 1 on success;
    // on failure the outputs are set to NAN.
    int readAcceleration(float *x, float *y, float *z) {
      int16_t raw[3];
      if (!readRegisters(REG_OUTX_L_A, (uint8_t *)raw, sizeof(raw))) {
        *x = *y = *z = NAN;
        return 0;
      }
      // 4 g full scale over a signed 16-bit range
      *x = raw[1] * 4.0 / 32768.0;
      *y = -raw[0] * 4.0 / 32768.0;
      *z = raw[2] * 4.0 / 32768.0;
      return 1;
    }

    // Combined read: one STATUS check, then a single 12-byte burst over the
    // contiguous gyro + accel output registers - a third of the bus time of
    // reading each sensor separately, and both samples are guaranteed to be
    // from the same instant. Returns 1 with all outputs filled on success;
    // 0 if either sensor has no fresh sample or the transaction failed.
    // Units and axis convention match readGyroscope()/readAcceleration().
    int readAccelerationAndGyroscope(float *gx, float *gy, float *gz,
                                     float *ax, float *ay, float *az) {
      int status = readRegister(REG_STATUS);
      if (status < 0 || (status & 0x03) != 0x03)
        return 0;
      int16_t raw[6];  // gyro X,Y,Z then accel X,Y,Z
      if (!readRegisters(REG_OUTX_L_G, (uint8_t *)raw, sizeof(raw))) {
        *gx = *gy = *gz = *ax = *ay = *az = NAN;
        return 0;
      }
      *gx = raw[1] * 0.0175 * (PI / 180.0);
      *gy = -raw[0] * 0.0175 * (PI / 180.0);
      *gz = raw[2] * 0.0175 * (PI / 180.0);
      *ax = raw[4] * 4.0 / 32768.0;
      *ay = -raw[3] * 4.0 / 32768.0;
      *az = raw[5] * 4.0 / 32768.0;
      return 1;
    }

    // Angular rate in rad/s, NoU3 axis convention. Returns 1 on success;
    // on failure the outputs are set to NAN.
    int readGyroscope(float *x, float *y, float *z) {
      int16_t raw[3];
      if (!readRegisters(REG_OUTX_L_G, (uint8_t *)raw, sizeof(raw))) {
        *x = *y = *z = NAN;
        return 0;
      }
      // 17.5 mdps per LSB at +/-500 dps, converted to rad/s
      *x = raw[1] * 0.0175 * (PI / 180.0);
      *y = -raw[0] * 0.0175 * (PI / 180.0);
      *z = raw[2] * 0.0175 * (PI / 180.0);
      return 1;
    }

    // Temperature in degrees C. Returns 1 on success.
    int readTemperatureFloat(float &temperature_deg) {
      int16_t raw;
      if (!readRegisters(REG_OUT_TEMP_L, (uint8_t *)&raw, sizeof(raw)))
        return 0;
      // 256 LSB per degree C, 0 LSB = 25 C
      temperature_deg = raw / 256.0f + 25.0f;
      return 1;
    }

    int readTemperature(int &temperature_deg) {
      float temperature = 0;
      int result = readTemperatureFloat(temperature);
      temperature_deg = (int)temperature;
      return result;
    }

    // STATUS_REG data-ready flags: accel bit 0, gyro bit 1, temperature bit 2
    int accelerationAvailable() { return (readRegister(REG_STATUS) & 0x01) ? 1 : 0; }
    int gyroscopeAvailable() { return (readRegister(REG_STATUS) & 0x02) ? 1 : 0; }
    int temperatureAvailable() { return (readRegister(REG_STATUS) & 0x04) ? 1 : 0; }

    float accelerationSampleRate() { return 104.0f; }
    float gyroscopeSampleRate() { return 104.0f; }

    // Route accel and gyro data-ready to the INT1 pin
    void enableInterrupt() { writeRegister(REG_INT1_CTRL, 0x03); }

  private:
    enum : uint8_t {
      REG_INT1_CTRL  = 0x0D,
      REG_WHO_AM_I   = 0x0F,
      REG_CTRL1_XL   = 0x10,
      REG_CTRL2_G    = 0x11,
      REG_CTRL3_C    = 0x12,
      REG_CTRL7_G    = 0x16,
      REG_CTRL8_XL   = 0x17,
      REG_CTRL9_XL   = 0x18,
      REG_STATUS     = 0x1E,
      REG_OUT_TEMP_L = 0x20,
      REG_OUTX_L_G   = 0x22,
      REG_OUTX_L_A   = 0x28,
    };

    int configure() {
      // Software reset; the chip keeps stale state through a host warm reset.
      writeRegister(REG_CTRL3_C, 0x01);
      unsigned long start = millis();
      while ((readRegister(REG_CTRL3_C) & 0x01) && millis() - start < 100)
        delay(1);

      writeRegister(REG_CTRL9_XL, 0xE2); // disable I3C interface
      writeRegister(REG_CTRL3_C, 0x44);  // block data update + address auto-increment
      writeRegister(REG_CTRL2_G, 0x44);  // gyro: 104 Hz, +/-500 dps
      writeRegister(REG_CTRL1_XL, 0x4A); // accel: 104 Hz, +/-4 g, LPF2 enabled
      writeRegister(REG_CTRL7_G, 0x00);  // gyro high-performance mode
      writeRegister(REG_CTRL8_XL, 0x09); // accel low-pass filter at ODR/4
      return 1;
    }

    // Returns the register value, or -1 on failure.
    int readRegister(uint8_t reg) {
      uint8_t value;
      if (!readRegisters(reg, &value, 1))
        return -1;
      return value;
    }

    bool readRegisters(uint8_t reg, uint8_t *data, size_t length) {
      if (_address == 0)
        return false;
      _wire->beginTransmission(_address);
      _wire->write(reg);
      if (_wire->endTransmission(false) != 0)
        return false;
      if (_wire->requestFrom(_address, length) != length)
        return false;
      while (length-- > 0)
        *data++ = _wire->read();
      return true;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
      if (_address == 0)
        return false;
      _wire->beginTransmission(_address);
      _wire->write(reg);
      _wire->write(value);
      return _wire->endTransmission() == 0;
    }

    TwoWire *_wire = nullptr;
    uint8_t _address = 0; // 0 until a chip has been detected
};
