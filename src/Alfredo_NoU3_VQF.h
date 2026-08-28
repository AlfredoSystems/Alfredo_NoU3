#pragma once

// 6DOF IMU orientation for the NoU3, wrapping the VQF sensor fusion
// algorithm (src/vqf/, https://github.com/dlaidig/vqf, Laidig & Seel 2023)
// with automatic online gyroscope bias calibration. No user calibration
// steps are required: gyro bias is measured and removed whenever the board
// is briefly at rest, and is also tracked continuously during motion.
// 6DOF means gyro + accelerometer only - the magnetometer is never used.
//
// This wrapper is the full interface to sensor fusion. On the NoU3 the
// library feeds it samples automatically and sketches use it through the
// NoU3.fusion member:
//
//   NoU3.fusion.getQuaternion(w, x, y, z);
//   if (NoU3.fusion.isResting()) { ... }
//   NoU3.fusion.setGyroScale(1.002f, 0.998f, 1.001f);
//
// On top of the raw filter it adds: unit conversion (gyro rad/s, accel g
// in; VQF wants m/s²), optional accelerometer offset/scale and gyro
// sensitivity calibration, measured-dt correction for the IMU's internal
// oscillator error, flash (NVS) persistence of all calibration values, and
// thread safety (every method below may be called from any task; the raw
// `vqf` member is the one exception).
//
// Yaw/pitch/roll out are in radians, standard ZYX convention (yaw about Z,
// pitch about Y, roll about X). In 6DOF there is no absolute heading
// reference, so yaw is relative to the orientation at startup.

#include <Arduino.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "vqf/vqf.hpp"

class NoU3_VQF {
  public:
    // Nominal sample rates. Pass 0 (or nothing) for accelSampleRateHz to
    // use the gyro rate for both (the LSM6 samples both at the same rate).
    explicit NoU3_VQF(float gyroSampleRateHz = 104.0f, float accelSampleRateHz = 0.0f)
        : vqf(makeParams(), 1.0f / gyroSampleRateHz,
              accelSampleRateHz > 0 ? 1.0f / accelSampleRateHz : 1.0f / gyroSampleRateHz),
          nominalTs(1.0f / gyroSampleRateHz),
          dtFiltered(1.0f / gyroSampleRateHz) {
      mutex = xSemaphoreCreateMutex();
    }

    // ---- feeding the filter (the NoU3 library does this automatically) ----

    // Feed each sensor at its own rate. Gyro in rad/s, accel in g.
    void updateGyro(float gx, float gy, float gz) {
      lock();
      updateGyroRaw(gx, gy, gz);
      unlock();
    }

    // Interrupt-driven gyro variant: dt is the measured time (seconds)
    // between this data-ready edge and the previous one. The IMU's internal
    // oscillator is off by a few percent part-to-part, which would become
    // the same percent of angle error on every rotation; the measured dt is
    // outlier-gated and heavily low-pass filtered, then used to correct the
    // integration. Intervals from missed edges (~2x nominal) never pollute
    // the rate estimate.
    void updateGyro(float gx, float gy, float gz, float dt) {
      lock();
      // Accept only intervals within 10% of the current estimate into the
      // rate filter: a missed data-ready edge shows up as ~2x nominal and a
      // fallback poll as an odd in-between value, and neither may pollute
      // the estimate. The gate is relative to the estimate itself, so it
      // still converges even if the oscillator error exceeds 10%. Alpha
      // 0.002 averages over ~500 samples: ISR latency jitter is a few
      // microseconds against a multi-millisecond period, so far under 0.1%
      // survives to the filtered rate.
      if (dt > 0.9f * dtFiltered && dt < 1.1f * dtFiltered) {
        dtFiltered += 0.002f * (dt - dtFiltered);
      }

      // Integrating the gyro over the true period at VQF's fixed nominal
      // period is equivalent to scaling angular rate by (true / nominal).
      // This corrects the strapdown integration - the accuracy-critical
      // part - while leaving filter coefficients untouched (a few percent
      // error in a time constant is negligible). Always use the slow
      // filtered period here, never the raw gap from a missed edge: scaling
      // a sample by ~2x "to preserve rotation" also scales the gyro's DC
      // bias, and VQF's rest detector reads that as a movement transient -
      // frequent misses then keep resetting the rest timer, silently
      // disabling bias calibration. A missed sample only costs (current
      // rate x one period) of angle, which is far cheaper.
      float k = dtFiltered / nominalTs;
      updateGyroRaw(gx * k, gy * k, gz * k);
      unlock();
    }

    void updateAccel(float ax, float ay, float az) {
      lock();
      updateAccelRaw(ax, ay, az);
      unlock();
    }

    // Convenience for same-rate use: updateGyro + updateAccel.
    void update(float gx, float gy, float gz, float ax, float ay, float az) {
      lock();
      updateGyroRaw(gx, gy, gz);
      updateAccelRaw(ax, ay, az);
      unlock();
    }

    float getMeasuredSampleRate() {  // gyro Hz, from the filtered dt
      lock();
      float rate = 1.0f / dtFiltered;
      unlock();
      return rate;
    }

    // ---- orientation ----

    // Orientation of the sensor body relative to a Z-up world frame.
    // Quaternion is Hamilton convention, body-to-world.
    void getQuaternion(float& w, float& x, float& y, float& z) {
      float q[4];
      readQuat(q);
      w = q[0];
      x = q[1];
      y = q[2];
      z = q[3];
    }

    float getYaw() {  // rad, about world Z (up), CCW positive viewed from above
      float q[4];
      readQuat(q);
      return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    }

    float getPitch() {  // rad, ZYX Euler, about Y
      float q[4];
      readQuat(q);
      float s = 2.0f * (q[0] * q[2] - q[1] * q[3]);
      s = constrain(s, -1.0f, 1.0f);  // clamp so rounding can't push asinf out of domain
      return asinf(s);
    }

    float getRoll() {  // rad, ZYX Euler, about X
      float q[4];
      readQuat(q);
      return atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    }

    // ---- automatic gyro bias calibration ----

    bool isResting() {  // true while VQF detects the sensor is at rest
      lock();
      bool resting = vqf.getRestDetected();
      unlock();
      return resting;
    }

    // Rest-detection diagnostics: each sensor's deviation from its slow
    // low-pass reference, relative to its rest threshold. Both must stay
    // below 1.0 continuously for ~1.5 s for rest to be detected. If one
    // value keeps popping above 1.0, that sensor is what's vetoing rest.
    // (Rest is also vetoed while any gyro axis reads above the 2 deg/s
    // bias clip - check the raw gyroscope fields for that.)
    void getRestDeviations(float& gyroRelative, float& accelRelative) {
      vqf_real_t dev[2];
      lock();
      vqf.getRelativeRestDeviations(dev);
      unlock();
      gyroRelative = dev[0];
      accelRelative = dev[1];
    }

    void getGyroBias(float& x, float& y, float& z) {  // current estimate, rad/s
      vqf_real_t bias[3];
      lock();
      vqf.getBiasEstimate(bias);
      unlock();
      x = bias[0];
      y = bias[1];
      z = bias[2];
    }

    float getGyroBiasSigma() {  // its 1-sigma uncertainty, rad/s
      lock();
      float sigma = vqf.getBiasEstimate(0);
      unlock();
      return sigma;
    }

    // Seed the bias estimate (e.g. at boot from values a previous run saved
    // to flash) so yaw is accurate before the first rest. sigmaRadPerS
    // expresses how much to trust the seed; the default (~0.3 deg/s) is
    // loose enough for the online estimator to quickly correct what changed
    // since the save (e.g. from temperature).
    void seedGyroBias(float x, float y, float z, float sigmaRadPerS = 0.00524f) {
      vqf_real_t bias[3] = {x, y, z};
      lock();
      vqf.setBiasEstimate(bias, sigmaRadPerS);
      unlock();
    }

    // ---- optional calibration (see CalibrateAccel / CalibrateGyroScale) ----

    // Accelerometer calibration. Never required; improves absolute tilt
    // accuracy by ~1-2 deg. Applied as: corrected = (raw - offset) * scale, in g.
    void setAccelCalibration(float offsetX, float offsetY, float offsetZ,
                             float scaleX = 1.0f, float scaleY = 1.0f, float scaleZ = 1.0f) {
      lock();
      accelOffset[0] = offsetX;
      accelOffset[1] = offsetY;
      accelOffset[2] = offsetZ;
      accelScale[0] = scaleX;
      accelScale[1] = scaleY;
      accelScale[2] = scaleZ;
      unlock();
    }

    // Gyro sensitivity calibration. Never required; the chip's factory trim
    // is within ~1%, which costs up to ~3.6 deg per full rotation of yaw.
    // No online estimator can observe scale error in 6DOF, so this is the
    // only fix. Applied as: corrected = raw * scale.
    void setGyroScale(float scaleX, float scaleY, float scaleZ) {
      lock();
      gyroScale[0] = scaleX;
      gyroScale[1] = scaleY;
      gyroScale[2] = scaleZ;
      unlock();
    }

    void getGyroScale(float& scaleX, float& scaleY, float& scaleZ) {
      lock();
      scaleX = gyroScale[0];
      scaleY = gyroScale[1];
      scaleZ = gyroScale[2];
      unlock();
    }

    void reset() {  // forget orientation and bias state
      lock();
      vqf.resetState();
      unlock();
    }

    // ---- flash (NVS) persistence ----
    // Only the calibration values (accel offset/scale, gyro scale) persist;
    // gyro bias is deliberately not saved - it is re-measured from rest at
    // every boot (see NoU3.calibrateIMUs()), so no stale estimate can leak
    // between sessions.

    // Loads what the calibration examples saved: accel calibration and
    // gyro scale. Called by NoU3.begin().
    void loadCalibrationFromFlash() {
      Preferences prefs;
      if (!prefs.begin(PREFS_NAMESPACE, true)) {
        return;  // nothing saved yet
      }
      float accCal[6];
      if (prefs.getBytes(KEY_ACCEL_CAL, accCal, sizeof(accCal)) == sizeof(accCal)) {
        setAccelCalibration(accCal[0], accCal[1], accCal[2], accCal[3], accCal[4], accCal[5]);
      }
      float gyrScale[3];
      if (prefs.getBytes(KEY_GYRO_SCALE, gyrScale, sizeof(gyrScale)) == sizeof(gyrScale)) {
        setGyroScale(gyrScale[0], gyrScale[1], gyrScale[2]);
      }
      prefs.end();
    }

    // Used by the calibration examples: applies the values and saves them
    // where loadCalibrationFromFlash() (i.e. NoU3.begin()) finds them.
    void saveAccelCalibrationToFlash(float offsetX, float offsetY, float offsetZ,
                                     float scaleX, float scaleY, float scaleZ) {
      setAccelCalibration(offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ);
      float cal[6] = {offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ};
      Preferences prefs;
      if (prefs.begin(PREFS_NAMESPACE)) {
        prefs.putBytes(KEY_ACCEL_CAL, cal, sizeof(cal));
        prefs.end();
      }
    }

    void saveGyroScaleToFlash(float scaleX, float scaleY, float scaleZ) {
      setGyroScale(scaleX, scaleY, scaleZ);
      float scale[3] = {scaleX, scaleY, scaleZ};
      Preferences prefs;
      if (prefs.begin(PREFS_NAMESPACE)) {
        prefs.putBytes(KEY_GYRO_SCALE, scale, sizeof(scale));
        prefs.end();
      }
    }

    // Direct access to the underlying VQF filter for advanced tuning
    // (setTauAcc, setRestDetectionThresholds, getQuat6D, ...). NOT guarded
    // by the wrapper's lock - on the NoU3, only touch this from setup().
    VQF vqf;

  private:
    // VQF tuning deltas from the published defaults.
    static VQFParams makeParams() {
      VQFParams params;
      // The default 2 deg/s bias clip assumes a better-trimmed gyro than
      // some LSM6 parts have: rest detection is vetoed whenever any axis of
      // the low-passed raw gyro exceeds the clip, so a part with ~2 deg/s
      // of bias never rests and never gets calibrated (measured on real
      // NoU3s: X-axis bias riding right at the clip). 5 deg/s accepts
      // real-world part-to-part bias while still rejecting a robot that is
      // actually rotating.
      params.biasClip = 5.0f;
      return params;
    }

    static constexpr float GRAVITY_M_S2 = 9.80665f;
    static constexpr const char* PREFS_NAMESPACE = "nou3imu";
    static constexpr const char* KEY_ACCEL_CAL = "accCal";
    static constexpr const char* KEY_GYRO_SCALE = "gyrScale";

    void lock() {
      if (mutex != NULL) xSemaphoreTake(mutex, portMAX_DELAY);
    }
    void unlock() {
      if (mutex != NULL) xSemaphoreGive(mutex);
    }

    void updateGyroRaw(float gx, float gy, float gz) {
      vqf_real_t gyr[3] = {gx * gyroScale[0], gy * gyroScale[1], gz * gyroScale[2]};
      vqf.updateGyr(gyr);
    }

    void updateAccelRaw(float ax, float ay, float az) {
      vqf_real_t acc[3] = {
          (ax - accelOffset[0]) * accelScale[0] * GRAVITY_M_S2,
          (ay - accelOffset[1]) * accelScale[1] * GRAVITY_M_S2,
          (az - accelOffset[2]) * accelScale[2] * GRAVITY_M_S2,
      };
      vqf.updateAcc(acc);
    }

    void readQuat(float q[4]) {
      vqf_real_t out[4];
      lock();
      vqf.getQuat6D(out);
      unlock();
      for (int i = 0; i < 4; i++) q[i] = out[i];
    }

    SemaphoreHandle_t mutex = NULL;
    float accelOffset[3] = {0.0f, 0.0f, 0.0f};
    float accelScale[3] = {1.0f, 1.0f, 1.0f};
    float gyroScale[3] = {1.0f, 1.0f, 1.0f};
    float nominalTs;
    float dtFiltered;
};
