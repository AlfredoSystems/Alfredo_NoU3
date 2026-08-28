/*
  CalibrateGyroScale  (OPTIONAL - NoU3 orientation works fine without this)

  One-time gyro sensitivity (scale factor) calibration. The chip's factory
  trim is within ~1%, which shows up as yaw reading short or long by up to
  ~3.6 degrees per full rotation. In 6DOF there is no heading reference, so
  no online estimator can see this error - a measured rotation is the only
  fix. Sensitivity is stable over time and temperature (~0.01%/degC), so
  once is enough. For best results do this warmed up to normal operating
  temperature.

  How to use: put the NoU3 flat against a straightedge (table edge, book).
  When prompted, spin it about the vertical axis a WHOLE number of turns -
  at least 3, more is better - and end aligned against the straightedge
  again. Speed doesn't matter, pausing to re-grip is fine (only 5 seconds of
  stillness ends the measurement). The sketch detects which axis you spun
  and how many turns you did (your job is only to end where you started),
  computes the scale, and saves it to flash - every sketch loads it
  automatically at NoU3.begin() from then on. Yaw only needs Z; stand the
  board on edge and spin again if you also want X or Y.
*/

#include <Alfredo_NoU3.h>

const float STILL_RAD_S = 0.03f;       // ~1.7 deg/s: counts as not moving
const float BIAS_MAX_RAD_S = 0.1f;     // ~6 deg/s: still enough to measure bias
const float SPIN_START_RAD_S = 0.5f;   // ~30 deg/s: the spin has clearly begun
const int BIAS_SAMPLES = 400;          // ~2 s of readings
const unsigned long END_STILL_MS = 5000;
const int MIN_TURNS = 3;

float scale[3] = {1.0f, 1.0f, 1.0f};
bool doneAxis[3] = {false, false, false};

void setup() {
  Serial.begin(115200);
  NoU3.begin();

  // NoU3.begin() loaded any scale saved by a past run; keep those axes.
  NoU3.fusion.getGyroScale(scale[0], scale[1], scale[2]);

  Serial.println("Gyro scale calibration. Align the board against a straightedge.");
}

void readGyro(float g[3]) {
  delay(2);  // fields refresh at 104 Hz in the background
  g[0] = NoU3.gyroscope_x;
  g[1] = NoU3.gyroscope_y;
  g[2] = NoU3.gyroscope_z;
}

void loop() {
  static const char axisName[3] = {'X', 'Y', 'Z'};
  float g[3];

  // Measure gyro bias over ~2 s of stillness (restarts if bumped), so the
  // spin integral below is drift-free even through re-grip pauses.
  Serial.println("\nHold still...");
  float bias[3];
  {
    double sum[3] = {0, 0, 0};
    int n = 0;
    while (n < BIAS_SAMPLES) {
      readGyro(g);
      if (fabsf(g[0]) > BIAS_MAX_RAD_S || fabsf(g[1]) > BIAS_MAX_RAD_S || fabsf(g[2]) > BIAS_MAX_RAD_S) {
        sum[0] = sum[1] = sum[2] = 0;
        n = 0;
        continue;
      }
      for (int i = 0; i < 3; i++) sum[i] += g[i];
      n++;
    }
    for (int i = 0; i < 3; i++) bias[i] = sum[i] / n;
  }

  Serial.println("Ready! Spin a whole number of turns (3+) about one axis; end aligned.");

  // Integrate all three axes with real elapsed time until the spin has
  // happened and the board has then been still for 5 s. Using wall-clock dt
  // (not the nominal sample period) keeps the IMU oscillator's rate error
  // out of this measurement - at runtime that error is corrected separately
  // from the measured interrupt timing, so the scale saved here must be
  // pure sensitivity.
  double totalRad[3] = {0, 0, 0};
  bool spinStarted = false;
  uint32_t lastUs = micros();
  unsigned long stillSinceMs = millis();
  while (true) {
    readGyro(g);
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    bool still = true;
    for (int i = 0; i < 3; i++) {
      float rate = g[i] - bias[i];
      totalRad[i] += rate * dt;
      if (fabsf(rate) > STILL_RAD_S) still = false;
      if (fabsf(rate) > SPIN_START_RAD_S) spinStarted = true;
    }
    if (!still) stillSinceMs = millis();
    if (spinStarted && millis() - stillSinceMs > END_STILL_MS) break;
  }

  // Which axis, how many turns, and did it end aligned?
  int dom = 0;
  if (fabs(totalRad[1]) > fabs(totalRad[dom])) dom = 1;
  if (fabs(totalRad[2]) > fabs(totalRad[dom])) dom = 2;
  float measuredDeg = fabsf(degrees((float)totalRad[dom]));
  int turns = (int)roundf(measuredDeg / 360.0f);
  float residualDeg = fabsf(measuredDeg - turns * 360.0f);

  if (turns < MIN_TURNS) {
    Serial.print("Only ");
    Serial.print(measuredDeg / 360.0f, 1);
    Serial.println(" turns measured - do at least 3. Trying again.");
    return;
  }
  if (residualDeg > 45.0f) {
    Serial.println("Didn't end near a whole number of turns - realign and try again.");
    return;
  }
  for (int i = 0; i < 3; i++) {
    if (i != dom && fabs(degrees((float)totalRad[i])) > 60.0f) {
      Serial.println("Too much rotation about a second axis - spin flatter and try again.");
      return;
    }
  }

  scale[dom] = turns * 360.0f / measuredDeg;
  doneAxis[dom] = true;
  NoU3.fusion.saveGyroScaleToFlash(scale[0], scale[1], scale[2]);

  Serial.print(axisName[dom]);
  Serial.print(": measured ");
  Serial.print(measuredDeg, 1);
  Serial.print(" deg over ");
  Serial.print(turns);
  Serial.print(" turns -> scale ");
  Serial.print(scale[dom], 5);
  Serial.println(" (saved to flash - sketches load it automatically at NoU3.begin())");
  Serial.println("(For reference, the equivalent manual call is:)");
  Serial.print("  NoU3.fusion.setGyroScale(");
  for (int i = 0; i < 3; i++) {
    Serial.print(scale[i], 5);
    Serial.print(i < 2 ? ", " : ");\n");
  }

  if (doneAxis[0] && doneAxis[1] && doneAxis[2]) {
    Serial.println("\nAll three axes calibrated - done!");
    while (true) delay(1000);
  }
  Serial.println("Spin about another axis to calibrate it too, or you're done (Z is the one yaw needs).");
}
