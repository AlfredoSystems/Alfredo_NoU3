/*
  CalibrateGyroScale  (OPTIONAL - NoU3 orientation works fine without this)

  One-time gyro sensitivity calibration, guided over the PestoLink terminal
  (everything is mirrored to USB Serial too). The chip's factory trim is
  within ~1%, which shows up as yaw reading short or long by a few degrees
  per full rotation. No online estimator can see this error in 6DOF, so a
  measured rotation is the only fix. Sensitivity is stable over time and
  temperature, so once is enough.

  How: connect with PestoLink and follow the prompts. Set the robot on a
  start mark, press BOOT, spin it 5 full turns in place, set it back
  EXACTLY on the mark, and hold still - the sketch notices the rest and
  finishes on its own. It detects which gyro axis you spun (mount your
  NoU3 any way you like), computes the scale, and saves it to flash -
  every sketch loads it automatically at NoU3.begin().

  Gyro bias comes from VQF's automatic rest-time estimation (it converges
  while the robot sits on the mark), so there is no separate hold-still
  measurement step.

  RSL light:  solid      = waiting for PestoLink
              slow fade  = waiting for you to press BOOT
              fast blink = hold still a moment (bias not settled yet)
              solid      = spin phase - go!
*/

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

const int EXPECTED_TURNS = 5;
const float NO_SPIN_TURNS = 0.5f;        // less than this = you didn't spin yet
const float TURN_TOLERANCE_DEG = 90.0f;  // a 5% gyro error over 5 turns is 90 deg
const float CROSS_AXIS_LIMIT_DEG = 60.0f;
const float BIAS_SIGMA_OK_RAD_S = 0.0017f;  // ~0.1 deg/s: bias estimate has settled
const float STILL_RAD_S = 0.03f;         // ~1.7 deg/s: gyro counts as not moving
const unsigned long END_REST_MS = 2000;  // this long at rest ends the measurement

const int PIN_BOOT_BUTTON = 0;

const char axisName[3] = {'X', 'Y', 'Z'};

// ---------------------------------------------------------------------------
// Printing: every message goes to both USB Serial and the PestoLink terminal.
// PestoLink silently DROPS terminal lines sent within 200 ms of the previous
// one, so say() paces itself (fine everywhere except the integration loop) and
// sayLive() is for inside measurement loops - call it at most ~1x/second.
// ---------------------------------------------------------------------------
unsigned long lastSayMs = 0;

void sayLive(const char *fmt, ...) {
  char line[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  Serial.println(line);
  PestoLink.printTerminal(line);
  lastSayMs = millis();
}

void say(const char *fmt, ...) {
  char line[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  while (millis() - lastSayMs < 220) delay(5);  // don't outrun the rate limit
  Serial.println(line);
  PestoLink.printTerminal(line);
  lastSayMs = millis();
}

// Debounced BOOT press edge. Call it often; returns true once per press.
bool bootPressed() {
  static bool lastDown = false;
  static unsigned long lastChangeMs = 0;
  bool down = digitalRead(PIN_BOOT_BUTTON) == LOW;
  if (down != lastDown && millis() - lastChangeMs > 30) {
    lastDown = down;
    lastChangeMs = millis();
    if (down) return true;
  }
  return false;
}

void waitForBoot() {
  NoU3.setServiceLight(LIGHT_ENABLED);  // slow fade: your move
  while (!bootPressed()) delay(5);
}

void readGyro(float g[3]) {
  delay(2);  // fields refresh at 104 Hz in the background
  g[0] = NoU3.gyroscope_x;
  g[1] = NoU3.gyroscope_y;
  g[2] = NoU3.gyroscope_z;
}

void setup() {
  Serial.begin(115200);
  PestoLink.begin("NoU3_GyroCal");
  NoU3.begin();
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

  // Solid light until PestoLink connects (BOOT skips this for Serial-only use).
  NoU3.setServiceLight(LIGHT_DISABLED);
  Serial.println("Connect with PestoLink to \"NoU3_GyroCal\"");
  Serial.println("(or press BOOT to run with Serial prompts only)");
  while (!PestoLink.isConnected() && !bootPressed()) delay(10);
  delay(300);  // let the terminal come up before greeting it

  say("== Gyro calibration ==");
  say(".");
  say("Set the robot on a start mark, then press BOOT.");
}

void loop() {
  float g[3];

  waitForBoot();

  // ---- Gyro bias, courtesy of VQF's automatic rest-time estimation ----
  // While the robot has been sitting on its mark, the fusion filter has
  // already been measuring the bias. Usually this passes instantly; the
  // blink phase only appears if the robot was just moved (or just booted).
  if (!(NoU3.fusion.isResting() && NoU3.fusion.getGyroBiasSigma() < BIAS_SIGMA_OK_RAD_S)) {
    NoU3.setServiceLight(LIGHT_CALIBRATING);  // fast blink: hands off
    say(".");
    say("Hold still a moment...");
    while (!(NoU3.fusion.isResting() && NoU3.fusion.getGyroBiasSigma() < BIAS_SIGMA_OK_RAD_S)) {
      delay(20);
    }
  }

  // VQF works in scale-corrected units (it applies the current gyro scale
  // to raw samples before estimating bias), so integrate the same way and
  // apply this run's result as a correction ON TOP of the current scale.
  // That also makes a rerun a true check: it should report ~0.0% off.
  float bias[3], scale[3];
  NoU3.fusion.getGyroBias(bias[0], bias[1], bias[2]);
  NoU3.fusion.getGyroScale(scale[0], scale[1], scale[2]);

  // ---- The spin --------------------------------------------------------
  NoU3.setServiceLight(LIGHT_ON);  // solid: go
  say(".");
  say("Spin the robot %d full turns", EXPECTED_TURNS);
  say(".");
  say("End back on the mark and let go,");
  say("the NoU3 will detect when you are done.");

  // Integrate all three axes with real elapsed time until the robot has
  // clearly spun and then come back to rest. Using wall-clock dt (not the
  // nominal sample period) keeps the IMU oscillator's rate error out of
  // this measurement - at runtime that error is corrected separately from
  // the measured interrupt timing, so the scale saved here must be pure
  // sensitivity. (BOOT restarts, in case you lose count mid-spin.)
  double totalRad[3] = {0, 0, 0};
  bool spinStarted = false;
  bool cancelled = false;
  uint32_t lastUs = micros();
  unsigned long lastStatusMs = millis();
  unsigned long restSinceMs = millis();
  unsigned long gyroQuietSinceMs = millis();
  while (true) {
    readGyro(g);
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    for (int i = 0; i < 3; i++) totalRad[i] += (g[i] * scale[i] - bias[i]) * dt;

    int dom = 0;
    if (fabs(totalRad[1]) > fabs(totalRad[dom])) dom = 1;
    if (fabs(totalRad[2]) > fabs(totalRad[dom])) dom = 2;
    if (fabs(totalRad[dom]) > NO_SPIN_TURNS * 2.0 * PI) spinStarted = true;

    // "Done" = the robot went quiet and stayed quiet. VQF's rest detector
    // is the nice signal, but it also watches the accelerometer, and a
    // hand steadying the robot has enough tremor to hold it off - so a
    // plain gyro-quiet check (immune to that) can end the wait too.
    for (int i = 0; i < 3; i++) {
      if (fabsf(g[i] * scale[i] - bias[i]) > STILL_RAD_S) gyroQuietSinceMs = millis();
    }
    if (!NoU3.fusion.isResting()) restSinceMs = millis();
    if (spinStarted && (millis() - restSinceMs > END_REST_MS ||
                        millis() - gyroQuietSinceMs > END_REST_MS)) break;

    if (millis() - lastStatusMs > 2000) {
      sayLive("Turns: %.1f of %d", fabs(totalRad[dom]) / (2.0 * PI), EXPECTED_TURNS);
      // Rest-detector internals, Serial only, for debugging stuck runs:
      // relative deviations >= 1.0 are what's vetoing VQF rest.
      float devG, devA;
      NoU3.fusion.getRestDeviations(devG, devA);
      Serial.printf("  [debug] rest=%d devGyro=%.2f devAccel=%.2f\n",
                    NoU3.fusion.isResting(), devG, devA);
      lastStatusMs = millis();
    }
    if (bootPressed()) {
      cancelled = true;
      break;
    }
  }
  if (cancelled) {
    say(".");
    say("Restarting.");
    return;
  }

  // ---- Which axis was it, and was it really 5 turns? -------------------
  int dom = 0;
  if (fabs(totalRad[1]) > fabs(totalRad[dom])) dom = 1;
  if (fabs(totalRad[2]) > fabs(totalRad[dom])) dom = 2;
  float measuredDeg = fabsf(degrees((float)totalRad[dom]));

  if (fabsf(measuredDeg - EXPECTED_TURNS * 360.0f) > TURN_TOLERANCE_DEG) {
    say(".");
    say("That didn't look like %d turns", EXPECTED_TURNS);
    say("ending on the mark (saw %.1f).", measuredDeg / 360.0f);
    say(".");
    say("Press BOOT to restart.");
    return;
  }
  bool crossAxis = false;
  for (int i = 0; i < 3; i++) {
    if (i != dom && fabs(degrees((float)totalRad[i])) > CROSS_AXIS_LIMIT_DEG) crossAxis = true;
  }
  if (crossAxis) {
    say(".");
    say("Too much wobble - keep it flat.");
    say(".");
    say("Press BOOT to restart.");
    return;
  }

  // ---- Success: save and report ---------------------------------------
  scale[dom] *= EXPECTED_TURNS * 360.0f / measuredDeg;
  NoU3.fusion.saveGyroScaleToFlash(scale[0], scale[1], scale[2]);

  say(".");
  say("Done! Gyro %c was %+.2f%% off.", axisName[dom],
      (measuredDeg / (EXPECTED_TURNS * 360.0f) - 1.0f) * 100.0f);
  say("Calibration value saved to flash.");
  say(".");
  say("You can now upload your robot code.");

  NoU3.setServiceLight(LIGHT_OFF);
  while (true) delay(1000);  // finished - reset or reflash to run again
}
