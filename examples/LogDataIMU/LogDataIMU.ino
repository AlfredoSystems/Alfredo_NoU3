/*
  LogDataIMU

  Records IMU data to a CSV file on a microSD card. Each row has the micros()
  timestamp, the gyroscope (rad/s), the accelerometer (g), and the VQF fused
  orientation - roll, pitch, and yaw (rad). One row is written per IMU sample
  (~104 Hz), and every recording goes to a new file: IMU_0001.CSV,
  IMU_0002.CSV, ...

  Wiring (SPI microSD module):
    MISO -> GPIO 5
    SCK  -> GPIO 6
    MOSI -> GPIO 7
    CS   -> GPIO 8 (held high at startup, until the SD library takes over)
    GPIO 4 is held low.

  How to use:
  - Insert a FAT32-formatted microSD card. Cards over 32 GB usually come
    formatted as exFAT and must be reformatted to FAT32.
  - Press BOOT to start recording. The LED flashes at 3 Hz while recording.
  - Press BOOT again to stop. The LED turns off and the file is saved.
  - If the card fills up, recording stops and the LED stays solid until you
    press BOOT.
  - Keep the robot still for a moment at power-up: NoU3.calibrateIMUs() waits
    for it to be at rest before zeroing yaw.
*/

#include <Alfredo_NoU3.h>
#include <SD.h>
#include <SPI.h>

const int PIN_SD_MISO = 5;
const int PIN_SD_SCK = 6;
const int PIN_SD_MOSI = 7;
const int PIN_SD_CS = 8;
const int PIN_HOLD_LOW = 4;
const int PIN_BOOT_BUTTON = 0;

const unsigned long DEBOUNCE_MS = 50;
const unsigned long FLASH_PERIOD_MS = 1000 / 3;    // 3 Hz
const unsigned long CARD_CHECK_MS = 1000;
const unsigned long FLUSH_INTERVAL_MS = 2000;      // bounds data lost if power is cut
const int SAMPLE_QUEUE_LENGTH = 1024;              // ~10 s of samples

struct ImuSample {
  uint32_t micros;
  float gx, gy, gz;
  float ax, ay, az;
  float roll, pitch, yaw;
};

enum LoggerState { IDLE, RECORDING, CARD_FULL };

LoggerState state = IDLE;
bool cardMounted = false;
File logFile;
QueueHandle_t sampleQueue;
volatile bool sampling = false;
unsigned long lastCardCheckMs = 0;
unsigned long lastFlushMs = 0;

// Writing to the card can block for several milliseconds, so samples are
// captured here and queued for loop() to write. The library publishes each
// sample's raw readings first and its fused angles last, so a change in the
// angles marks a complete new sample.
void taskSampleIMU(void *pvParameters) {
  float lastRoll = NAN, lastPitch = NAN, lastYaw = NAN;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1));

    float roll = NoU3.roll, pitch = NoU3.pitch, yaw = NoU3.yaw;
    if (roll == lastRoll && pitch == lastPitch && yaw == lastYaw) continue;
    lastRoll = roll;
    lastPitch = pitch;
    lastYaw = yaw;

    if (!sampling) continue;

    ImuSample sample = {
        micros(),
        NoU3.gyroscope_x, NoU3.gyroscope_y, NoU3.gyroscope_z,
        NoU3.acceleration_x, NoU3.acceleration_y, NoU3.acceleration_z,
        roll, pitch, yaw,
    };
    // If the card stalls long enough to fill the queue, samples are
    // dropped; the gap shows up in the micros column.
    xQueueSend(sampleQueue, &sample, 0);
  }
}

void setup() {
  Serial.begin(115200);
  NoU3.begin();
  NoU3.setServiceLight(LIGHT_OFF);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

  pinMode(PIN_HOLD_LOW, OUTPUT);
  digitalWrite(PIN_HOLD_LOW, LOW);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  NoU3.calibrateIMUs();

  sampleQueue = xQueueCreate(SAMPLE_QUEUE_LENGTH, sizeof(ImuSample));
  xTaskCreatePinnedToCore(taskSampleIMU, "taskSampleIMU", 2048, NULL, 2, NULL, 1);
}

void loop() {
  // The module has no card-detect pin, so check for the card once a second
  // while not recording: mount one that was inserted, and unmount one that
  // was removed so it can be mounted again when reinserted.
  if (state != RECORDING && millis() - lastCardCheckMs >= CARD_CHECK_MS) {
    lastCardCheckMs = millis();
    if (!cardMounted) {
      cardMounted = SD.begin(PIN_SD_CS);
      if (cardMounted) Serial.println("SD card mounted");
    } else if (!cardStillPresent()) {
      unmountCard();
    }
  }

  if (bootButtonPressed()) {
    if (state == IDLE)
      startRecording();
    else
      stopRecording();
  }

  if (state == RECORDING) {
    writeQueuedSamples();
    if (state == RECORDING && millis() - lastFlushMs >= FLUSH_INTERVAL_MS) {
      lastFlushMs = millis();
      logFile.flush();
    }
  }

  updateLight();
  delay(1);
}

bool bootButtonPressed() {
  static bool lastDown = false;
  static unsigned long lastChangeMs = 0;

  bool down = digitalRead(PIN_BOOT_BUTTON) == LOW;
  if (down == lastDown || millis() - lastChangeMs < DEBOUNCE_MS) return false;
  lastDown = down;
  lastChangeMs = millis();
  return down;
}

// Opening the root directory makes the card answer a status command, which
// fails once it has been pulled out.
bool cardStillPresent() {
  File root = SD.open("/");
  bool present = root;
  root.close();
  return present;
}

void unmountCard() {
  SD.end();
  cardMounted = false;
  Serial.println("SD card removed");
}

void startRecording() {
  if (!cardMounted) {
    Serial.println("No SD card - insert one to record");
    return;
  }

  char path[16];
  if (!nextLogPath(path, sizeof(path))) return;

  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) {
    Serial.printf("Could not create %s\n", path);
    return;
  }
  logFile.setBufferSize(4096);

  const char header[] = "micros,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_g,accel_y_g,accel_z_g,roll_rad,pitch_rad,yaw_rad\n";
  if (logFile.print(header) != strlen(header)) {
    logFile.close();
    state = CARD_FULL;
    return;
  }

  xQueueReset(sampleQueue);
  lastFlushMs = millis();
  sampling = true;
  state = RECORDING;
  Serial.printf("Recording to %s\n", path);
}

void stopRecording() {
  sampling = false;
  if (state == RECORDING) {
    writeQueuedSamples();
    logFile.close();
    Serial.println("Recording stopped");
  }
  state = IDLE;
}

// Picks the first unused name: /IMU_0001.CSV, /IMU_0002.CSV, ...
bool nextLogPath(char *path, size_t size) {
  for (int i = 1; i <= 9999; i++) {
    snprintf(path, size, "/IMU_%04d.CSV", i);
    if (!SD.exists(path)) return true;
  }
  return false;
}

void writeQueuedSamples() {
  ImuSample s;
  char line[160];
  while (state == RECORDING && xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
    int len = snprintf(line, sizeof(line), "%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                       (unsigned long)s.micros, s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.roll, s.pitch, s.yaw);

    if (logFile.write((const uint8_t *)line, len) != (size_t)len) {
      // A failed write means the card is either full or was pulled out.
      sampling = false;
      logFile.close();
      if (cardStillPresent()) {
        state = CARD_FULL;
        Serial.println("SD card full - press BOOT to clear");
      } else {
        unmountCard();
        state = IDLE;
      }
    }
  }
}

void updateLight() {
  switch (state) {
    case IDLE:
      NoU3.setServiceLight(LIGHT_OFF);
      break;
    case RECORDING:
      NoU3.setServiceLight(millis() % FLASH_PERIOD_MS < FLASH_PERIOD_MS / 2 ? LIGHT_ON : LIGHT_OFF);
      break;
    case CARD_FULL:
      NoU3.setServiceLight(LIGHT_ON);
      break;
  }
}
