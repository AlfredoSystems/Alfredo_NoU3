#include "esp32-hal-ledc.h"
#include "Arduino.h"

#include "Alfredo_NoU3_LSM6.h"
#include "Alfredo_NoU3_MMC5.h"
#include "Alfredo_NoU3_PCA9.h"
#include "Alfredo_NoU3_encoder.h"

#include "Alfredo_NoU3.h"

PCA9685 pca9685;

LSM6Class LSM6;
MMC5983MAClass MMC5;

NoU_Agent NoU3;

// Task handles for stack analysis
TaskHandle_t lsm6_task_handle = NULL;
TaskHandle_t mmc5_task_handle = NULL;
TaskHandle_t service_light_task_handle = NULL;
TaskHandle_t monitor_task_handle = NULL;

// Spinlock protecting all public IMU data fields (acceleration, gyroscope,
// magnetometer, roll, pitch, yaw). Writers use portENTER_CRITICAL; readers
// that need a consistent multi-field snapshot should do the same.
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;

float fmap(float val, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min) return out_min;
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

volatile bool newDataAvailableLSM6 = true;
volatile bool newDataAvailableMMC5 = true;
void interruptRoutineLSM6()
{
    newDataAvailableLSM6 = true;
}
void interruptRoutineMMC5()
{
    newDataAvailableMMC5 = true;
}

void taskUpdateLSM6(void *pvParameters)
{
    while (true)
    {
        // Check LSM6 for new data
        if (newDataAvailableLSM6)
        {
            newDataAvailableLSM6 = false;
            NoU3.updateLSM6();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskUpdateMMC5(void *pvParameters)
{
    while (true)
    {
        // Check MMC5983MA for new data
        if (newDataAvailableMMC5)
        {
            newDataAvailableMMC5 = false;
            NoU3.updateMMC5();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskUpdateServiceLight(void *pvParameters)
{
    while (true)
    {
        NoU3.updateServiceLight();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// void taskMonitor(void *pvParameters)
// {
//     while (true)
//     {
//         // Periodically check the stack usage for all tasks
//         UBaseType_t lsm6_hwm = uxTaskGetStackHighWaterMark(lsm6_task_handle);
//         UBaseType_t mmc5_hwm = uxTaskGetStackHighWaterMark(mmc5_task_handle);
//         UBaseType_t sl_hwm = uxTaskGetStackHighWaterMark(service_light_task_handle);
//         Serial.printf("Stack HWM (words) - LSM6: %u, MMC5: %u, ServiceLight: %u\n", lsm6_hwm, mmc5_hwm, sl_hwm);
//         vTaskDelay(pdMS_TO_TICKS(2000)); // Check every 2 seconds
//     }
// }

// A warm reset can interrupt an I2C transaction, leaving a device holding SDA
// low. Clock SCL until it releases the line, then send a STOP.
static void recoverI2cBus(uint8_t sdaPin, uint8_t sclPin)
{
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    for (int i = 0; i < 9 && digitalRead(sdaPin) == LOW; i++)
    {
        digitalWrite(sclPin, LOW);
        delayMicroseconds(5);
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(5);
    }
    // STOP condition: SDA rising while SCL is high
    pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sdaPin, HIGH);
}

void NoU_Agent::begin()
{
    recoverI2cBus(PIN_I2C_SDA_QWIIC, PIN_I2C_SCL_QWIIC);
    recoverI2cBus(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU);

    Wire.begin(PIN_I2C_SDA_QWIIC, PIN_I2C_SCL_QWIIC, 400000);

    Wire1.begin(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU, 400000);
    beginMotors();
    beginIMUs();

    beginServiceLight();

    // Create a low-priority monitoring task to check stack usage
    // xTaskCreatePinnedToCore(taskMonitor, "taskMonitor", 2048, NULL, 1, &monitor_task_handle, 1);
}

void NoU_Agent::beginMotors()
{
    pca9685.setupSingleDevice(Wire1, 0x40);
    pca9685.setupOutputEnablePin(12);
    pca9685.enableOutputs(12);
    pca9685.setToFrequency(1526);
}

void NoU_Agent::beginIMUs()
{
    // Initialize LSM6
    if (LSM6.begin(Wire1) == false)
    {
        //Serial.println("LSM6 did not respond.");
    }
    else
    {
        pinMode(PIN_INTERRUPT_LSM6, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_LSM6), interruptRoutineLSM6, RISING);
        LSM6.enableInterrupt(); // LSM6 collects readings at 104 hz
        // Tasks are created once and never deleted (deleting a task that is
        // mid-I2C-transaction would leak the Wire1 bus mutex); re-calling
        // beginIMUs() re-initializes the sensor and reuses the task.
        if (lsm6_task_handle == NULL)
            xTaskCreatePinnedToCore(taskUpdateLSM6, "taskUpdateLSM6", 2048, NULL, 2, &lsm6_task_handle, 1);
    }

    // Initialize MMC5
    if (MMC5.begin(Wire1) == false)
    {
        //Serial.println("MMC5 did not respond.");
    }
    else
    {
        MMC5.softReset();
        MMC5.setFilterBandwidth(800);
        MMC5.setContinuousModeFrequency(100); // Allowed values are 1000, 200, 100, 50, 20, 10, 1 and 0
        MMC5.enableAutomaticSetReset();
        MMC5.enableContinuousMode();
        pinMode(PIN_INTERRUPT_MMC5, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_MMC5), interruptRoutineMMC5, RISING);
        MMC5.enableInterrupt();

        if (mmc5_task_handle == NULL)
            xTaskCreatePinnedToCore(taskUpdateMMC5, "taskUpdateMMC5", 2048, NULL, 2, &mmc5_task_handle, 1);
    }
}

bool NoU_Agent::updateLSM6()
{
    bool isNewData = false;

    if (LSM6.accelerationAvailable())
    {
        float ax, ay, az;
        LSM6.readAcceleration(&ax, &ay, &az); // result in Gs
        portENTER_CRITICAL(&imu_mux);
        acceleration_x = ax - acceleration_x_offset;
        acceleration_y = ay - acceleration_y_offset;
        acceleration_z = az - acceleration_z_offset;
        if (calibrationActive)
        {
            calSumAccelX += ax;
            calSumAccelY += ay;
            calSumAccelZ += az;
            calNumAccelSamples++;
        }
        portEXIT_CRITICAL(&imu_mux);

        isNewData = true;
    }

    if (LSM6.gyroscopeAvailable())
    {
        float gx, gy, gz;
        LSM6.readGyroscope(&gx, &gy, &gz); // Results in rad per second
        portENTER_CRITICAL(&imu_mux);
        gyroscope_x = gx - gyroscope_x_offset;
        gyroscope_y = gy - gyroscope_y_offset;
        gyroscope_z = gz - gyroscope_z_offset;
        if (calibrationActive)
        {
            calSumGyroX += gx;
            calSumGyroY += gy;
            calSumGyroZ += gz;
            calNumGyroSamples++;
        }
        updateAngles(); // called inside the lock — reads gyro fields, writes roll/pitch/yaw
        portEXIT_CRITICAL(&imu_mux);

        isNewData = true;
    }

    return isNewData;
}

bool NoU_Agent::updateMMC5()
{
    MMC5.clearMeasDoneInterrupt();

    float mx, my, mz;
    bool isNewData = MMC5.readMagnetometer(&mx, &my, &mz); // Results in µT (microteslas)

    if (isNewData)
    {
        portENTER_CRITICAL(&imu_mux);
        magnetometer_x = mx;
        magnetometer_y = my;
        magnetometer_z = mz;
        portEXIT_CRITICAL(&imu_mux);
    }

    return isNewData;
}

void NoU_Agent::updateAngles()
{
    // micros(), not millis(): samples arrive every ~9.6 ms (104 Hz), so
    // millisecond resolution would put ~10% jitter on each timestep.
    static bool firstSample = true;
    static unsigned long lastTimeUs = 0;

    unsigned long currentTimeUs = micros();

    if (firstSample)
    {
        firstSample = false;
        lastTimeUs = currentTimeUs;
        return;
    }

    float timestep = (currentTimeUs - lastTimeUs) / 1000000.0; // convert us to seconds
    lastTimeUs = currentTimeUs;

    float deltaPitch = gyroscope_x * timestep;
    float deltaRoll = gyroscope_y * timestep;
    float deltaYaw = gyroscope_z * timestep;

    pitch += deltaPitch;
    roll += deltaRoll;
    yaw += deltaYaw;
}

void NoU_Agent::calibrateIMUs(float gravity_x, float gravity_y, float gravity_z)
{
    // The background task stays the only reader of the sensor. While
    // calibrationActive is set, updateLSM6() accumulates raw readings into
    // the calSum* fields; we just wait a second and average them.
    portENTER_CRITICAL(&imu_mux);
    calSumAccelX = calSumAccelY = calSumAccelZ = 0;
    calSumGyroX = calSumGyroY = calSumGyroZ = 0;
    calNumAccelSamples = 0;
    calNumGyroSamples = 0;
    calibrationActive = true;
    portEXIT_CRITICAL(&imu_mux);

    vTaskDelay(pdMS_TO_TICKS(1000));

    portENTER_CRITICAL(&imu_mux);
    calibrationActive = false;
    if (calNumAccelSamples > 0)
    {
        acceleration_x_offset = (calSumAccelX / calNumAccelSamples) - gravity_x;
        acceleration_y_offset = (calSumAccelY / calNumAccelSamples) - gravity_y;
        acceleration_z_offset = (calSumAccelZ / calNumAccelSamples) - gravity_z;
    }
    if (calNumGyroSamples > 0)
    {
        gyroscope_x_offset = calSumGyroX / calNumGyroSamples;
        gyroscope_y_offset = calSumGyroY / calNumGyroSamples;
        gyroscope_z_offset = calSumGyroZ / calNumGyroSamples;
    }
    roll = 0;
    pitch = 0;
    yaw = 0;
    portEXIT_CRITICAL(&imu_mux);
}

void NoU_Agent::stopMotors()
{
    pca9685.setAllChannelsDutyCycle(0);
}

void NoU_Agent::beginServiceLight()
{
    if (service_light_task_handle != NULL)
    {
        vTaskDelete(service_light_task_handle);
        service_light_task_handle = NULL;
    }

    ledcAttachChannel(RSL_PIN, RSL_PWM_FREQ, RSL_PWM_RES, RSL_CHANNEL);
    setServiceLight(LIGHT_DISABLED);
    updateServiceLight();

    xTaskCreatePinnedToCore(taskUpdateServiceLight, "taskUpdateServiceLight", 1024, NULL, 2, &service_light_task_handle, 1);
}

void NoU_Agent::setServiceLight(serviceLightState state)
{
    stateServiceLight = state;
}

void NoU_Agent::updateServiceLight()
{
    int dutyLED = 0;

    switch (stateServiceLight)
    {
    case LIGHT_OFF:
        dutyLED = 1;
        break;
    case LIGHT_ON:
        dutyLED = (1 << RSL_PWM_RES) - 1;
        break;
    case LIGHT_ENABLED: {
        unsigned long t = millis() % 1000;
        dutyLED = (t < 500) ? (int)(t * 2) : (int)((1000 - t) * 2);
        break;
    }
    case LIGHT_DISABLED:
        dutyLED = (1 << RSL_PWM_RES) - 1;
        break;
    case LIGHT_CALIBRATING:
        dutyLED = (millis() % 100 < 40) ? (1 << RSL_PWM_RES) - 1 : 1;
        break;
    }
    // Serial.println(dutyLED);
    ledcWrite(RSL_PIN, dutyLED);
}

NoU_Motor::NoU_Motor(uint8_t motorPort)
{
    this->motorPort = motorPort;
    set(0);
}

void NoU_Motor::beginEncoder(int8_t pinA, int8_t pinB)
{
    // the 6 pin pairs are for M2-M7. M1 and M8 do not have encoders
    const uint8_t encoderPinMap[6][2] = {
        {17, 18}, // M2, E2
        {15, 16}, // M3, E3
        {10, 11}, // M4, E5
        {41, 42}, // M5, E4
        {40, 39}, // M6, E6
        {38, 37}  // M7, E1
    };

    if (pinA == -1 || pinB == -1)
    {
        if (this->motorPort < 2 || this->motorPort > 7)
            return;

        if (pinA == -1) pinA = encoderPinMap[this->motorPort - 2][0];
        if (pinB == -1) pinB = encoderPinMap[this->motorPort - 2][1];
    }

    this->_encoder.begin(pinA, pinB);
}

int32_t NoU_Motor::getPosition()
{
    return this->_encoder.getPosition();
}

void NoU_Motor::resetPosition()
{
    this->_encoder.resetPosition();
}

void NoU_Motor::set(float output)
{
    if (this->motorPort < 1 || this->motorPort > 8) return;

    uint8_t portMap[8][2] = {{4, 5}, {6, 7}, {8, 9}, {10, 11}, {14, 15}, {12, 13}, {2, 3}, {0, 1}};
    // needs to be changed to this at some point
    // uint8_t portMap[8][2] = {{5, 4}, {7, 6}, {9, 8}, {11, 10}, {14, 15}, {12, 13}, {2, 3}, {0, 1}};

    float motorPower = applyCurve(output);
    if (inverted)
        motorPower = motorPower * -1;

    float pinZeroDuty = 0;
    float pinOneDuty = 0;

    if (brakeMode)
    {
        if (motorPower <= 0)
        {
            pinZeroDuty = ((abs(motorPower) * -1) + 1) * 100;
            pinOneDuty = 100;
        }
        else
        {
            pinZeroDuty = 100;
            pinOneDuty = ((abs(motorPower) * -1) + 1) * 100;
        }
    }
    else
    {
        if (motorPower >= 0)
        {
            pinZeroDuty = abs(motorPower * 100);
            pinOneDuty = 0;
        }
        else
        {
            pinZeroDuty = 0;
            pinOneDuty = abs(motorPower * 100);
        }
    }

    pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][0], pinZeroDuty);
    pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][1], pinOneDuty);
}

float NoU_Motor::applyCurve(float input) const
{
    float sign = (input == 0) ? 0 : (input > 0 ? 1 : -1);
    float x = fabs(input);

    float range = 1.0f - deadband;
    if (x < deadband || range <= 0.0f)
        return 0;

    float curved = pow(max(fmap(constrain(x, 0, 1), deadband, 1, 0, 1), 0.0f), exponent);
    float output = fmap(curved, 0, 1, minimumOutput, maximumOutput);

    return output * sign;
}

void NoU_Motor::setMotorCurve(float minimumOutput, float maximumOutput, float deadband, float exponent)
{
    setMinimumOutput(minimumOutput);
    setMaximumOutput(maximumOutput);
    setDeadband(deadband);
    setExponent(exponent);
}

void NoU_Motor::setMinimumOutput(float minimumOutput)
{
    this->minimumOutput = constrain(minimumOutput, 0, maximumOutput);
}

void NoU_Motor::setMaximumOutput(float maximumOutput)
{
    this->maximumOutput = constrain(maximumOutput, minimumOutput, 1);
}

void NoU_Motor::setDeadband(float deadband)
{
    deadband = constrain(deadband, 0, 1);
    this->deadband = deadband;
}

void NoU_Motor::setExponent(float exponent)
{
    exponent = max(0.0f, exponent);
    this->exponent = exponent;
}

void NoU_Motor::setInverted(bool isInverted)
{
    this->inverted = isInverted;
}

void NoU_Motor::setBrakeMode(bool isBrakeMode)
{
    this->brakeMode = isBrakeMode;
}

NoU_Servo::NoU_Servo(uint8_t servoPort, uint16_t minPulse, uint16_t maxPulse)
{
    this->minPulse = minPulse;
    this->maxPulse = maxPulse;

    if (servoPort < 1 || servoPort > 6) return;

    switch (servoPort)
    {
    case 1: pin = PIN_SERVO_1; channel = SERVO_1_CHANNEL; break;
    case 2: pin = PIN_SERVO_2; channel = SERVO_2_CHANNEL; break;
    case 3: pin = PIN_SERVO_3; channel = SERVO_3_CHANNEL; break;
    case 4: pin = PIN_SERVO_4; channel = SERVO_4_CHANNEL; break;
    case 5: pin = PIN_SERVO_5; channel = SERVO_5_CHANNEL; break;
    case 6: pin = PIN_SERVO_6; channel = SERVO_6_CHANNEL; break;
    }
    ledcAttachChannel(pin, SERVO_PWM_FREQ, SERVO_PWM_RES, channel);
}

void NoU_Servo::write(float degrees)
{
    writeMicroseconds(fmap(degrees, 0, 180, minPulse, maxPulse));
}

void NoU_Servo::writeMicroseconds(uint16_t pulseLength)
{
    this->pulse = pulseLength;
    ledcWrite(pin, fmap(pulseLength, 0, 20000, 0, (1 << SERVO_PWM_RES) - 1));
}

void NoU_Servo::setMinimumPulse(uint16_t minPulse)
{
    this->minPulse = minPulse;
}

void NoU_Servo::setMaximumPulse(uint16_t maxPulse)
{
    this->maxPulse = maxPulse;
}

uint16_t NoU_Servo::getMicroseconds()
{
    return pulse;
}

float NoU_Servo::getDegrees()
{
    return fmap(pulse, minPulse, maxPulse, 0, 180);
}

NoU_Drivetrain::NoU_Drivetrain(NoU_Motor *leftMotor, NoU_Motor *rightMotor)
    : frontLeftMotor(leftMotor), frontRightMotor(rightMotor),
      rearLeftMotor(nullptr), rearRightMotor(nullptr), drivetrainType(DRIVE_TWO_MOTORS)
{
}

NoU_Drivetrain::NoU_Drivetrain(NoU_Motor *frontLeftMotor, NoU_Motor *frontRightMotor,
                               NoU_Motor *rearLeftMotor, NoU_Motor *rearRightMotor)
    : frontLeftMotor(frontLeftMotor), frontRightMotor(frontRightMotor),
      rearLeftMotor(rearLeftMotor), rearRightMotor(rearRightMotor), drivetrainType(DRIVE_FOUR_MOTORS)
{
}

void NoU_Drivetrain::setMotors(float frontLeftPower, float frontRightPower, float rearLeftPower, float rearRightPower)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->set(rearLeftPower);
        rearRightMotor->set(rearRightPower);
    }
    frontLeftMotor->set(frontLeftPower);
    frontRightMotor->set(frontRightPower);
}

void NoU_Drivetrain::tankDrive(float leftPower, float rightPower)
{
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::arcadeDrive(float throttle, float rotation, bool invertedReverse)
{
    float leftPower = 0;
    float rightPower = 0;
    float maxInput = (throttle > 0 ? 1 : -1) * max(fabs(throttle), fabs(rotation));
    if (throttle > 0)
    {
        if (rotation > 0)
        {
            leftPower = maxInput;
            rightPower = throttle - rotation;
        }
        else
        {
            leftPower = throttle + rotation;
            rightPower = maxInput;
        }
    }
    else
    {
        if (rotation > 0)
        {
            leftPower = invertedReverse ? maxInput : throttle + rotation;
            rightPower = invertedReverse ? throttle + rotation : maxInput;
        }
        else
        {
            leftPower = invertedReverse ? throttle - rotation : maxInput;
            rightPower = invertedReverse ? maxInput : throttle - rotation;
        }
    }
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::curvatureDrive(float throttle, float rotation, bool isQuickTurn)
{
    float angularPower;
    bool overPower;

    if (isQuickTurn)
    {
        if (fabs(throttle) < quickStopThreshold)
        {
            quickStopAccumulator = (1 - quickStopAlpha) * quickStopAccumulator + quickStopAlpha * rotation * 2;
        }
        overPower = true;
        angularPower = rotation;
    }
    else
    {
        overPower = false;
        angularPower = fabs(throttle) * rotation - quickStopAccumulator;

        if (quickStopAccumulator > 1)
            quickStopAccumulator--;
        else if (quickStopAccumulator < -1)
            quickStopAccumulator++;
        else
            quickStopAccumulator = 0;
    }

    float leftPower;
    float rightPower;

    leftPower = throttle + angularPower;
    rightPower = throttle - angularPower;

    if (overPower)
    {
        if (leftPower > 1)
        {
            rightPower -= leftPower - 1;
            leftPower = 1;
        }
        else if (rightPower > 1)
        {
            leftPower -= rightPower - 1;
            rightPower = 1;
        }
        else if (leftPower < -1)
        {
            rightPower -= leftPower + 1;
            leftPower = -1;
        }
        else if (rightPower < -1)
        {
            leftPower -= rightPower + 1;
            rightPower = -1;
        }
    }
    float maxMagnitude = max(fabs(leftPower), fabs(rightPower));
    if (maxMagnitude > 1)
    {
        leftPower /= maxMagnitude;
        rightPower /= maxMagnitude;
    }
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::holonomicDrive(float latest_xVelocity, float latest_yVelocity, float latest_rotation, bool plusConfig)
{
    static float xVelocity = 0;
    static float yVelocity = 0;
    static float rotation = 0;

    if (!isnan(latest_xVelocity)) xVelocity = latest_xVelocity;
    if (!isnan(latest_yVelocity)) yVelocity = latest_yVelocity;
    if (!isnan(latest_rotation)) rotation = latest_rotation;

    if (drivetrainType == DRIVE_TWO_MOTORS)
        return;

    float frontLeftPower = 0;
    float frontRightPower = 0;
    float rearLeftPower = 0;
    float rearRightPower = 0;

    if (plusConfig)
    {
        frontLeftPower = yVelocity - rotation;
        frontRightPower = xVelocity - rotation;
        rearLeftPower = -xVelocity - rotation;
        rearRightPower = -yVelocity - rotation;
    }
    else
    { // X config
        frontLeftPower = xVelocity + yVelocity - rotation;
        frontRightPower = xVelocity - yVelocity - rotation;
        rearLeftPower = -xVelocity + yVelocity - rotation;
        rearRightPower = -xVelocity - yVelocity - rotation;
    }

    float maxMagnitude = max(fabs(frontLeftPower), max(fabs(frontRightPower), max(fabs(rearLeftPower), fabs(rearRightPower))));
    if (maxMagnitude > 1)
    {
        frontLeftPower /= maxMagnitude;
        frontRightPower /= maxMagnitude;
        rearLeftPower /= maxMagnitude;
        rearRightPower /= maxMagnitude;
    }

    setMotors(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
}

void NoU_Drivetrain::setMotorCurves(float minimumOutput, float maximumOutput, float deadband, float exponent)
{
    setMinimumOutput(minimumOutput);
    setMaximumOutput(maximumOutput);
    setDeadband(deadband);
    setExponent(exponent);
}

void NoU_Drivetrain::setMinimumOutput(float minimumOutput)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setMinimumOutput(minimumOutput);
        rearRightMotor->setMinimumOutput(minimumOutput);
    }
    frontLeftMotor->setMinimumOutput(minimumOutput);
    frontRightMotor->setMinimumOutput(minimumOutput);
}

void NoU_Drivetrain::setMaximumOutput(float maximumOutput)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setMaximumOutput(maximumOutput);
        rearRightMotor->setMaximumOutput(maximumOutput);
    }
    frontLeftMotor->setMaximumOutput(maximumOutput);
    frontRightMotor->setMaximumOutput(maximumOutput);
}

void NoU_Drivetrain::setExponent(float exponent)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setExponent(exponent);
        rearRightMotor->setExponent(exponent);
    }
    frontLeftMotor->setExponent(exponent);
    frontRightMotor->setExponent(exponent);
}

void NoU_Drivetrain::setDeadband(float deadband)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setDeadband(deadband);
        rearRightMotor->setDeadband(deadband);
    }
    frontLeftMotor->setDeadband(deadband);
    frontRightMotor->setDeadband(deadband);
}
