#pragma once

#include <inttypes.h>

#include "Alfredo_NoU3_LSM6.h"
#include "Alfredo_NoU3_MMC5.h"
#include "Alfredo_NoU3_PCA9.h"
#include "Alfredo_NoU3_VQF.h"
#include "Alfredo_NoU3_encoder.h"

const int PIN_SNS_VERSION = 1;
const int PIN_SNS_VIN = 2;

const int PIN_SERVO_1 = 4;
const int PIN_SERVO_2 = 5;
const int PIN_SERVO_3 = 6;
const int PIN_SERVO_4 = 7;
const int PIN_SERVO_5 = 8;
const int PIN_SERVO_6 = 9;

const int PIN_I2C_SDA_QWIIC = 33;
const int PIN_I2C_SCL_QWIIC = 34;
const int PIN_I2C_SDA_IMU = 35;
const int PIN_I2C_SCL_IMU = 36;

const int RSL_PIN = 45;

const int PIN_INTERRUPT_LSM6 = 48;
const int PIN_INTERRUPT_MMC5 = 47;

// PWM Configuration
const int SERVO_PWM_RES = 12; // bits
const int SERVO_PWM_FREQ = 50; // Hz
const int RSL_PWM_RES = 10; // bits
const int RSL_PWM_FREQ = 1000; // Hz

const int RSL_CHANNEL = 1;
const int SERVO_1_CHANNEL = 2;
const int SERVO_2_CHANNEL = 3;
const int SERVO_3_CHANNEL = 4;
const int SERVO_4_CHANNEL = 5;
const int SERVO_5_CHANNEL = 6;
const int SERVO_6_CHANNEL = 7;

typedef enum {
    LIGHT_OFF,
    LIGHT_ON,
    LIGHT_DISABLED,
    LIGHT_ENABLED,
    LIGHT_CALIBRATING
} serviceLightState;

typedef enum {
    DRIVE_TWO_MOTORS = 0,
    DRIVE_FOUR_MOTORS = 1
} DrivetrainType;

class NoU_Agent {
    public:
		void begin();

		void beginMotors();
		void beginIMUs();
        bool updateIMUs() __attribute__((unavailable("Delete this function call, this function is obsolete and removed. IMU updates are now automatic.")));
        bool updateLSM6();
        bool updateMMC5();

        void updateAngles();

        // Blocks until the robot has been still long enough for the fusion
        // filter to detect rest and measure the gyro bias (~1.5 s), then
        // zeroes yaw at the current heading. Call it in setup() with the
        // robot sitting in its starting orientation. Returns immediately if
        // no IMU was detected.
        void calibrateIMUs();

        // IMU sensor fusion (VQF, 6DOF - the magnetometer is never used):
        // quaternion, rest detection, automatic gyro bias calibration,
        // optional calibration hooks, and flash persistence. The library
        // feeds it sensor samples automatically; sketches call it directly,
        // e.g. NoU3.fusion.getQuaternion(w, x, y, z). See Alfredo_NoU3_VQF.h.
        NoU3_VQF fusion{104.0f};  // keep in sync with the LSM6 ODR

		float getBatteryVoltage(){ return analogReadMilliVolts(PIN_SNS_VIN) * 0.001 * 7.818; };
		float getVersionVoltage(){ return analogReadMilliVolts(PIN_SNS_VERSION) * 0.001 ; };

        void stopMotors();

        void beginServiceLight();
        void setServiceLight(serviceLightState state);
        void updateServiceLight();

        // Raw sensor data: accel in g, gyro in rad/s (bias not removed -
        // the fusion filter handles bias internally; its estimate is
        // available via NoU3.fusion.getGyroBias()).
        float acceleration_x=0, acceleration_y=0, acceleration_z=0;
        float gyroscope_x=0, gyroscope_y=0, gyroscope_z=0;
        // Raw magnetometer data, for legacy boards only: the latest NoU3
        // has no magnetometer, and it is never used in sensor fusion.
        float magnetometer_x=0, magnetometer_y=0, magnetometer_z=0;

        float roll = 0, pitch = 0, yaw = 0;

        volatile serviceLightState stateServiceLight;

    private:
        float yawReference = 0;  // published yaw = filter yaw - this (guarded by the IMU spinlock)
        uint32_t fusionLastEdgeUs = 0;  // previous data-ready edge timestamp
};

class NoU_Motor {
    public:
        NoU_Motor(uint8_t motorPort);
        void set(float output);
        void setInverted(bool isInverted);
        void setBrakeMode(bool isBrakeMode);
        void setMotorCurve(float minimumOutput, float maximumOutput, float deadband, float exponent);
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setExponent(float exponent);
        void setDeadband(float deadband);

        void beginEncoder(int8_t pinA = -1, int8_t pinB = -1);
        int32_t getPosition();
        void resetPosition();

    private:
        float applyCurve(float output) const;
        uint8_t motorPort;
        bool inverted = false;
        bool brakeMode = false;
        float minimumOutput = 0;
        float maximumOutput = 1;
        float exponent = 1;
        float deadband = 0;

        Encoder _encoder;
};

class NoU_Servo {
    public:
        NoU_Servo(uint8_t servoPort, uint16_t minPulse = 540, uint16_t maxPulse = 2300);
        void write(float degrees);
        void writeMicroseconds(uint16_t pulseLength);
        void setMinimumPulse(uint16_t minPulse);
        void setMaximumPulse(uint16_t maxPulse);
        uint16_t getMicroseconds();
        float getDegrees();
    private:
        uint8_t pin = 0;
        uint8_t channel = 0;
        uint16_t minPulse;
        uint16_t maxPulse;
        uint16_t pulse = 0;
};

class NoU_Drivetrain {
    public:
        NoU_Drivetrain(NoU_Motor* leftMotor, NoU_Motor* rightMotor);
        NoU_Drivetrain(NoU_Motor* frontLeftMotor, NoU_Motor* frontRightMotor,
                        NoU_Motor* rearLeftMotor, NoU_Motor* rearRightMotor);
        void tankDrive(float leftPower, float rightPower);
        void arcadeDrive(float throttle, float rotation, bool invertedReverse = false);
        void curvatureDrive(float throttle, float rotation, bool isQuickTurn = true);
        void holonomicDrive(float xVelocity, float yVelocity, float rotation, bool plusConfig = false);
        void setMotorCurves(float minimumOutput, float maximumOutput, float deadband, float exponent);
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setExponent(float Exponent);
        void setDeadband(float Deadband);
    private:
        void setMotors(float frontLeftPower, float frontRightPower, float rearLeftPower, float rearRightPower);
        NoU_Motor *frontLeftMotor;
        NoU_Motor *frontRightMotor;
        NoU_Motor *rearLeftMotor;
        NoU_Motor *rearRightMotor;
        DrivetrainType drivetrainType;
        float quickStopThreshold = 0.2;
        float quickStopAlpha = 0.1;
        float quickStopAccumulator = 0;
};

extern NoU_Agent NoU3;
extern LSM6Class LSM6;
extern MMC5983MAClass MMC5;

extern PCA9685 pca9685;
