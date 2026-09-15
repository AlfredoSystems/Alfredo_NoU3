/**
 * Example code for a robot using a NoU3 controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

// If your robot has more than a drivetrain, add those actuators here
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// If yaw reads a few degrees short or long per rotation (spin the robot 5 
// times and check), run the CalibrateGyroScale example. It measures your
// gyro's scale error and saves the fix to flash, and every sketch (including
// this one) loads it automatically at NoU3.begin().

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE
    PestoLink.begin("NoU3_Bluetooth");
    Serial.begin(115200);

    NoU3.begin();

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);

    // Set the robot down pointing "field forward" before powering on.
    NoU3.setServiceLight(LIGHT_CALIBRATING);
    NoU3.calibrateIMUs();
}

void loop() {
    static unsigned long lastPrintTime = 0;
    if (lastPrintTime + 100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw );
        lastPrintTime = millis();
    }

    // This measures your batterys voltage and sends it to PestoLink
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        float fieldPowerX = PestoLink.getAxis(0);
        float fieldPowerY = -1 * PestoLink.getAxis(1);
        float rotationPower = -1 * PestoLink.getAxis(2);

        // Get robot heading (in radians) from the gyro
        float heading = NoU3.yaw;

        // Rotate joystick vector to be robot-centric
        float cosA = cos(heading);
        float sinA = sin(heading);

        float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
        float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

        //set motor power
        drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.stopMotors();
        NoU3.setServiceLight(LIGHT_DISABLED);
    }
}
