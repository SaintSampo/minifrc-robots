#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <VL53L0X.h>
#include "PIDF.h"

NoU_Motor frontLeftMotor(4);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(7);
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

NoU_Motor intakeMotor(1);
NoU_Motor spindexerMotors(5);
NoU_Motor launcherLeftMotor(2);
NoU_Motor launcherRightMotor(8);

float Ku = 8.28;
float Tu = 1.0 / (195.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float Kp = 0.8 * Ku;            //0.8 * X_Ku;
float Kd = 0.125 * Ku * Tu;     //0.125 * X_Tu;
PIDF scanPID(Kp, 0.0, Kd);

// VL53L0X time-of-flight sensor on the qwiic port (NoU3.begin() sets up Wire on those pins)
VL53L0X distanceSensor;
float readDistanceCm() {
  return distanceSensor.readRangeContinuousMillimeters() / 10.0;
}

// Like Arduino's map(), but for floats
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Rotate the robot in place exactly 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process.
float measured_angle = 28.38 * -1.0;  //have to multiply by -1 because the PothosBot NoU3 is upside down
float angular_scale = (5.0 * 2.0 * PI) / measured_angle;

typedef enum {
  INTAKE = 4,
  LAUNCHERS = 1,
  SPINDEXERS = 3,
  SCAN_LEFT = 6,
  SCAN_RIGHT = 7,
  TEST_PIDF = 12,
  START_AUTO = 13
} pestoButtons;

void setup() {
  PestoLink.begin("PothosBot");
  Serial.begin(115200);

  NoU3.begin();
  //give the driver 2 seconds to set teh robot down before starting calibration
  delay(2000);
  // calibration takes exactly one second. Do not move the robot during calibration.
  NoU3.setServiceLight(LIGHT_CALIBRATING);
  NoU3.calibrateIMUs();

  distanceSensor.setTimeout(500);
  distanceSensor.init();
  distanceSensor.startContinuous();

  frontLeftMotor.setInverted(true);
  //frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  //rearRightMotor.setInverted(true);
  //intakeMotors.setInverted(true);
  //launcherLeftMotor.setInverted(true);
  launcherRightMotor.setInverted(true);
  //intakeMotor.setInverted(true);
  drivetrain.setMotorCurves(0.25, 1, 0.05, 1.5);
}

void loop() {
  float heading = NoU3.yaw * angular_scale;

  //float scanKp = mapFloat(PestoLink.getAxis(3), -1.0, 1.0, 0.0, 3.6 * 4.0);
  // scanPID.set_gains(scanKp, 0.0, 0.0, 0.0);

  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()) {
    PestoLink.printfTerminal("distance (cm): %.3f, gyro yaw (radians): %.3f, Kp: %.2f\r\n", readDistanceCm(), heading, scanPID.kp);
    PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
    lastPrintTime = millis();
  }

  if (PestoLink.isConnected()) {
    intakeMotor.set(PestoLink.buttonHeld(INTAKE) ? 1 : 0);

    // While the scan-and-fire routine is active it owns the drivetrain,
    // launcher, and spindexer, so teleop only commands them when it is idle
    if (!scanAndFire()) {
      launcherLeftMotor.set(PestoLink.buttonHeld(LAUNCHERS) ? 1 : 0);
      launcherRightMotor.set(PestoLink.buttonHeld(LAUNCHERS) ? 1 : 0);
      spindexerMotors.set(PestoLink.buttonHeld(SPINDEXERS) ? 1 : 0);

      float throttle = -PestoLink.getAxis(1);
      float rotation = PestoLink.getAxis(2);

      drivetrain.arcadeDrive(throttle, rotation);
    }

    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.stopMotors();
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}

//---------------------------------------- Auto Aim ----------------------------------//

enum ScanState { SCAN_IDLE, SCAN_SCANNING, SCAN_AIMING, SCAN_TESTING };
ScanState scanState = SCAN_IDLE;
int buttonScan;
int buttonFire;
float scanEffort;
float smallestReading;
float targetAngle;

// Turns the robot toward targetAngle with the scan PID.
// Returns the current error in radians.
float aimAtTarget() {
  float currentAngle = NoU3.yaw * angular_scale;
  float errorAngle = targetAngle - currentAngle;
  float effort = -1 * scanPID.update(errorAngle);

  drivetrain.arcadeDrive(0, effort);

  return errorAngle;
}

// Runs one step of the scan-and-fire routine, call it once per loop.
// Hold a scan button to sweep, then hold the opposite button to aim and fire.
// Returns true while the routine is controlling the robot.
bool scanAndFire() {
  switch (scanState) {

    case SCAN_IDLE:
      if (PestoLink.buttonHeld(TEST_PIDF)) {
        // Control loop test: hold the test button to aim at a target 90 degrees
        // from the current heading, a step input for judging the PID response.
        // Only the drivetrain moves, the launcher and spindexer stay off.
        targetAngle = NoU3.yaw * angular_scale + PI / 2.0;
        scanPID.clear_history();
        scanState = SCAN_TESTING;
      } else if (PestoLink.buttonHeld(SCAN_LEFT) || PestoLink.buttonHeld(SCAN_RIGHT)) {
        bool scanningLeft = PestoLink.buttonHeld(SCAN_LEFT);
        buttonScan = scanningLeft ? SCAN_LEFT : SCAN_RIGHT;
        buttonFire = scanningLeft ? SCAN_RIGHT : SCAN_LEFT;
        scanEffort = scanningLeft ? -0.47 : 0.47;
        smallestReading = 1000000000.0;
        targetAngle = NoU3.yaw * angular_scale;
        scanState = SCAN_SCANNING;
      }
      break;

    case SCAN_SCANNING:
      if (PestoLink.buttonHeld(buttonScan)) {
        drivetrain.arcadeDrive(0, scanEffort);

        float reading = readDistanceCm();
        if (reading < smallestReading) {
          smallestReading = reading;
          targetAngle = NoU3.yaw * angular_scale;
        }
      } else if (PestoLink.buttonHeld(buttonFire)) {
        scanPID.clear_history();
        scanState = SCAN_AIMING;
      } else {
        // scan button released without the fire button held, cancel
        scanState = SCAN_IDLE;
      }
      break;

    case SCAN_AIMING:
      if (PestoLink.buttonHeld(buttonFire)) {
        float errorAngle = aimAtTarget();
        launcherLeftMotor.set(1);
        launcherRightMotor.set(1);

        float angleThreshold = 2.0 * (PI / 180.0);  // 4 degrees
        if (abs(errorAngle) < angleThreshold) {
          spindexerMotors.set(1);
        } else {
          spindexerMotors.set(0);
        }
      } else {
        scanState = SCAN_IDLE;
      }
      break;

    case SCAN_TESTING:
      if (PestoLink.buttonHeld(TEST_PIDF)) {
        aimAtTarget();
        launcherLeftMotor.set(0);
        launcherRightMotor.set(0);
        spindexerMotors.set(0);
      } else {
        scanState = SCAN_IDLE;
      }
      break;
  }

  return scanState != SCAN_IDLE;
}
