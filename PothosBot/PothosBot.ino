#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "DistanceSensor.h"
#include "PIDF.h"

NoU_Motor frontLeftMotor(6);
NoU_Motor frontRightMotor(3);
NoU_Motor rearLeftMotor(4);
NoU_Motor rearRightMotor(5);
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

NoU_Motor intakeMotorLeft(2);
NoU_Motor intakeMotorRight(8);
NoU_Motor spindexerMotors(7);
NoU_Motor launcherMotors(1);

float Ku = 8.28;
float Tu = 1.0 / (195.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float Kp = 0.8 * Ku;            //0.8 * X_Ku;
float Kd = 0.125 * Ku * Tu;     //0.125 * X_Tu;
PIDF scanPID(Kp, 0.0, Kd);

// Like Arduino's map(), but for floats
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Rotate the robot in place exactly 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process.
float measured_angle = 28.38 * -1.0;  //have to multiply by -1 because the PothosBot NoU3 is upside down
float angular_scale = (5.0 * 2.0 * PI) / measured_angle;

typedef enum {
  LAUNCHERS_REVERSE = 0,
  LAUNCHERS = 1,
  SPINDEXERS_REVERSE = 2,
  SPINDEXERS = 3,
  INTAKE = 4,
  INTAKE_REVERSE = 5,
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

  beginDistanceSensor();

  //frontLeftMotor.setInverted(true);
  //frontRightMotor.setInverted(true);
  //rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(true);
  intakeMotorLeft.setInverted(true);
  intakeMotorRight.setInverted(true);
  spindexerMotors.setInverted(true);
  //launcherMotors.setInverted(true);
  drivetrain.setMotorCurves(0.2, 1, 0.05, 1.2);
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
    // Press START_AUTO to run the autonomous routine; it blocks until done
    if (PestoLink.buttonHeld(START_AUTO)) {
      runAuto();
    } else {
      float intake_power = PestoLink.buttonHeld(INTAKE) - PestoLink.buttonHeld(INTAKE_REVERSE);
      intakeMotorLeft.set(intake_power);
      intakeMotorRight.set(intake_power);

      // While the scan-and-fire routine is active it owns the drivetrain,
      // launcher, and spindexer, so teleop only commands them when it is idle
      if (!scanAndFire()) {
        float launcher_power = (0.65 * PestoLink.buttonHeld(LAUNCHERS)) - PestoLink.buttonHeld(LAUNCHERS_REVERSE);
        float spindexer_power = (0.9 * PestoLink.buttonHeld(SPINDEXERS)) - PestoLink.buttonHeld(SPINDEXERS_REVERSE);
        launcherMotors.set(launcher_power);
        spindexerMotors.set(spindexer_power);

        float throttle = -PestoLink.getAxis(1);
        float rotation = PestoLink.getAxis(2);

        drivetrain.arcadeDrive(throttle, rotation);
      }
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
float targetAngle;

// Readings closer than this count as the target; farther ones are ignored
const float TARGET_DISTANCE_CM = 200.0;
// Ignore the shot unless at least this many on-target readings were collected
const int MIN_TARGET_COUNT = 3;
float angleSum;
float distanceSum;
int targetCount;
float launcherPower;

// Measured (distance cm, launcher power) pairs, sorted by distance.
// PLACEHOLDER VALUES - replace with your collected data.
struct LaunchPoint { float distanceCm; float power; };
LaunchPoint launchTable[] = {
  { 10.0, 0.6 },
  { 40.0, 1.0 },
};
const int launchTableSize = sizeof(launchTable) / sizeof(launchTable[0]);

// Linearly interpolates launcher power for a distance, clamped past the ends.
float launcherPowerForDistance(float distanceCm) {
  if (distanceCm <= launchTable[0].distanceCm) return launchTable[0].power;
  if (distanceCm >= launchTable[launchTableSize - 1].distanceCm) return launchTable[launchTableSize - 1].power;

  for (int i = 1; i < launchTableSize; i++) {
    if (distanceCm < launchTable[i].distanceCm) {
      return mapFloat(distanceCm,
                      launchTable[i - 1].distanceCm, launchTable[i].distanceCm,
                      launchTable[i - 1].power, launchTable[i].power);
    }
  }
  return launchTable[launchTableSize - 1].power;  // unreachable
}

// Turns the robot toward targetAngle with the scan PID.
// Returns the current error in radians.
float aimAtTarget() {
  float currentAngle = NoU3.yaw * angular_scale;
  float errorAngle = targetAngle - currentAngle;
  float effort = -1 * scanPID.update(errorAngle);

  drivetrain.arcadeDrive(0, effort);

  return errorAngle;
}

// Begins a sweep: clears the collected readings and sets the sweep rotation.
void startScan(float effort) {
  scanEffort = effort;
  angleSum = 0;
  distanceSum = 0;
  targetCount = 0;
  targetAngle = NoU3.yaw * angular_scale;  // fallback if the target is never seen
}

// One sweep iteration: rotate and collect any on-target reading.
void scanStep() {
  drivetrain.arcadeDrive(0, scanEffort);

  float reading = readDistanceCm();
  Serial.printf("reading: %.2f, angle: %.3f \n", reading, NoU3.yaw * angular_scale);
  if (reading < TARGET_DISTANCE_CM) {
    angleSum += NoU3.yaw * angular_scale;
    distanceSum += reading;
    targetCount++;
  }
}

// Ends a sweep: aims at the average angle of the on-target readings (the
// center) and picks the launcher power for their average distance.
// Returns false if not enough of the target was seen.
bool lockTarget() {
  if (targetCount < MIN_TARGET_COUNT) return false;

  targetAngle = angleSum / targetCount;
  launcherPower = launcherPowerForDistance(distanceSum / targetCount);
  scanPID.clear_history();
  return true;
}

// One aiming iteration: turn toward the target, spin the launcher, and run
// the spindexer while lined up.
void aimAndFireStep() {
  float errorAngle = aimAtTarget();
  launcherMotors.set(launcherPower);

  float angleThreshold = 2.0 * (PI / 180.0);
  spindexerMotors.set(fabs(errorAngle) < angleThreshold ? 1 : 0);
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
        startScan(scanningLeft ? -0.5 : 0.5);
        scanState = SCAN_SCANNING;
      }
      break;

    case SCAN_SCANNING:
      if (PestoLink.buttonHeld(buttonScan)) {
        scanStep();
      } else if (PestoLink.buttonHeld(buttonFire) && lockTarget()) {
        scanState = SCAN_AIMING;
      } else {
        // scan cancelled, or not enough of the target was seen
        scanState = SCAN_IDLE;
      }
      break;

    case SCAN_AIMING:
      if (PestoLink.buttonHeld(buttonFire)) {
        aimAndFireStep();
      } else {
        scanState = SCAN_IDLE;
      }
      break;

    case SCAN_TESTING:
      if (PestoLink.buttonHeld(TEST_PIDF)) {
        aimAtTarget();
        launcherMotors.set(0);
        spindexerMotors.set(0);
      } else {
        scanState = SCAN_IDLE;
      }
      break;
  }

  return scanState != SCAN_IDLE;
}

//---------------------------------------- Auto Mode ----------------------------------//

// arcadeDrive rotation that turns the robot LEFT, and the sign of a left turn in
// gyro heading units. If auto turns the wrong way, negate BOTH of these.
const float AUTO_LEFT_ROTATION = -0.5;
const float AUTO_LEFT_SIGN = 1.0;
const float AUTO_LAUNCH_POWER = 1.0;
const float AUTO_TURN_TOLERANCE = 3.0 * (PI / 180.0);

// Blocks until the robot has turned to target (or 3 s passes), using the scan PID.
void turnToAngle(float target) {
  targetAngle = target;
  scanPID.clear_history();
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (fabs(aimAtTarget()) < AUTO_TURN_TOLERANCE) break;
    delay(5);
  }
  drivetrain.arcadeDrive(0, 0);
}

// The full autonomous routine. Blocks from START_AUTO until it finishes.
void runAuto() {
  float startHeading = NoU3.yaw * angular_scale;
  intakeMotorLeft.set(1);
  intakeMotorRight.set(1);
  drivetrain.arcadeDrive(0.5, 0);
  delay(900);
  drivetrain.arcadeDrive(0.375, 0);
  delay(1500);
  drivetrain.arcadeDrive(-0.5, 0);
  delay(750);
  drivetrain.arcadeDrive(0, 0);
  delay(1100);

  turnToAngle(startHeading + AUTO_LEFT_SIGN * (PI / 2.0));  // turn left 90 degrees
  delay(1000);

  // Sweep left 135 degrees collecting readings, same as a teleop scan
  launcherMotors.set(AUTO_LAUNCH_POWER);  // spin up while sweeping
  float scanStart = NoU3.yaw * angular_scale;
  startScan(AUTO_LEFT_ROTATION);
  while (AUTO_LEFT_SIGN * ((NoU3.yaw * angular_scale) - scanStart) < 135.0 * (PI / 180.0)) {
    scanStep();
    delay(5);
  }

  // Aim at the target the sweep found and fire
  if (lockTarget()) {
    unsigned long fireStart = millis();
    while (millis() - fireStart < 6000) {
      aimAndFireStep();
      delay(5);
    }
  }
  launcherMotors.set(0);
  spindexerMotors.set(0);
  drivetrain.arcadeDrive(0, 0);

  drivetrain.arcadeDrive(0.8, 0);   // forward 0.3 s
  delay(300);

  turnToAngle(startHeading + AUTO_LEFT_SIGN * PI);  // rotate to 180 degrees (absolute)

  drivetrain.arcadeDrive(0.8, 0);   // forward 2 s
  delay(2000);
  drivetrain.arcadeDrive(0, 0);
}
