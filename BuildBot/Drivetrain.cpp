#include <Alfredo_NoU3.h>
#include "PIDF.h"
#include "Drivetrain.h"
#include "Vision.h"

// If your robot has more than a drivetrain, add those actuators here
NoU_Motor frontLeftMotor(2);
NoU_Motor frontRightMotor(7);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(6);

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float X_Ku = 3.25;
float X_Tu = 1.0 / (390.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float X_Kp = 0.4 * X_Ku;            //0.6 * X_Ku;
float X_Kd = 0.1 * X_Ku * X_Tu;     //0.125 * X_Tu;

PIDF xPID(X_Kp, 0, X_Kd);
PIDF yPID(X_Kp, 0, X_Kd);

float V_Ku = 0.001 * 66.0 * 0.5;
float V_Tu = 1.0 / (40.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float V_Kp = 0.5 * V_Ku;            //0.6 * X_Ku;
float V_Kd = 0.1 * V_Ku * V_Tu;     //0.125 * X_Tu;

PIDF vxPID(V_Kp, 0, V_Kd, 0, -0.8, 0.8);
PIDF vyPID(V_Kp, 0, V_Kd, 0, -0.8, 0.8);

float R_Ku = 7.0;
float R_Tu = 1.0 / (175.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute

float R_Kp = 0.4 * R_Ku;
float R_Kd = 0.1 * R_Ku * R_Tu;

PIDF rotationPID(R_Kp, 0, R_Kd);

void beginDrivetrain() {
  drivetrain.setMotorCurves(0.3, 1, 0.15, 1.8);

  frontLeftMotor.setInverted(true);
  rearRightMotor.setInverted(false);
  rearLeftMotor.setInverted(true);

  frontLeftMotor.beginEncoder();
  frontRightMotor.beginEncoder();
  rearLeftMotor.beginEncoder();
  rearRightMotor.beginEncoder();
}

float getDrivetrainPosition(char direction) {
  int fl = -1 * frontLeftMotor.getPosition();
  int fr = -1 * frontRightMotor.getPosition();
  int rl = -1 * rearLeftMotor.getPosition();
  int rr = -1 * rearRightMotor.getPosition();

  //Serial.printf("fl: %5d | fr: %5d | rl: %5d | rr: %5d \r\n", fl, fr, rl, rr);

  float WHEEL_CIRCUMFERENCE = 57.0;  //mm
  float TICKS_PER_REV = 585.0;

  float mmPerTick = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
  float ticks = 0;

  if (direction == 'X') ticks = (float)(fl - fr - rl + rr) / 4.0;
  if (direction == 'Y') ticks = (float)(fl + fr + rl + rr) / 4.0;

  float pos = ticks * mmPerTick;

  return pos;
}

void updateDrivetrain(float gamepadX, float gamepadY, float gamepadRotation) {
  float fieldPowerX = gamepadX;
  float fieldPowerY = -1 * gamepadY;
  float rotationPower = -1 * gamepadRotation;

  // Get robot heading (in radians) from the gyro
  float heading = NoU3.yaw * ANGULAR_SCALE;

  // Rotate joystick vector to be robot-centric
  float cosA = cos(heading);
  float sinA = sin(heading);

  float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
  float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

  //set motor power
  drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
}

void gyroControlAngle(float targetAngle) {
  static float startAngle = 0;
  float rotationEffort = 0;

  float currentAngle = NoU3.yaw * ANGULAR_SCALE;
  float errorAngle = (startAngle + targetAngle) - currentAngle;
  rotationEffort = rotationPID.update(errorAngle);

  drivetrain.holonomicDrive(NAN, NAN, rotationEffort);

  //Serial.printf("target(ticks): %.1f  |  current(ticks): %.3f |  error(ticks): %.3f  |  effort: %.1f \n",targetAngle,currentAngle,errorAngle,effort);
}

void encoderControlX(float targetX) {
  static float startX = 0;
  float effortX = 0;

  float currentX = getDrivetrainPosition('X');
  float errorX = (startX + targetX) - currentX;
  effortX = xPID.update(errorX);

  drivetrain.holonomicDrive(effortX, NAN, NAN);
}

void encoderControlY(float targetY) {
  static float startY = 0;
  float effortY = 0;

  float currentY = getDrivetrainPosition('Y');
  float errorY = (startY + targetY) - currentY;
  effortY = yPID.update(errorY);

  drivetrain.holonomicDrive(NAN, effortY, NAN);
}

void resetDrivetrainEncoders() {
  frontLeftMotor.resetPosition();
  frontRightMotor.resetPosition();
  rearLeftMotor.resetPosition();
  rearRightMotor.resetPosition();
}


void visionControlXY(float targetDistance, float targetOffset) {
  // Get current encoder positions (The "Fast" Truth)
  float currentEncoderX = getDrivetrainPosition('X');
  float currentEncoderY = getDrivetrainPosition('Y');

  static float setpointX_abs = currentEncoderX;
  static float setpointY_abs = currentEncoderY;
  static float visionErrorX = 0;
  static float visionErrorY = 0;
  static float currentVisX = 0;
  static float currentVisY = 0;

  // 2. Update Vision (The "Slow" Correction)
  updateVision();
  calculatePositionVision((float)get_xCenter(), (float)get_width());

  if (get_isTagVisable()) {
    // Current absolute position according to vision (inverted per your logic)
    currentVisX = -1 * get_xOffsetVision();
    currentVisY = -1 * get_distanceVision();

    // Calculate how far off we are from the tag target
    visionErrorX = targetOffset - currentVisX;
    visionErrorY = targetDistance - currentVisY;

    // --- DEADBAND ---
    // Ignore tiny camera jitters (e.g., < 5mm)
    if (abs(visionErrorX) < 5.0) visionErrorX = 0;
    if (abs(visionErrorY) < 5.0) visionErrorY = 0;

    // --- NEW TARGET CALCULATION ---
    // Target = Where we are now (Encoders) + Where we need to go (Vision Error)
    float newTargetX = currentEncoderX + visionErrorX;
    float newTargetY = currentEncoderY + visionErrorY;

    // --- LOW PASS FILTER ---
    // Blend the new target with the old one to smooth out camera lag
    // Alpha 0.2 means: 20% new vision data, 80% old smooth path
    float alpha = 0.005; 
    setpointX_abs = (setpointX_abs * (1.0 - alpha)) + (newTargetX * alpha);
    setpointY_abs = (setpointY_abs * (1.0 - alpha)) + (newTargetY * alpha);
  }

  // 3. CALCULATE EFFORTS (Using the global PIDs)
  // We calculate the error against our smooth 'setpointX_abs', not the jittery camera
  float errorX = setpointX_abs - currentEncoderX;
  float errorY = setpointY_abs - currentEncoderY;

  float effortX = xPID.update(errorX); // Ensure you use the correct PID instance here (vxPID or xPID)
  float effortY = yPID.update(errorY);

  // 4. DRIVE (Single Call)
  // Combine efforts so X and Y happen simultaneously
  drivetrain.holonomicDrive(effortX, effortY, NAN);
  
  // Debug output
  //Serial.println(String() + "currentEncoderX:" + currentEncoderX + " currentVisX:" + currentVisX + " visionErrorX:" + visionErrorX + " setpointX_abs:" + setpointX_abs + " errorX:" + errorX + " effortX:" + effortX);
  //Serial.println(String() + "TargX:" + setpointX_abs + " CurrX:" + currentEncoderX + " EffX:" + effortX);
}
