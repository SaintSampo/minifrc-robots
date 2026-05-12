#pragma once

#include <Alfredo_NoU3.h>
#include "PIDF.h"
#include "uwbAgent.h"

#define MEASURED_ANGLE 56.331;
#define ANGULAR_SCALE (10.0 * 2.0 * PI) / MEASURED_ANGLE;

NoU_Motor frontLeftMotor(6);
NoU_Motor frontRightMotor(5);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float X_Ku = 1.016;
float X_Tu = 1.0 / (300.0 / 60.0);
float X_Kp = 0.4 * X_Ku;
float X_Kd = 0.1 * X_Ku * X_Tu;
PIDF xPID(X_Kp, 0, X_Kd, 0);

float Y_Ku = 1.2; 
float Y_Tu = 1.0 / (300.0 / 60.0);
float Y_Kp = 0.4 * Y_Ku;
float Y_Kd = 0.1 * Y_Ku * Y_Tu;
PIDF yPID(Y_Kp, 0, Y_Kd, 0.1, 0.6);

float R_Ku = 3.5;
float R_Tu = 1.0 / (170.0 / 60.0);
float R_Kp = 0.4 * R_Ku;
float R_Kd = 0.1 * R_Ku * R_Tu;
PIDF rotationPID(R_Kp, 0, R_Kd);

void beginDrivetrain() {
  drivetrain.setMotorCurves(0.3, 0.6, 0.15, 1.8);

  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(false);

  frontLeftMotor.beginEncoder();
  frontRightMotor.beginEncoder();
  rearLeftMotor.beginEncoder();
  rearRightMotor.beginEncoder();

  //xPID.setDebug("xPID");
  //yPID.setDebug("yPID");
  //rotationPID.setDebug("rotationPID");
}

float getDrivetrainPosition(char direction) {
  int fl = 1 * frontLeftMotor.getPosition();
  int fr = -1 * frontRightMotor.getPosition();
  int rl = -1 * rearLeftMotor.getPosition();
  int rr = 1 * rearRightMotor.getPosition();

  const float WHEEL_CIRCUMFERENCE = 44.0;  //mm
  const float TICKS_PER_REV = 68.0 * 12.0;
  const float STRAFE_MULTIPLIER = 1; // Theoretically, this is 1/cos(45°) ≈ 1.414.

  float mmPerTick = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
  float ticks = 0;

  if (direction == 'X') ticks = (float)(fl - fr - rl + rr) / 4.0 / STRAFE_MULTIPLIER;
  if (direction == 'Y') ticks = (float)(fl + fr + rl + rr) / 4.0;

  return ticks * mmPerTick;
}

void updateDrivetrain(float gamepadX, float gamepadY, float gamepadRotation) {
  float fieldPowerX = gamepadX;
  float fieldPowerY = -1 * gamepadY;
  float rotationPower = -1 * gamepadRotation;

  float heading = NoU3.yaw * ANGULAR_SCALE;

  float cosA = cos(heading);
  float sinA = sin(heading);

  float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
  float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

  drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
}

void gyroControlAngle(float targetAngle) {
  static float startAngle = 0;

  float currentAngle = NoU3.yaw * ANGULAR_SCALE;
  float errorAngle = (startAngle + targetAngle) - currentAngle;
  float rotationEffort = rotationPID.update(errorAngle);

  drivetrain.holonomicDrive(NAN, NAN, rotationEffort);
}

void encoderControlX(float targetX) {
  static float startX = 0;

  float currentX = getDrivetrainPosition('X');
  float errorX = (startX + targetX) - currentX;
  float effortX = xPID.update(errorX);

  drivetrain.holonomicDrive(effortX, NAN, NAN);
}

void encoderControlY(float targetY) {
  static float startY = 0;

  float currentY = getDrivetrainPosition('Y');
  float errorY = (startY + targetY) - currentY;
  float effortY = yPID.update(errorY);

  drivetrain.holonomicDrive(NAN, effortY, NAN);
}

void resetDrivetrainEncoders() {
  frontLeftMotor.resetPosition();
  frontRightMotor.resetPosition();
  rearLeftMotor.resetPosition();
  rearRightMotor.resetPosition();
}

// targetX, targetY in meters — matches the UWB anchor coordinate system
void uwbControlXY(UWBAgent& uwb, float targetX, float targetY) {
  // Complementary filter state (world space, mm). Encoders provide high-frequency
  // dead-reckoning; UWB corrects accumulated drift with absolute position.
  static constexpr float ALPHA = 0.0f; // UWB blend weight; (1-ALPHA) trusts encoder prediction
  static float fusedWorldX = NAN;
  static float fusedWorldY = NAN;
  static float lastEncX = 0;
  static float lastEncY = 0;
  static bool initialized = false;

  float currentEncX = getDrivetrainPosition('X');
  float currentEncY = getDrivetrainPosition('Y');
  float heading = NoU3.yaw * ANGULAR_SCALE;
  float cosA = cos(heading);
  float sinA = sin(heading);

  // Seed the fused estimate from UWB on first valid reading
  if (!initialized) {
    if (!uwb.isPositionValid()) return;
    fusedWorldX = uwb.getX() * 1000.0f;
    fusedWorldY = uwb.getY() * 1000.0f;
    lastEncX = currentEncX;
    lastEncY = currentEncY;
    initialized = true;
    return;
  }

  // Encoder delta in robot space → rotate to world space
  float dEncX = currentEncX - lastEncX;
  float dEncY = currentEncY - lastEncY;
  lastEncX = currentEncX;
  lastEncY = currentEncY;

  // Robot-to-world rotation (inverse of updateDrivetrain's world-to-robot)
  float dWorldX = dEncX * cosA - dEncY * sinA;
  float dWorldY = dEncX * sinA + dEncY * cosA;

  // Complementary filter: blend encoder dead-reckoning with UWB absolute fix
  if (uwb.isPositionValid()) {
    float uwbX = uwb.getX() * 1000.0f;
    float uwbY = uwb.getY() * 1000.0f;
    fusedWorldX = (1.0f - ALPHA) * (fusedWorldX + dWorldX) + ALPHA * uwbX;
    fusedWorldY = (1.0f - ALPHA) * (fusedWorldY + dWorldY) + ALPHA * uwbY;
  } else {
    fusedWorldX += dWorldX;
    fusedWorldY += dWorldY;
  }

  // World-space error, then rotate to robot space for the existing PIDs
  float errWorldX = targetX * 1000.0f - fusedWorldX;
  float errWorldY = targetY * 1000.0f - fusedWorldY;

  // World-to-robot rotation (same as updateDrivetrain)
  float errRobotX =  errWorldX * cosA + errWorldY * sinA;
  float errRobotY = -errWorldX * sinA + errWorldY * cosA;

  float effortX = xPID.update(errRobotX);
  float effortY = yPID.update(errRobotY);

  drivetrain.holonomicDrive(effortX, effortY, NAN);
}
