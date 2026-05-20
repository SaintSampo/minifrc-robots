#pragma once

#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "OpticalFlowAgent.h"
#include "PIDF.h"
#include "Pose.h"

NoU_Motor frontLeftMotor(5);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(6);
NoU_Motor rearRightMotor(3);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// X/Y PID — Ziegler–Nichols
float X_Ku = 100 * 0.4;
float X_Tu = 1.0 / (110.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float X_Kp = 0.4 * X_Ku;            //0.6 * X_Ku;
float X_Kd = 0.1 * X_Ku * X_Tu;     //0.125 * X_Tu;

PIDF xPID(X_Kp, 0, X_Kd);
PIDF yPID(X_Kp, 0, X_Kd);

// Theta PID — Ziegler–Nichols
float R_Ku = 3.0;
float R_Tu = 1.0 / (200.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float R_Kp = 0.4 * R_Ku;
float R_Kd = 0.1 * R_Ku * R_Tu;

PIDF rotationPID(R_Kp, 0, R_Kd);

Pose lastFieldEndPose = {0, 0, 0};

void beginDrivetrain() {
  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(false);
  rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(false);
  drivetrain.setMotorCurves(0.3, 1, 0.05, 1.8);

  frontLeftMotor.beginEncoder();
  frontRightMotor.beginEncoder();
  rearLeftMotor.beginEncoder();
  rearRightMotor.beginEncoder();
}

Pose getDrivetrainPosition() {
  int fl = -1 * frontLeftMotor.getPosition();
  int fr = -1 * frontRightMotor.getPosition();
  int rl = -1 * rearLeftMotor.getPosition();
  int rr = -1 * rearRightMotor.getPosition();

  //Serial.printf("fl: %5d | fr: %5d | rl: %5d | rr: %5d \r\n", fl, fr, rl, rr);

  float WHEEL_CIRCUMFERENCE = 45.0;  //mm
  float TICKS_PER_REV = 585.0;

  float mmPerTick = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
  float ticks = 0;

  float positionX = mmPerTick * (float)(fl - rr) / 2.0;
  float positionY = mmPerTick * (float)(fr - rl) / 2.0;

  Pose position = {positionX, positionY, 0};

  return position;
}


void directControlXY(float gamepadX, float gamepadY) {
  float fieldPowerX = gamepadX;
  float fieldPowerY = -1 * gamepadY;

  // Get robot heading (in radians) from the gyro
  float heading = OpticalFlow_getTheta();

  // Rotate joystick vector to be robot-centric
  float cosA = cos(heading);
  float sinA = sin(heading);

  float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
  float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

  //set motor power
  drivetrain.holonomicDrive(robotPowerX, robotPowerY, NAN, true);
}


void directControlTheta(float gamepadRotation) {
  float rotationPower = -1 * gamepadRotation;

  //set motor power
  drivetrain.holonomicDrive(NAN, NAN, rotationPower, true);
}


void otosControlXY(Pose fieldJournyPose) {
  Pose fieldCurrentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

  Pose fieldErrorPose = (lastFieldEndPose + fieldJournyPose) - fieldCurrentPose;
  Pose robotErrorPose = fieldErrorPose.toRobotFrame(fieldCurrentPose.theta);

  float effortX = xPID.update(robotErrorPose.x);
  float effortY = yPID.update(robotErrorPose.y);

  drivetrain.holonomicDrive(effortX, effortY, NAN, true);

  //PestoLink.printfTerminal("X: %.3f  |  Y: %.3f  |  theta: %.3f \n", fieldCurrentPose.x, fieldCurrentPose.y, fieldCurrentPose.theta);
  //PestoLink.printfTerminal("X: %.3f  |  Y: %.3f  |  theta: %.3f \n", effortX, effortY, effortTheta);
  //Serial.printf("currentX: %.3f | fieldErrorX: %.3f | robotErrorX: %.3f | effortX: %.3f \n", fieldCurrentPose.x, fieldErrorPose.x, robotErrorPose.x, effortX);
  //Serial.printf("currentY: %.3f | fieldErrorY: %.3f | robotErrorY: %.3f | effortY: %.3f \n", fieldCurrentPose.y, fieldErrorPose.y, robotErrorPose.y, effortY);
}


void otosControlAngle(float targetAngle) {
  static float startAngle = 0;
  float rotationEffort = 0;

  float currentAngle = OpticalFlow_getTheta();
  float errorAngle = (startAngle + targetAngle) - currentAngle;
  rotationEffort = rotationPID.update(errorAngle);

  drivetrain.holonomicDrive(NAN, NAN, rotationEffort, true);

  //Serial.printf("target(ticks): %.1f  |  current(ticks): %.3f |  error(ticks): %.3f  |  effort: %.1f \n",targetAngle,currentAngle,errorAngle,effort);
}


void encoderControlX(float targetX) {
  static float startX = 0;
  float effortX = 0;

  float currentX = getDrivetrainPosition().x;
  float errorX = (startX + targetX) - currentX;
  effortX = xPID.update(errorX);

  drivetrain.holonomicDrive(effortX, NAN, NAN, true);
}


void encoderControlY(float targetY) {
  static float startY = 0;
  float effortY = 0;

  float currentY = getDrivetrainPosition().y;
  float errorY = (startY + targetY) - currentY;
  effortY = yPID.update(errorY);

  drivetrain.holonomicDrive(NAN, effortY, NAN, true);
}


void resetDrivetrainEncoders() {
  frontLeftMotor.resetPosition();
  frontRightMotor.resetPosition();
  rearLeftMotor.resetPosition();
  rearRightMotor.resetPosition();
}







