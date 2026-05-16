#pragma once

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "OpticalFlowAgent.h"
#include "PIDF.h"

extern NoU_Drivetrain drivetrain;

struct Pose {
  float x, y, theta;

  Pose operator+(const Pose& other) const {
    return {x + other.x, y + other.y, theta + other.theta};
  }

  Pose operator-(const Pose& other) const {
    return {x - other.x, y - other.y, theta - other.theta};
  }

  Pose operator*(float scale) const {
    return {x * scale, y * scale, theta};
  }

  friend Pose operator*(float scale, const Pose& pose) {
    return pose * scale;
  }

  Pose toRobotFrame(float currentTheta) const {
    float cosTheta = cos(currentTheta);
    float sinTheta = sin(currentTheta);
    return {cosTheta * x + sinTheta * y, -sinTheta * x + cosTheta * y, theta};
  }
};

// X/Y PID — Ziegler–Nichols
float X_Ku = 100 * 0.4;
float X_Tu = 1.0 / (110.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float X_Kp = 0.4 * X_Ku;            //0.6 * X_Ku;
float X_Kd = 0.1 * X_Ku * X_Tu;     //0.125 * X_Tu;

PIDF xPID(X_Kp, 0, X_Kd);
PIDF yPID(X_Kp, 0, X_Kd);

// Theta PID — Ziegler–Nichols
float R_Ku = 7.0;
float R_Tu = 1.0 / (175.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float R_Kp = 0.4 * R_Ku;
float R_Kd = 0.1 * R_Ku * R_Tu;

PIDF thetaPID(0, 0, 0);

Pose lastFieldEndPose = {0, 0, 0};

inline void drivetrain_set(Pose fieldJournyPose) {
  Pose fieldCurrentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

  Pose fieldErrorPose = (lastFieldEndPose + fieldJournyPose) - fieldCurrentPose;
  Pose robotErrorPose = fieldErrorPose.toRobotFrame(fieldCurrentPose.theta);

  float effortX = xPID.update(robotErrorPose.x);
  float effortY = yPID.update(robotErrorPose.y);
  float effortTheta = thetaPID.update(robotErrorPose.theta);

  drivetrain.holonomicDrive(effortX, effortY, effortTheta);

  //PestoLink.printfTerminal("X: %.3f  |  Y: %.3f  |  theta: %.3f \n", fieldCurrentPose.x, fieldCurrentPose.y, fieldCurrentPose.theta);
  PestoLink.printfTerminal("X: %.3f  |  Y: %.3f  |  theta: %.3f \n", effortX, effortY, effortTheta);
}

inline void test_otos() {
  const Pose legs[] = {
    { 0.5,  0.0, 0},
    { 0.0,  0.5, 0},
    {-0.5,  0.0, 0},
    { 0.0, -0.5, 0}
  };

  for (const Pose& leg : legs) {
    unsigned long start = millis();
    while (millis() - start < 3000) {
      drivetrain_set(leg);
      delay(1);
    }
    lastFieldEndPose = lastFieldEndPose + leg;
  }

  drivetrain.holonomicDrive(0, 0, 0);
}