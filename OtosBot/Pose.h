#pragma once

#include <Arduino.h>

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
