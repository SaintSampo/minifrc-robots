#pragma once

#include <VL53L0X.h>

// VL53L0X time-of-flight sensor on the qwiic port (NoU3.begin() sets up Wire on those pins)
inline VL53L0X distanceSensor;

inline void beginDistanceSensor() {
  distanceSensor.setTimeout(100);
  distanceSensor.init();
  // Lower the return signal rate limit (default is 0.25) to catch weak, distant reflections
  //distanceSensor.setSignalRateLimit(0.1);
  // Increase laser pulse periods (defaults are 14 and 10 PCLKs)
  //distanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  //distanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  // Reduce timing budget to 20 ms (default is ~33 ms).
  //distanceSensor.setMeasurementTimingBudget(20000);
  // Tells the sensor to continuously stream readings back-to-back at hardware speed
  distanceSensor.startContinuous();
}

inline float readDistanceCm() {
  return distanceSensor.readRangeContinuousMillimeters() / 10.0;
}
