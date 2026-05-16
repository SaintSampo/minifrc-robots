#pragma once

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <math.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

static QwiicOTOS myOtos;
static bool otosConnected = false;

inline void OpticalFlow_begin() {
    if (myOtos.begin() == false) {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        return;
    }

    otosConnected = true;

    Serial.println("OTOS connected!");
    myOtos.calibrateImu();
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    myOtos.resetTracking();

    sfe_otos_pose2d_t offsetPose = {0, 0, 0};
    myOtos.setOffset(offsetPose);
}

inline void OpticalFlow_resetTracking() {
    myOtos.resetTracking();
}

inline void OpticalFlow_rotateTracking(float angle) {
    sfe_otos_pose2d_t offsetPose = {0, 0, angle};
    myOtos.setOffset(offsetPose);
}

inline float OpticalFlow_getX() {
    if (!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    return myPosition.x;
}

inline float OpticalFlow_getY() {
    if (!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    return myPosition.y;
}

inline float OpticalFlow_getTheta() {
    if (!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    static float lastWrappedTheta = 0;
    static float unwrappedTheta = 0;

    float currentWrappedTheta = myPosition.h;
    float delta = currentWrappedTheta - lastWrappedTheta;

    if (delta > M_PI) {
        delta -= 2 * M_PI;
    } else if (delta < -M_PI) {
        delta += 2 * M_PI;
    }

    unwrappedTheta += delta;
    lastWrappedTheta = currentWrappedTheta;

    return unwrappedTheta;
}

inline float OpticalFlow_getWrappedTheta() {
    if (!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    return myPosition.h;
}
