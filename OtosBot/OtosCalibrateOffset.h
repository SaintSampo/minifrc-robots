#pragma once

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "OpticalFlowAgent.h"
#include "PIDF.h"

extern NoU_Drivetrain drivetrain;

// Rotates the robot one full turn and uses the 180° and 360° position readings
// to solve for the sensor's physical offset from the robot's center of rotation.
//
// Math: rotating π radians with the sensor at (dx, dy) from center displaces the
// sensor by (-2·dx, -2·dy) in the field frame. At 360° the offset contribution
// cancels entirely, leaving only mechanical drift. Subtracting half the total drift
// from the 180° reading isolates the offset displacement, giving dx and dy.
//
// X/Y position is intentionally NOT corrected during the spin — doing so would
// fight the offset displacement we need to measure.
inline void calibrate_otos_offset() {

    const float R_Ku = 3.0f;
    const float R_Tu = 60.0f / 200.0f;
    PIDF rotPID(0.4f * R_Ku, 0.0f, 0.1f * R_Ku * R_Tu);

    const float full_target = TWO_PI;

    float x_at_half = 0, y_at_half = 0;
    bool half_recorded = false;

    unsigned long start = millis();
    float start_theta = OpticalFlow_getTheta();
    delay(300);
    while (millis() - start < 12000) {
        float theta = OpticalFlow_getTheta();
        float absTheta = fabsf(theta);

        if (!half_recorded && absTheta >= M_PI) {
            x_at_half = OpticalFlow_getX();
            y_at_half = OpticalFlow_getY();
            half_recorded = true;
        }
        if (absTheta >= TWO_PI - 0.05f) break;

        drivetrain.holonomicDrive(0, 0, rotPID.update(start_theta + full_target - theta), true);
        delay(1);
    }

    PestoLink.printfTerminal("stopping");
    drivetrain.holonomicDrive(0, 0, 0, true);
    delay(1000);

    if (!half_recorded) {
        PestoLink.printfTerminal("Calibration failed: robot did not complete half rotation\r\n");
        return;
    }

    float x_end = OpticalFlow_getX();
    float y_end = OpticalFlow_getY();

    float x_corrected = x_at_half - x_end / 2.0f;
    float y_corrected = y_at_half - y_end / 2.0f;
    float offset_x = -x_corrected / 2.0f;
    float offset_y = -y_corrected / 2.0f;

        delay(200);
    PestoLink.printfTerminal("=== OTOS Offset Calibration ===\r\n");
        delay(200);
    PestoLink.printfTerminal("  offset X: %.4f m\r\n", offset_x);
        delay(200);
    PestoLink.printfTerminal("  offset Y: %.4f m\r\n", offset_y);
        delay(200);
    PestoLink.printfTerminal("  (drift X: %.4f m, drift Y: %.4f m)\r\n", x_end, y_end);
        delay(200);
    PestoLink.printfTerminal("================================\r\n");
}
