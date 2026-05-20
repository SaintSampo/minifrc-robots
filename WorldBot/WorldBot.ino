#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

#include "drivetrain.h"
#include "uwbAgent.h"

// SPI pins for the UWB module
UWBAgent uwb(/*sck*/7, /*miso*/6, /*mosi*/5, /*ss*/4, /*rst*/8, /*irq*/9);

void setup() {
  PestoLink.begin("WorldBot");
  Serial.begin(115200);

  NoU3.begin();

  NoU3.setServiceLight(LIGHT_CALIBRATING);
  NoU3.calibrateIMUs();

  beginDrivetrain();

  static const float ANCHOR_X[4] = { 0.0f, 1.829f, 1.829f, 0.0f };
  static const float ANCHOR_Y[4] = { 0.0f, 0.0f,  2.743f, 2.743f };
  for (int i = 0; i < 4; i++) uwb.setAnchorPosition(i, ANCHOR_X[i], ANCHOR_Y[i]);

  if (!uwb.begin()) {
    Serial.println("UWB init failed");
  }
}

void loop() {
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()){

    float angular_scale = (10.0 * 2.0 * PI) / 56.331;
    PestoLink.printfTerminal("DW X: %.3f  Y: %.3f, R: %.3f\r\n", uwb.getX(), uwb.getY(), NoU3.yaw * angular_scale);
    //PestoLink.printfTerminal("Encoder X: %.3f  Y: %.3f\r\n", getDrivetrainPosition('X'),getDrivetrainPosition('Y'));
    lastPrintTime = millis();

    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);
  }

  Serial.printf("%.3f, %.3f\r\n", uwb.getX(), uwb.getY());

  if (PestoLink.isConnected()) {
    if (PestoLink.buttonHeld(13)) {
      NoU3.calibrateIMUs();
    }

    if (PestoLink.buttonHeld(0)) {
      gyroControlAngle(0);
    } else if (PestoLink.buttonHeld(2)) {
      encoderControlX(0);
    } else if (PestoLink.buttonHeld(1)) {
      encoderControlY(0);
    } else if (PestoLink.buttonHeld(3)) {
      uwbControlXY(uwb, 0.9145, 0.9145/2);
      gyroControlAngle(0);
    } else {
      float gamepadX        =       PestoLink.getAxis(0);
      float gamepadY        =       PestoLink.getAxis(1);
      float gamepadRotation = 0.6 * PestoLink.getAxis(2);
      updateDrivetrain(gamepadX, gamepadY, gamepadRotation);
    }

    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    updateDrivetrain(0, 0, 0);
    NoU3.setServiceLight(LIGHT_DISABLED);
  }

  delay(1);
}
