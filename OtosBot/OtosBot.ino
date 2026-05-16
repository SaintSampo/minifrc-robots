#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "AutoModeAgent.h"
#include "OpticalFlowAgent.h"

NoU_Motor frontLeftMotor(5);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(6);
NoU_Motor rearRightMotor(3);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
  Serial.begin(115200);
  PestoLink.begin("BuddyBot");

  pinMode(0, INPUT_PULLUP);

  NoU3.begin();

  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(false);
  rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(false);
  drivetrain.setMotorCurves(0.3, 1, 0.05, 1.8);
  
  OpticalFlow_begin();
  NoU3.calibrateIMUs(); // takes 1000ms
}

void loop() {
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()) {
    //Serial.printf("yaw(rad):%.3f,pitch(rad):%.3f,roll(rad):%.3f\r\n", NoU3.yaw, NoU3.pitch, NoU3.roll);
    //PestoLink.printfTerminal("elevator:%.3d, pivot:%.3d\r\n",elevator.getPosition(),pivot.getPosition());
    PestoLink.printfTerminal("X(m):%.3f, Y(m):%.3f, theta(rad):%.3f\r\n", OpticalFlow_getX(), OpticalFlow_getY(),  OpticalFlow_getTheta());
    lastPrintTime = millis();
  }

  // Measures battery voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  // Here we decide what the throttle and rotation direction will be based on gamepad inputs
  if (PestoLink.isConnected()) {
    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -1 * PestoLink.getAxis(1);
    float rotationPower = -1 * PestoLink.getAxis(2);
    
    // Get robot heading (in radians) from a gyro
    float heading = OpticalFlow_getTheta();

    // Rotate joystick vector to be field-centric
    float cosA = cos(heading);
    float sinA = sin(heading);

    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;
    
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower, true);
    
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.setServiceLight(LIGHT_DISABLED);
  } 

  while (PestoLink.keyHeld(Key::Q) || !digitalRead(0)) {
    Pose origin = {0, 0, 0};
    drivetrain_set(origin);
    //PestoLink.printfTerminal("starting automode setup");
    delay(1);
  }
}