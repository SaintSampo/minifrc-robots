#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "PIDF.h"
#include "Drivetrain.h"
#include "Vision.h"

NoU_Motor pivotMotor(5);

NoU_Servo coralServo(1);
float P_Ku = 0.006;
float P_Tu = 1.0 / (150.0 / 60.0);  //Period = 1/(BPM/60), BPM is measuring full cycles per minute

float P_Kp = 0.6 * P_Ku;
float P_Kd = 0.125 * P_Ku * P_Tu;

PIDF pivotPID(0.040, 0, 0, 0, -0.4, 0.4);

typedef enum {
  STATE_GRAB_CORAL = 6,
  STATE_SCORE_L3 = 7,
  STATE_DEALGIFY = 5,
} pestoButtons;

void setup() {
  PestoLink.begin("BuildBot");
  Serial.begin(115200);

  NoU3.begin();

  pivotMotor.setInverted(true);
  pivotMotor.beginEncoder();

  delay(500);
  NoU3.setServiceLight(LIGHT_CALIBRATING);
  NoU3.calibrateIMUs();  // this takes exactly one second. Do not move the robot during calibration.

  beginDrivetrain();
  beginVision();
}

void loop() {
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()) {
    //Serial.printf("gyro yaw (radians): %.3f | PestoAxis2: %.3f\r\n",  NoU3.yaw*ANGULAR_SCALE, PestoLink.getAxis(2));
    //Serial.printf("X(mm): %.3f | Y(mm): %.3f \r\n",  getDrivetrainPosition('X'), getDrivetrainPosition('Y'));
    lastPrintTime = millis();
  }

  // This measures your battery's voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  if (bumpDetected()){
    Serial.println("bump detected!");
  }

  if (PestoLink.isConnected()) {
    if (PestoLink.buttonHeld(8)) {
      setauto();
    }
    if (PestoLink.buttonHeld(9)) {
      automode();
    }
    if (PestoLink.buttonHeld(13)) {
      NoU3.calibrateIMUs();
    }

    float gamepadX = 0.9 * PestoLink.getAxis(0);
    float gamepadY = 0.9 * PestoLink.getAxis(1);
    float gamepadRotation = 0.6 * PestoLink.getAxis(2);

    updateDrivetrain(gamepadX, gamepadY, gamepadRotation);

    //encoderControlX(gamepadX * 50);
    //encoderControlY(gamepadY * 50);
    
    //gyroControlAngle(gamepadRotation*2);

    //visionControlXY(-200.0, 0.0);

    updateActuators();
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    updateDrivetrain(0, 0, 0);  // stop motors if connection is lost
    NoU3.setServiceLight(LIGHT_DISABLED);
  }

  delay(1);
}

void setauto(){
  resetDrivetrainEncoders();
  for(int i = 0; i < 1000; i++){

    encoderControlX(0);
    encoderControlY(-60);
    delay(1);
  }
}

void automode(){
  coralServo.write(135);
  delay(500);
  resetDrivetrainEncoders();
  for(int i = 0; i < 500; i++){
    encoderControlX(0);
    encoderControlY(60);
    gyroControlAngle(0);
    set_pivot(0);
    delay(1);
  }
  for(int i = 0; i < 1500; i++){
    set_pivot(4900);
    delay(1);
  }
  coralServo.write(90);
  delay(1000);
  for(int i = 0; i < 1500; i++){
    set_pivot(0);
    delay(1);
  }
}

void updateActuators() {
  if (PestoLink.buttonHeld(STATE_GRAB_CORAL)) {
    coralServo.write(135);
  } else {
    coralServo.write(90);
  }

  if (PestoLink.buttonHeld(STATE_SCORE_L3)) {
    set_pivot(4000);
  } else if (PestoLink.buttonHeld(STATE_DEALGIFY)) {
    set_pivot(5750);
  } else if (PestoLink.buttonHeld(15)) {
    set_pivot(4900);
  } else {
    set_pivot(0);
  }

  // if (PestoLink.buttonHeld(STATE_SCORE_L3)) {
  //   set_pivot(4000);
  // } else if (PestoLink.buttonHeld(STATE_DEALGIFY)) {
  //   set_pivot(6000);
  // } else {
  //   set_pivot(0);
  // }
}

void set_pivot(float targetAngle) {
  static float startAngle = 0;  //pivotMotor.getPosition();
  float effort = 0;

  float currentAngle = pivotMotor.getPosition();
  float errorAngle = (startAngle + targetAngle) - currentAngle;
  effort = pivotPID.update(errorAngle);

  pivotMotor.set(effort);

  //Serial.printf("target(ticks): %.1f  |  current(ticks): %.3f |  error(ticks): %.3f  |  effort: %.1f \n",targetAngle,currentAngle,errorAngle,effort);
}


bool bumpDetected(){
  static unsigned long lastBumpTime = 0;
  if (lastBumpTime + 700 < millis()) {
    if (fabs(NoU3.acceleration_y) >= 0.8){
      lastBumpTime = millis();
      return true;
    }
  }

  return false;
}

