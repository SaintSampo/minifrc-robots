#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "Drivetrain.h"
#include "OpticalFlowAgent.h"
#include "OtosCalibrateOffset.h"
#include "Pose.h"


void setup() {
  Serial.begin(115200);
  PestoLink.begin("OtosBot");
  NoU3.begin();

  beginDrivetrain();
  pinMode(0, INPUT_PULLUP);

  NoU3.setServiceLight(LIGHT_CALIBRATING);
  OpticalFlow_begin();
  NoU3.calibrateIMUs(); // takes 1000ms
}

void loop() {
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()) {
    //Serial.printf("yaw(rad):%.3f,pitch(rad):%.3f,roll(rad):%.3f\r\n", NoU3.yaw, NoU3.pitch, NoU3.roll);
    //PestoLink.printfTerminal("elevator:%.3d, pivot:%.3d\r\n",elevator.getPosition(),pivot.getPosition());
    //PestoLink.printfTerminal("X(m):%.3f, Y(m):%.3f, theta(rad):%.3f\r\n", OpticalFlow_getX(), OpticalFlow_getY(),  OpticalFlow_getTheta());
    lastPrintTime = millis();
  }

  // Measures battery voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  // Here we decide what the throttle and rotation direction will be based on gamepad inputs
  if (PestoLink.isConnected()) {

    if (PestoLink.keyHeld(Key::Q) || !digitalRead(0)) {
      automode();
    }

    float gamepadX = PestoLink.getAxis(0);
    float gamepadY = PestoLink.getAxis(1);
    float gamepadRotation = PestoLink.getAxis(2);

    directControlXY(gamepadX, gamepadY);
    if (PestoLink.buttonHeld(0)){
      Pose orgin = {0,0,0};
      pointToward(orgin);
    } else{
      directControlTheta(gamepadRotation);
    }
    
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.stopMotors();
    NoU3.setServiceLight(LIGHT_DISABLED);
  } 
  delay(1);
}

void pointToward(Pose pointPose){
  float currentX = OpticalFlow_getX();
  float currentY = OpticalFlow_getY();

  float targetAngle = atan2(pointPose.y - currentY, pointPose.x - currentX);
  PestoLink.printfTerminal("target theta(rad): %.3f\r\n", targetAngle);
  otosControlAngle(targetAngle);
}

void automode(){

}

void automode2(){
  // //PestoLink.printfTerminal("starting automode setup");
  // Pose origin = {0, 0, 0};
  // Pose spin_ten = {0, 0, 10*2*PI};
  // //drivetrain_set(origin);
  // //test_otos();
  // //calibrate_otos_offset();
  // unsigned long start = millis();
  // while (millis() - start < 15000) {
  //   drivetrain_set(spin_ten);
  //   delay(1);
  // }
}

void automode3(){
  // coralServo.write(135);
  // delay(500);
  // resetDrivetrainEncoders();
  // for(int i = 0; i < 500; i++){
  //   encoderControlX(0);
  //   encoderControlY(60);
  //   gyroControlAngle(0);
  //   set_pivot(0);
  //   delay(1);
  // }
  // for(int i = 0; i < 1500; i++){
  //   set_pivot(4900);
  //   delay(1);
  // }
  // coralServo.write(90);
  // delay(1000);
  // for(int i = 0; i < 1500; i++){
  //   set_pivot(0);
  //   delay(1);
  // }
}