/**
 * Example code for a robot using a NoU3 controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

// If your robot has more than a drivetrain and one servo, add those actuators here 
NoU_Motor frontLeftMotor(4);
NoU_Motor frontRightMotor(5);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(6);

NoU_Motor leftLauncherMotor(2);
NoU_Motor rightLauncherMotor(7);
NoU_Motor intakeMotor(8);

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE BEFORE PAIRING THEIR ROBOT TO ANY LAPTOP
    PestoLink.begin("Rekitbot");
    Serial.begin(115200);
    
    NoU3.begin();

    // If a motor in your drivetrain is spinning the wrong way, change the value for it here from 'false' to 'true'
    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(false);
    rearLeftMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    leftLauncherMotor.setInverted(true);
    rightLauncherMotor.setInverted(false);
    intakeMotor.setInverted(true);
}

void loop() {

    // This measures your batteries voltage and sends it to PestoLink
    // You could use this value for a lot of cool things, for example make LEDs flash when your batteries are low?
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
    if (PestoLink.isConnected()) {
        float throttle = -PestoLink.getAxis(1);
        float rotation = PestoLink.getAxis(0);
        
        drivetrain.arcadeDrive(throttle, rotation);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    // Here we decide what the servo angle will be based on if button 0 is pressed
    float launcherPower = 0;
    float intakePower = 0;

    if (PestoLink.buttonHeld(0)) {
        launcherPower = 1;
    }
    else {
        launcherPower = 0;
    }

    if (PestoLink.buttonHeld(1)) {
        intakePower = 1;
    }
    else {
        intakePower = 0;
    }

    // Here we set the drivetrain motor speeds and servo angle based on what we found in the above code
    leftLauncherMotor.set(launcherPower);
    rightLauncherMotor.set(launcherPower);
    intakeMotor.set(intakePower);
}
