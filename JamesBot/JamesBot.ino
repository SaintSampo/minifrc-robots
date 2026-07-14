 /**
 * Example code for a robot using a NoU3 controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

// If your robot has more than a drivetrain and one servo, add those actuators here 
NoU_Motor frontLeftMotor(5);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(7);
NoU_Motor rearRightMotor(8);

NoU_Servo agitator(1);
NoU_Motor launcher(1);

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE BEFORE PAIRING THEIR ROBOT TO ANY LAPTOP
    PestoLink.begin("JamesBot");
    Serial.begin(115200);
    
    NoU3.begin();

    // If a motor in your drivetrain is spinning the wrong way, change the value for it here from 'false' to 'true'
    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    rearLeftMotor.setInverted(true);
    rearRightMotor.setInverted(true);
}

void loop() {

    // This measures your batterys voltage and sends it to PestoLink
    // You could use this value for a lot of cool things, for example make LEDs flash when your batteries are low?
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
    if (PestoLink.isConnected()) {
        float throttle = -PestoLink.getAxis(1);
        float rotation = PestoLink.getAxis(0);
        Serial.println("connected");
        drivetrain.arcadeDrive(throttle, rotation);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.stopMotors();
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    // if (PestoLink.buttonPressed(0)) {
    //     agitatorAngle -= 5 ;
    //     if(agitatorAngle < 0) agitatorAngle = 0;
    // }
    // if (PestoLink.buttonPressed(1)) {
    //     agitatorAngle += 5 ;
    //     if(agitatorAngle > 180) agitatorAngle = 180;
    // }
    //agitator.write(agitatorAngle);

    if (PestoLink.buttonPressed(1)) {
        agitator.write(92);
    }

    if (PestoLink.buttonPressed(0)) {
        agitator.write(92-9);
    }

    if (PestoLink.buttonPressed(2)) {
        launcher.set(0);
    }

    if (PestoLink.buttonPressed(3)) {
        launcher.set(-0.8);
    }
}
