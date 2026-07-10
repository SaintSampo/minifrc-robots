/**
 * Example code for a robot using a NoU3 controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <HCSR04.h>

#define SONAR_TRIGGER_PIN PIN_I2C_SDA_QWIIC
#define SONAR_ECHO_PIN PIN_I2C_SCL_QWIIC

// If your robot has more than a drivetrain and one servo, add those actuators here 
NoU_Motor frontLeftMotor(4);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(7);

NoU_Motor leftLauncherMotor(2);
NoU_Motor rightLauncherMotor(8);

NoU_Motor rightSpindexerMotor(5);

NoU_Motor intakeMotor(1);

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE BEFORE PAIRING THEIR ROBOT TO ANY LAPTOP
    PestoLink.begin("PothosBot");
    Serial.begin(115200);

    HCSR04.begin(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);

    Wire1.begin(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU, 400000);
    NoU3.beginMotors();
    NoU3.beginIMUs();
    NoU3.beginServiceLight();

    // If a motor in your drivetrain is spinning the wrong way, change the value for it here from 'false' to 'true'
    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(false);
    rearLeftMotor.setInverted(true);
    rearRightMotor.setInverted(false);
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

    static float intakePower = 0;
    static float spindexerPower = 0;
    static float launcherPower = 0;

    if (PestoLink.buttonHeld(0)) {
        intakePower = 1;
    }
    else {
        intakePower = 0;
    }

    if (PestoLink.buttonHeld(2)) {
        spindexerPower = 1;
    }
    else {
        spindexerPower = 0;
    }

    if (PestoLink.buttonHeld(1)) {
        launcherPower = 0;
    }

    if (PestoLink.buttonHeld(3)) {
        launcherPower = 1;
    }

    // Here we set the drivetrain motor speeds and servo angle based on what we found in the above code
    intakeMotor.set(intakePower);
    leftLauncherMotor.set(launcherPower);
    rightLauncherMotor.set(launcherPower);
    rightSpindexerMotor.set(spindexerPower);
}
