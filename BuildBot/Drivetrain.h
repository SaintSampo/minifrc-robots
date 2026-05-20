#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

void beginDrivetrain();
float getDrivetrainPosition(char direction);

void updateDrivetrain(float gamepadX, float gamepadY, float gamepadRotation);
void gyroControlAngle(float targetAngle);
void encoderControlX(float targetX);
void encoderControlY(float targetY);
void visionControlXY(float targetDistance, float targetOffset);

void resetDrivetrainEncoders();

#define MEASURED_ANGLE 27.164;
#define ANGULAR_SCALE (5.0 * 2.0 * PI) / MEASURED_ANGLE;

#endif