#include "PIDF.h"

PIDF::PIDF(float kp, float ki, float kd, float kf, float min_output, float max_output) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;

    this->min_output = min_output;
    this->max_output = max_output;

    this->max_integral = 1000;
}

void PIDF::setTerms(float kp, float ki, float kd, float kf){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;
}

void PIDF::setLimits(float min_output, float max_output){
    this->min_output = min_output;
    this->max_output = max_output;
}

void PIDF::setMaxIntegral(float max_integral){
    this->max_integral = max_integral;
}

float PIDF::update(float error, float velocity) {
    unsigned long current_time = micros();
    float timestep; //seconds

    if (this->prev_time == -1) {
        this->prev_time = current_time;
        return 0;
    } else {
        timestep = (current_time - this->prev_time) / 1000000.0; // Convert us to seconds
    }

    this->prev_time = current_time;

    // Derivative
    float derivative = 0;
    if (timestep != 0) derivative = (error - this->prev_error) / timestep;

    // Integral
    this->integral += error * timestep;
    this->integral = constrain(this->integral, -1 * this->max_integral, this->max_integral);

    // Calculate output
    float output = this->kp * error + this->ki * this->integral + this->kd * derivative + this->kf * velocity;
    this->prev_error = error;

    // Bound output by min and max values
    output = constrain(output, this->min_output, this->max_output);

    //Serial.printf("kd %.3f  |  dt(S) %.5f  |  error %.3f  |  derivative %.3f  \n",this->kd,timestep,error,derivative);
    //Serial.printf("error: %1.3f  |  err*Kp %1.3f  |  derivative*kd %1.3f  | output %1.3f  \n",error, this->kp*error, this->kd*derivative, output);

    return output;
}

void PIDF::clear_history() {
    this->prev_error = 0;
    this->prev_time = 0;
}