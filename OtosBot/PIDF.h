#pragma once

#include <Arduino.h>

class PIDF {
public:
    PIDF(float kp = 1.0, float ki = 0.0, float kd = 0.0, float kf = 0.0,
        float min_output = -1.0, float max_output = 1.0
        );

    float update(float error, float velocity = 0);
    void clear_history();

private:

    float kp, ki, kd, kf;
    float min_output, max_output;

    float prev_error, integral;
    unsigned long start_time, prev_time;
};

inline PIDF::PIDF(float kp, float ki, float kd, float kf,
         float min_output, float max_output) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;
    this->min_output = min_output;
    this->max_output = max_output;

    this->prev_error = 0;
    this->integral = 0;

    this->start_time = 0;
    this->prev_time = 0;
}

inline float PIDF::update(float error, float velocity) {
    unsigned long current_time = millis();
    float timestep;

    if (this->prev_time == 0) {
        this->start_time = current_time;
        timestep = 0.001;
    } else {
        timestep = (current_time - this->prev_time) / 1000.0;
    }

    this->prev_time = current_time;

    float derivative = 0;
    if (timestep != 0) derivative = (error - this->prev_error) / timestep;

    this->integral += error * timestep;

    float max_integral = 1000;
    if (this->integral > max_integral) this->integral = max_integral;
    if (this->integral < -max_integral) this->integral = -max_integral;

    float output = this->kp * error + this->ki * this->integral + this->kd * derivative + this->kf * velocity;
    this->prev_error = error;

    output = constrain(output, this->min_output, this->max_output);

    return output;
}

inline void PIDF::clear_history() {
    this->prev_error = 0;
    this->prev_time = 0;
}
