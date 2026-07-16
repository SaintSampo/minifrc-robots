#pragma once
#include <Arduino.h>

class PIDF {
public:
    PIDF(float kp = 1.0, float ki = 0.0, float kd = 0.0, float kf = 0.0,
        float min_output = -1.0, float max_output = 1.0,
        float max_integral = 1.0
        );

    float update(float error, float velocity = 0);
    void clear_history();

    float kp, ki, kd, kf;

    float min_output, max_output, max_integral;

    float prev_error, integral;
    unsigned long prev_time;
};

inline PIDF::PIDF(float kp, float ki, float kd, float kf,
         float min_output, float max_output, float max_integral) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kf = kf;
    this->min_output = min_output;
    this->max_output = max_output;
    this->max_integral = max_integral;

    this->prev_error = 0;
    this->integral = 0;
    this->prev_time = 0;
}

inline float PIDF::update(float error, float velocity) {
    unsigned long current_time = millis();
    bool first_call = (this->prev_time == 0);

    float timestep = first_call ? 0.001 : (current_time - this->prev_time) / 1000.0;
    this->prev_time = current_time;

    float derivative = 0;
    if (!first_call && timestep != 0) derivative = (error - this->prev_error) / timestep;

    // ki is folded into the accumulation so changing it at runtime doesn't rescale
    // past history, and max_integral clamps in output units
    this->integral += this->ki * error * timestep;
    this->integral = constrain(this->integral, -this->max_integral, this->max_integral);

    float output = this->kp * error + this->integral + this->kd * derivative + this->kf * velocity;
    this->prev_error = error;

    return constrain(output, this->min_output, this->max_output);
}

inline void PIDF::clear_history() {
    this->prev_error = 0;
    this->integral = 0;
    this->prev_time = 0;
}
