#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>

class PIDF {
public:
    PIDF(float kp = 1.0, float ki = 0.0, float kd = 0.0, float kf = 0.0,
        float min_output = 0.0, float max_output = 1.0)
        : kp(kp), ki(ki), kd(kd), kf(kf), min_output(min_output), max_output(max_output),
          prev_error(0), integral(0), start_time(0), prev_time(0) {}

    float update(float error, float velocity = 0) {
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

        return constrain(output, this->min_output, this->max_output);
    }

    void clear_history() {
        this->prev_error = 0;
        this->prev_time = 0;
    }

private:
    float kp, ki, kd, kf;
    float min_output, max_output;

    float prev_error, integral;
    unsigned long start_time, prev_time;
};

#endif
