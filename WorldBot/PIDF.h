#pragma once

#include <Arduino.h>

class PIDF {
public:
    PIDF(float kp = 1.0, float ki = 0.0, float kd = 0.0, float kf = 0.0, float min_output = -1, float max_output = 1)
        : kp(kp), ki(ki), kd(kd), kf(kf), min_output(min_output), max_output(max_output), max_integral(1000) {}

    void setTerms(float kp, float ki, float kd, float kf) {
        this->kp = kp; this->ki = ki; this->kd = kd; this->kf = kf;
    }

    void setLimits(float min_output, float max_output) {
        this->min_output = min_output; this->max_output = max_output;
    }

    void setMaxIntegral(float max_integral) {
        this->max_integral = max_integral;
    }

    void setDebug(const char* label) {
        this->debugLabel = label;
        this->isDebug = true;
    }

    float update(float error, float velocity = 0) {
        unsigned long current_time = micros();
        float timestep;

        if (this->prev_time == (unsigned long)-1) {
            this->prev_time = current_time;
            return 0;
        } else {
            timestep = (current_time - this->prev_time) / 1000000.0;
        }

        this->prev_time = current_time;

        float derivative = 0;
        if (timestep != 0) derivative = (error - this->prev_error) / timestep;

        this->integral += error * timestep;
        this->integral = constrain(this->integral, -1 * this->max_integral, this->max_integral);

        float output = this->kp * error + this->ki * this->integral + this->kd * derivative + this->kf * velocity;

        if (this->isDebug) {
            float _p = this->kp * error, _i = this->ki * this->integral;
            float _d = this->kd * derivative, _f = this->kf * velocity;
            Serial.printf("[%s] err=%.*f  P=%.*f  I=%.*f  D=%.*f  F=%.*f\n",
                this->debugLabel,
                (error < 0 ? 2 : 3), error,
                (_p    < 0 ? 2 : 3), _p,
                (_i    < 0 ? 2 : 3), _i,
                (_d    < 0 ? 2 : 3), _d,
                (_f    < 0 ? 2 : 3), _f);
        }

        this->prev_error = error;
        
        return constrain(output, this->min_output, this->max_output);
    }

    void clear_history() {
        this->prev_error = 0;
        this->integral = 0;
        this->prev_time = (unsigned long)-1;
    }

private:
    float kp, ki, kd, kf;
    float min_output, max_output;
    float max_integral;

    bool isDebug = false;
    const char* debugLabel = nullptr;

    float prev_error = 0;
    float integral = 0;

    unsigned long prev_time = -1;
};
