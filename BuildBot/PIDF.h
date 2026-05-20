#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>

class PIDF {
public:
    PIDF(float kp = 1.0, float ki = 0.0, float kd = 0.0, float kf = 0.0, float min_output = -1, float max_output = 1);
    
    void setTerms(float kp, float ki, float kd, float kf);
    void setLimits(float min_output, float max_output);
    void setMaxIntegral(float max_integral);

    float update(float error, float velocity = 0);
    void clear_history();

private:

    float kp, ki, kd, kf;
    float min_output, max_output;
    float max_integral;

    float prev_error = 0;
    float integral = 0;

    unsigned long prev_time = -1;
};

#endif