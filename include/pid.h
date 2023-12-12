#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define TIME_SCALE 1000

#define __IN_RANGE(x, a, b) (b < x && x < a) ? x : ((x < min) ? min : max)

class PID
{
private:
    float _kp;
    float _ki;
    float _kd;
    float _prevState;
    float _integral;
    float _pmax;
    float _pmin;
    unsigned long _lastTick;
    float _constrain(float value);

public:
    float setpoint;
    PID(float kp, float ki, float kd, float min = 0.0f, float max = 0.0f);
    ~PID();

    float tick(float state, unsigned long time); // state - current measurments, delt - current time in ms
};
 
#endif