#include "pid.h"

PID::PID(float kp, float ki, float kd, float min, float max):
_kp(kp),
_ki(ki),
_kd(kd),
_pmin(min),
_pmax(max)
{
}

PID::~PID()
{
}

float PID::_constrain(float value){
    return (_pmin < value && value < _pmax) ? value : ((value < _pmin) ? _pmin : _pmax);
}

float PID::tick(float state, unsigned long time){
    unsigned long delta = time - _lastTick;
    float dt = (float)(delta / TIME_SCALE);
    
    float p = (setpoint - state) * _kp;
    _integral += (setpoint - state) * dt;
    float i = _integral * _ki;
    float d = (state - _prevState) * _kd;
    _prevState = state;

    Serial.print("PID P:");
    Serial.println(p);

    if(_pmin != 0.0f || _pmax != 0.0f){
        if(p > _pmin && p < _pmax){
            Serial.print("PID I:");
            Serial.println(i);
            Serial.print("PID D:");
            Serial.println(d);
            return _constrain(p + i + d);
        }
        else{
            _integral -= (setpoint - state) * dt; //undo last integration if system is already saturated by P term
            Serial.print("PID I:");
            Serial.println(i);
            Serial.print("PID D:");
            Serial.println(d);
            return _constrain(p + d); 
        }
    }
    else{
        Serial.print("PID I:");
        Serial.println(i);
        Serial.print("PID D:");
        Serial.println(d);
        return p + i + d;
    }
}
