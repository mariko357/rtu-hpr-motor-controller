#include "motor.h"

Motor::Motor(bool directionInverse, unsigned char motorA, unsigned char motorB, unsigned char motorBoostEnable,
                unsigned char encA, unsigned char encB) : 
                                                                                _motorA(motorA),
                                                                                _motorB(motorB),
                                                                                _motorBoostEnable(motorBoostEnable),
                                                                                _encA(encA),
                                                                                _encB(encB),
                                                                                _dirInverse(directionInverse),
                                                                                _lastEncPulse(0),
                                                                                pid(MOTOR_PID_P, MOTOR_PID_I, MOTOR_PID_D, -MOTOR_PID_RANGE, MOTOR_PID_RANGE)
{ 
}

Motor::~Motor()
{
}

void Motor::init(){
    pinMode(_motorBoostEnable, OUTPUT);
    pinMode(_encA, INPUT);
    pinMode(_encB, INPUT);
    _configurePWM(PWM_FREQ);
    _setInterrupts();
    disable();
}

void Motor::enable(){
    pinMode(_motorBoostEnable, INPUT); // Boost pin needs to be set floating and only pullled down by the resistor, i haven't managed to do it right way
}

void Motor::disable(){
    pinMode(_motorBoostEnable, OUTPUT);
    digitalWrite(_motorBoostEnable, HIGH);
}

void Motor::_configurePWM(unsigned int frequency){
    pinMode(_motorA, OUTPUT);
    pinMode(_motorB, OUTPUT);
    analogWrite(_motorA, 0);
    analogWrite(_motorB, 0);
}

void Motor::_setInterrupts(){
    attachInterrupt(_encA, encoderInterruptHandler, RISING);
}

void Motor::setPower(unsigned char dutyCycle, bool direction){
    bool correctedDirection = (!_dirInverse) ? !direction : direction;

    if(correctedDirection){
        analogWrite(_motorA, dutyCycle);
        analogWrite(_motorB, 0);
    }
    else{
        analogWrite(_motorA, 0);
        analogWrite(_motorB, dutyCycle);
    }
}

void Motor::brake(){
    analogWrite(_motorA, 255);
    analogWrite(_motorB, 255);
}

unsigned char Motor::getEncPins(unsigned char pin = PIN_A){
    return (!pin) ? _encA : _encB;
}

void Motor::setInterruptHandler(void (*handler)()){
    encoderInterruptHandler = handler;
}

void Motor::tick(unsigned long time){
    float pidValue = pid.tick(speed, time);
    if(pidValue <= 0){
      motor0.setPower((unsigned char)-pidValue, true);
    }
    else if(pidValue > 0){
      motor0.setPower((unsigned char)pidValue, false);
    }
}

void motor0InterruptHandler(){
    
    bool direction;
    if(digitalRead(motor0.getEncPins(PIN_B))){
        direction = false;
    }
    else{
        direction = true;
    }

    unsigned long now = micros();
    double dt = (now - motor0._lastEncPulse);
    motor0._lastEncPulse = now;
    double fdt = dt / TIME_DIVIDER;

    direction = (motor0._dirInverse) ? !direction : direction;
    motor0.speed = (direction) ? -(MOTOR_ENCODER_RADS_PER_PULSE / fdt) : (MOTOR_ENCODER_RADS_PER_PULSE / fdt);
}

Motor motor0(false);