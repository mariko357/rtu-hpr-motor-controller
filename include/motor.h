#include <Arduino.h>
#include "pid.h"

#define PWM_FREQ 4000
#define PWM_CHANNEL 0
#define MOTOR_ENCODER_COUNTS_PER_TURN 100.0f
#define MOTOR_ENCODER_RADS_PER_PULSE 2 * PI / MOTOR_ENCODER_COUNTS_PER_TURN
#define TIME_DIVIDER 1000000.0f
#define PIN_A 0
#define PIN_B 1

#define MOTOR_PID_RANGE 255.0f
#define MOTOR_PID_P 50.0f
#define MOTOR_PID_I 0.5f
#define MOTOR_PID_D 5.0f

#define MOTOR_A 9
#define MOTOR_B 10
#define MOTOR_BOOST_ENABLE 11
#define ENCODER_A 8
#define ENCODER_B 18


//for the V3 rwc maximum achievable range is around +-500rps, but the actual practical range is around +-250rps
void motor0InterruptHandler();

class Motor
{
private:
    unsigned char _motorA;
    unsigned char _motorB;
    unsigned char _motorBoostEnable;
    unsigned char _encA;
    unsigned char _encB;
    void _configurePWM(unsigned int frequency);
    void _setInterrupts();
public:
    Motor(bool directionInverse = false, unsigned char motorA = MOTOR_A, unsigned char motorB = MOTOR_B, unsigned char motorBoostEnable = MOTOR_BOOST_ENABLE,
                unsigned char encA = ENCODER_A, unsigned char encB = ENCODER_B);
    ~Motor();
    void (*encoderInterruptHandler)();
    void init();    
    void setPower(unsigned char dutyCycle, bool direction);
    void enable();
    void disable();
    void brake();
    void tick(unsigned long time);
    void setInterruptHandler(void (*handler)());
    unsigned char getEncPins(unsigned char pin);
    PID pid;
    float speed;
    bool _dirInverse;
    unsigned long _lastEncPulse;
};

extern Motor motor0;