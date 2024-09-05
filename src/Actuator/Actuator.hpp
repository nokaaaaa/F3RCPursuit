#pragma once
#include <mbed.h>

class Actuator{

    public:
        PwmOut pwmOut; //モーター
        DigitalOut dirOut;
        Actuator(PinName pwm_pin, PinName dir_pin);
        
        void act(float pwm,bool dir,float time);

    private:

};