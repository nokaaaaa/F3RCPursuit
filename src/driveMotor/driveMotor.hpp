#pragma once
#include <mbed.h>
#include "PIDcontroller/PIDcontroller.hpp"
#include <functional>
//#include "Encoder/Encoder.hpp"
using namespace std;

class DriveMotor{
    public:
        //Encoder encoder;
        PwmOut pwmOut; //モーター
        DigitalOut dirOut;
        PIDController pidController;
        PIDController pidSpeedController;
       
        void rotateTo(float target, bool idle=true);
    
        void setPWM(float signed_pwm); //PWMの直接書き込み

        void rotate(float targetSpeed,float speed); //速度を指定

        void rotatePermanent(float speed, bool idle=true);

        void stop();

        void attachLoop(function<void(void)> loop_func);

        bool sign;

        DriveMotor(PinName pwm_pin, PinName dir_pin, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2, bool sign=1);

        float target = 0.0f;
        float target_speed = 0.0f;
        bool moving = false;


        float lastEncoderAmount = 0.0f;

        float pwm = 0.0f;

        float _s1 = 0;
        float _s2 = 0;

    private:
        void rotateTowardTargetAccDcc(); //target[mm] に向かって動く
        Ticker movementTicker;
        function<void(void)> loop;
};
