#include <mbed.h>
#include <math.h>
#include "driveMotor.hpp"
#include "parameters.hpp"

//#include <iostream>
//using namespace std;

//初期化
DriveMotor::DriveMotor(PinName pwm_pin, PinName dir_pin, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2, bool sign): pwmOut(pwm_pin), dirOut(dir_pin), pidController(SPEED_ADJUSTMENT_FREQUENCY,kp_1,ki_1,kd_1), pidSpeedController(SPEED_ADJUSTMENT_FREQUENCY,kp_2,ki_2,kd_2), sign(sign) 
{
    pwmOut.period_ms(PWM_PERIOD);
    pidController.reset();
    
    loop = [this] {return;};
}

//PWMの書き込みここでモーターを動かしている
void DriveMotor::setPWM(float signed_pwm){
    if(signed_pwm > 0){
        pwmOut.write(signed_pwm);
        dirOut.write(!sign);
    }else{
        pwmOut.write(-signed_pwm);
        dirOut.write(sign);
    }
}


void DriveMotor::rotate(float targetSpeed, float speed){//speedはモーターの速度

    _s1 = speed;


    //速度を制限する

    if(targetSpeed > MAX_SPEED){
        targetSpeed = MAX_SPEED;
        pidController.reset();
    }else if(targetSpeed < -MAX_SPEED){
        targetSpeed = -MAX_SPEED;
        pidController.reset();
    }

    //加速度を制限する

    float targetAcc = (targetSpeed - speed) * SPEED_ADJUSTMENT_FREQUENCY;

    if(targetAcc > MAX_ACCELERATION){
        targetSpeed = speed + MAX_ACCELERATION / SPEED_ADJUSTMENT_FREQUENCY;
    }else if(targetAcc < -MAX_ACCELERATION){
        targetSpeed = speed - MAX_ACCELERATION / SPEED_ADJUSTMENT_FREQUENCY;
    }

    _s2 = targetSpeed;


    float u = pidSpeedController.calculate(targetSpeed - speed);

    pwm = pwm + u;

    setPWM(pwm);
}


//停止
void DriveMotor::stop(){
    movementTicker.detach();
    moving=false;
    setPWM(0);
}

/*
//現在決められている目標に向かって進む
//PID制御で目標値に向かうが，速度・加速度に制限を設けることで台形制御を実現する
void DriveMotor::rotateTowardTargetAccDcc(){
    //cout << "hoge" << endl;
    float distance = target - encoder.getAmount();

    float targetSpeed = pidController.calculate(distance);

    rotate(targetSpeed);

    if (abs(distance) < MOVEMENT_THRESHOLD){
        moving = false;
        stop();
    }
}

//目標まで移動
void DriveMotor::rotateTo(float target_point, bool idle){
    if(moving){
        //printf("warning: a motion requested while the motor is moving.");
        movementTicker.detach();
    }
    moving = true;

    target = target_point;

    //PID制御器の初期化
    pidController.reset();
    pidSpeedController.reset();

    //割り込みの設定
    movementTicker.attach([this] {rotateTowardTargetAccDcc();}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);

    //idle=trueなら移動が終わるまで待機
    if(idle){
        while(moving) {
            loop();
            wait_ns(1);
        }
    }
}


void DriveMotor::rotatePermanent(float speed, bool idle){
    if(moving){
        //printf("warning: a motion requested while the motor is moving.");
        movementTicker.detach();
    }
    moving = true;
    
    target_speed = speed;

    //PID制御器の初期化
    pidSpeedController.reset();

    //割り込みの設定
    movementTicker.attach([this] {rotate(target_speed);}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);

    //idle=trueなら移動が終わるまで待機
    if(idle){
        while(moving) {
            loop();
            wait_ns(1);
        }
    }
}

void DriveMotor::attachLoop(function<void(void)> loop_func){
    loop = loop_func;
}

*/