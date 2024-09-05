#include "Actuator.hpp"

Actuator::Actuator(PinName pwm_pin, PinName dir_pin):pwmOut(pwm_pin),dirOut(dir_pin)
{
    //初期設定
    pwmOut.period_us(1000);
    pwmOut.write(0);
    dirOut.write(0);
}

//time秒間動かす
void Actuator::act(float pwm,bool dir,float time)
{
    pwmOut.write(pwm);
    dirOut.write(dir);
    wait_us(1000000*time);//time秒待つ
 
}
