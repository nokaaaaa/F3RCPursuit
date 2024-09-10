#pragma once
#include <PinNames.h>

namespace DigitalOutPins
{
    constexpr PinName MOTOR1_DIR = PC_9; // モータ1の方向 3の位置
    constexpr PinName MOTOR2_DIR = PB_9; // モータ2の方向 4の位置
    constexpr PinName MOTOR3_DIR = PA_6; // モータ3の方向 2の位置
    constexpr PinName MOTOR4_DIR = PA_7; // モータ4の方向 1の位置

    constexpr PinName MOTOR5_DIR = PC_7;//モータ5の方向 昇降機
    constexpr PinName MOTOR6_DIR = PB_1;//モータ6の方向 回収機
}

namespace PwmOutPins
{
    constexpr PinName MOTOR1_PWM = PC_8; // モータ1のPWM
    constexpr PinName MOTOR2_PWM = PB_8; // モータ2のPWM
    constexpr PinName MOTOR3_PWM = PB_14; // モータ3のPWM
    constexpr PinName MOTOR4_PWM = PA_11;//モータ4のPWM

    constexpr PinName MOTOR5_PWM = PB_6; // モータ5のPWM
    constexpr PinName MOTOR6_PWM = PA_9;//モータ6のPWM
}

namespace DigitalInPins
{
    constexpr PinName MEASURING_ENCODER1_B = PC_0; // 測定輪エンコーダ1のB相
    constexpr PinName MEASURING_ENCODER2_B = PA_4; // 測定輪エンコーダ2のB相
}

namespace InterruptInPins
{
    constexpr PinName MEASURING_ENCODER1_A = PC_1; // 測定輪エンコーダ1のA相
    constexpr PinName MEASURING_ENCODER2_A = PB_0; // 測定輪エンコーダ2のA相
}



namespace AnalogInPins
{
   
}

namespace AnalogOutPins
{
    
}

namespace UartPins
{
    constexpr PinName BNO_RX = PC_11; // BNO055,RX
    constexpr PinName BNO_TX = PC_10; // BNO055,TX
}

namespace I2cPins
{
    
}
