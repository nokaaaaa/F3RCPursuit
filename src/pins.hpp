#pragma once
#include <PinNames.h>

namespace DigitalOutPins
{
    constexpr PinName MOTOR1_DIR = PA_4; // モータ1の方向
    constexpr PinName MOTOR2_DIR = PC_14; // モータ2の方向
    constexpr PinName MOTOR3_DIR = PA_7; // モータ3の方向
    constexpr PinName MOTOR4_DIR = PA_8; // モータ3の方向

    constexpr PinName MOTOR5_DIR = PA_8;//モータ4の方向
    constexpr PinName MOTOR6_DIR = PA_8;//モータ5の方向
}

namespace PwmOutPins
{
    constexpr PinName MOTOR1_PWM = PA_11; // モータ1のPWM
    constexpr PinName MOTOR2_PWM = PA_9; // モータ2のPWM
    constexpr PinName MOTOR3_PWM = PA_10; // モータ3のPWM
    constexpr PinName MOTOR4_PWM = PA_12;//モータ4のPWM

    constexpr PinName MOTOR5_PWM = PA_10; // モータ5のPWM
    constexpr PinName MOTOR6_PWM = PA_12;//モータ6のPWM
}

namespace DigitalInPins
{
    constexpr PinName MEASURING_ENCODER1_B = PC_0; // 測定輪エンコーダ1のB相
    constexpr PinName MEASURING_ENCODER2_B = PC_1; // 測定輪エンコーダ2のB相
    constexpr PinName MEASURING_ENCODER3_B = PB_2; // 測定輪エンコーダ3のB相
    constexpr PinName MEASURING_ENCODER4_B = PB_3; // 測定輪エンコーダ4のB相
}

namespace InterruptInPins
{
    constexpr PinName MEASURING_ENCODER1_A = PB_1; // 測定輪エンコーダ1のA相
    constexpr PinName MEASURING_ENCODER2_A = PA_12; // 測定輪エンコーダ2のA相
    constexpr PinName MEASURING_ENCODER3_A = PB_1; // 測定輪エンコーダ3のA相
    constexpr PinName MEASURING_ENCODER4_A = PB_12; // 測定輪エンコーダ4のA相
}



namespace AnalogInPins
{
    constexpr PinName DT35_1 = PB_2; // DT35_1
    constexpr PinName DT35_2 = PB_3; // DT35_2
    constexpr PinName DT35_3 = PB_4; // DT35_3
}

namespace AnalogOutPins
{
    
}

namespace UartPins
{
    constexpr PinName BNO_RX = PB_5; // BNO055,RX
    constexpr PinName BNO_TX = PB_6; // BNO055,TX
}

namespace I2cPins
{
    
}
