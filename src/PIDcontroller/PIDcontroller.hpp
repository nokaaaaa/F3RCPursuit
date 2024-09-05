#pragma once

class PIDController {
private:
    

public:
    float integral; // 積分値
    float prevError; // 前回のエラー


    int FREQUENCY; //制御頻度
    float KP; // P制御のゲイン
    float KI; // I制御のゲイン
    float KD; // D制御のゲイン

    PIDController(int frequency, float kp, float ki, float kd);
    float calculate(float error);
    void reset();

    void setGain(float kp, float ki, float kd);
};
