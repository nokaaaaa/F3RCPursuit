#include "PIDcontroller.hpp"

PIDController::PIDController(int frequency, float kp, float ki, float kd) : FREQUENCY(frequency), KP(kp), KI(ki/frequency), KD(kd*frequency) {
    prevError = 0.0f;
    integral = 0.0f;
}

float PIDController::calculate(float error) {
    float output = KP * error + KI * integral + KD * (error - prevError);
    prevError = error;
    integral += error;

    return output;
}

void PIDController::reset(){
    integral = 0.0;
}

void PIDController::setGain(float kp, float ki, float kd){
    KP = kp;
    KI = ki/FREQUENCY;
    KD = kd*FREQUENCY;
}