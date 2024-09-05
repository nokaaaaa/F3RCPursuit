#include "simpleFunctions.hpp"
#include <math.h>

float radiansMod(float x){
    //x (mod y) を -y/2 ~ y/2の範囲で出力
    //主に2つの方角のなす角度を計算するのに使用
    return fmod((fmod(x,2*PI) + PI),2*PI) - PI;
}
