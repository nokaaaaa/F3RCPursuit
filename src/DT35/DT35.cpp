#include "DT35.hpp"
DT35::DT35(PinName analog_in_pin):analog_in(analog_in_pin)
{
  //初期設定
}



//電流の値から距離を取得
float DT35::getObsDistance()
{
    float distance = 0;
    float voltage = analog_in.read() ;
    //(x,y)=(DT_MIN_ANALOG,DT_MIN_DISTANCE),(DT_MAX_ANALOG,DT_MAX_DISTANCE) の2点から距離と電流の一次関数を求める
    distance = (DT_MAX_DISTANCE-DT_MIN_DISTANCE)/(DT_MAX_ANALOG-DT_MIN_ANALOG)*(voltage-DT_MIN_ANALOG)+DT_MIN_DISTANCE;
    return distance;
}