#pragma once
#include <mbed.h>
#include "parameters.hpp"
#include "DT35/DT35.hpp"
typedef struct {
    float x; // 位置(x [mm])
    float y; // 位置(y [mm])
    float theta; // 姿勢(rad)
} PoseObs;

class Observation {
    public:
       Observation(DT35* dt1, DT35* dt2, DT35* dt3):dts1(*dt1),dts2(*dt2),dts3(*dt3)
       {
             poseObs={0,0,0};//初期化
       }
       DT35 dts1;
       DT35 dts2;
       DT35 dts3;     
        
        PoseObs poseObs;

        PoseObs getPoseObs();


        
    private:

};