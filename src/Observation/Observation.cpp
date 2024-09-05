#include "Observation.hpp"

//PoseObs型の構造体を返す
//参照https://kra.kibe.la/notes/5320 
PoseObs Observation::getPoseObs()
{
    //まずDT35_1とDT35_2で横の壁からの距離とヨー角を求める
    float l1 = dts1.getObsDistance();
    float l2 = dts2.getObsDistance();
    float l3 = dts3.getObsDistance();
    float coesin =((-1)*l1*sin(DT_1_degree)+l2*sin(DT_2_degree)-DT_1_y+DT_2_y);
    float coecos =((-1)*l1*cos(DT_1_degree)+l2*cos(DT_2_degree)-DT_1_x+DT_2_x);

    float cosvalue = abs(coecos/sqrt(coesin*coesin+coecos*coecos));
    float sinvalue = coecos*cosvalue/coesin;

    poseObs.theta = asin(sinvalue);

    poseObs.x=Distance_VERTICAL_WALL - DT_1_x*cosvalue +DT_1_y*sinvalue-l1*cos(poseObs.theta+DT_1_degree);

    //ヨー角からy座標を求める
    poseObs.y = -1*(DT_3_x*sinvalue+DT_3_y*cosvalue)+ l3*cos(poseObs.theta+DT_3_degree) - Distance_BESIDE_WALL;
    return poseObs;
}