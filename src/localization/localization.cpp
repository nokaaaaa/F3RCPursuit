#include <mbed.h>
#include <math.h>
#include "parameters.hpp"
#include "localization.hpp"
#include "driveBase/driveBase.hpp"
//ここでst35をまとめてObservation型の変数を宣言
Localization::Localization(DT35* dt1,DT35* dt2, DT35* dt3, BNO055* bno055, Encoder* encoder1, Encoder* encoder2):obs(dt1, dt2, dt3)
{
    encoders[0] = encoder1;
    encoders[1] = encoder2;
    ticker.attach([this] {Update();}, std::chrono::milliseconds(1000)/ENCODER_LOCALIZATION_FREQUENCY);
    flag = false;
    kalman = true;
}
 
void Localization::Update()
{

    float Moveincrement[2]={0,0};
    float increment_x=0;
    float increment_y=0;
    float increment_theta=0;
    float avetheta=0;//前回と今回の角度の平均
    
    Moveincrement[0]=encoders[0]->getMoveDistance()-beforeMove[0];//エンコーダ1(x方向)の移動距離
    Moveincrement[1]=encoders[1]->getMoveDistance()-beforeMove[1];//エンコーダ2(y方向)の移動距離
    
    direction=bno055.getRadians();//角度の取得
    increment_theta=direction -beforeTheta;

    avetheta=(direction+beforeTheta)/2;

    posX+=Moveincrement[0]*cos(avetheta)-  Moveincrement[1]*sin(avetheta);
    posY+=Moveincrement[0]*sin(avetheta)+  Moveincrement[1]*cos(avetheta);
    //カルマンフィルタ
    
    if(posX<KALMAN_OFF)kalman=false;//2つめのトッピングを回収したらfalseにする
    if(kalman)
    {
      posX=KALMAN_X*obs.getPoseObs().x+ (1-KALMAN_X)*posX;
      posY=KALMAN_Y*obs.getPoseObs().y + (1-KALMAN_Y)*posY;
      direction=KALMAN_THETA*obs.getPoseObs().theta + (1-KALMAN_THETA)*direction;

    }
    
    increment_x=posX-beforeX;
    increment_y=posY-beforeY;
    increment_theta=direction-beforeTheta;

    speedX=increment_x*ENCODER_LOCALIZATION_FREQUENCY;
    speedY=increment_y*ENCODER_LOCALIZATION_FREQUENCY;
    rotateSpeed=increment_theta*ENCODER_LOCALIZATION_FREQUENCY;

    beforeMove[0]=encoders[0]->getMoveDistance();
    beforeMove[1]=encoders[1]->getMoveDistance();

    beforeTheta=direction;
    beforeX=posX;
    beforeY=posY;

    //x,y,thetaの速度から　各モーターの速度[mm/s] をもとめる
    for(int i=0;i<4;i++)
     {
    motorSpeed[i]=-sin(direction+PI/4+i*(PI/2))*cos(direction)*speedX
                   +cos(direction+PI/4+i*(PI/2))*cos(direction)*speedY
                   +rotateSpeed*TRED_RADIUS;//計算後で確認
     }

    
}


void Localization::setPosition(float X, float Y, float D){
    //位置を強制的に設定
    posX = X;
    posY = Y;
    direction = D;
}

void Localization::addLocalization(function<void(float*, float*, float*)> f, int tag, bool activate){
    functions[tag] = f;
    activations[tag] = activate;
}

void Localization::activateLocalization(int tag){
    activations[tag] = true;
}

void Localization::inactivateLocalization(int tag){
    activations[tag] = false;
}

void Localization::loop(){
    for(auto i = functions.begin(); i != functions.end(); ++i){
        if(activations[i->first]){
            (i->second)(&posX, &posY, &direction);
        }
    }
}
