#pragma once 

#include "Encoder/Encoder.hpp"
#include <map>
#include <functional>
#include "BNO055/BNO055.hpp"
#include "Observation/Observation.hpp"
#include "DT35/DT35.hpp"

using namespace std;

//自己位置推

class Localization{
    public:
        Encoder* encoders[2]; //エンコーダー
        BNO055 bno055; 
        Observation obs;
        //DT35 dt1, dt2, dt3;
        //Observation observation;をいれるとエラー このときはobservationを構成するDT35をもってくる

        Localization(DT35* dt1, DT35*dt2, DT35* dt3, BNO055* bno055, Encoder* encoder1, Encoder* encoder2);

        //推定された位置
        float posX = 0.0f; //位置X[mm]
        float posY = 0.0f; //位置Y[mm]
        float direction = 0.0f; //方角D[rad]

        //推定された速度
        float rotateSpeed = 0.0f;
        float speedX = 0.0f;
        float speedY = 0.0f;

        float motorSpeed[4]={0,0,0,0};

        void setPosition(float X, float Y, float D); //外部から位置を強制的に設定する．
        void loop();
        void addLocalization(function<void(float*, float*, float*)> f, int tag, bool activate=true);
        void activateLocalization(int tag);
        void inactivateLocalization(int tag);

        

        int incrementedNumBefore[4] = {0,0,0,0};

        int _s1;



    private:
        void encoderLocalization(); //エンコーダーによる自己位置推定
        void Update();//もろもろの自己位置推定

        float beforeX=0;
        float beforeY=0;
        float beforeTheta=0;

        float beforeMove[2]={0,0};//エンコーダーの前の位置

        bool flag=false;
        bool kalman=true;//カルマンフィルタのオンオフ

        map<int, function<void(float*, float*, float*)>> functions;
        map<int, bool> activations;

        Ticker ticker;

         std::array<DT35*, 3> dts;
};