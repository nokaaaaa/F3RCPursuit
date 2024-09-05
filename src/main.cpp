#include <mbed.h>
#include <cmath>
#include <math.h>
#include "Observation/Observation.hpp"
#include "pins.hpp"
#include "DT35/DT35.hpp"
#include "BNO055/BNO055.hpp"
#include "parameters.hpp"
#include "Encoder/Encoder.hpp"
#include "driveBase/driveBase.hpp"
#include "localization/localization.hpp"
#include "Actuator/Actuator.hpp"

// モータの設定
DriveMotor motor0(PwmOutPins::MOTOR1_PWM , DigitalOutPins::MOTOR1_DIR, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_2, MOTOR_0_KI_2, MOTOR_0_KD_2, 0);
DriveMotor motor1(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR,  MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_1_KP_2, MOTOR_1_KI_2, MOTOR_1_KD_2);
DriveMotor motor2(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_2_KP_2, MOTOR_2_KI_2, MOTOR_2_KD_2);
DriveMotor motor3(PwmOutPins::MOTOR4_PWM, DigitalOutPins::MOTOR4_DIR, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_3_KP_2, MOTOR_3_KI_2, MOTOR_3_KD_2);

//アクチュエータの設定
Actuator act1(PwmOutPins::MOTOR5_PWM, DigitalOutPins::MOTOR5_DIR);//トッピング回収
Actuator act2(PwmOutPins::MOTOR6_PWM, DigitalOutPins::MOTOR6_DIR);//昇降機

//モーターシミュレーション用
Actuator motorSim1(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR);
Actuator motorSim2(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR);
Actuator motorSim3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR);
Actuator motorSim4(PwmOutPins::MOTOR4_PWM, DigitalOutPins::MOTOR4_DIR);


// encoderの設定(encoder1が横向き、encoder2が縦向き)
Encoder encoder1(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, ENC_RES_MAX, 0, false);
Encoder encoder2(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, ENC_RES_MAX, 0, false);
//DT35の設定
DT35 dt35_1(AnalogInPins::DT35_1);//横の壁に垂直に当てる
DT35 dt35_2(AnalogInPins::DT35_2);//横の壁に斜めに当てる
DT35 dt35_3(AnalogInPins::DT35_3);//手前の壁に垂直に当てる

//BNO055の設定
BNO055 bno055(UartPins::BNO_RX, UartPins::BNO_TX); 

//重要なメモ R2の向きが0どの間(2つめのトッピングをとるまで)はカルマンフィルタでposをとり、最後はロータリのみ
//重要なメモ2 カルマンフィルタによって得られるPos自体はOdometry classで20Hz(?)で計算する(Tickerの無駄を減らせる)
//重要なメモ3 最初のR2の向きが0度、反時計回りが正の角度（構造体のみ)
DriveBase driveBase(&dt35_1, &dt35_2, &dt35_3, &bno055, &encoder1, &encoder2,&motor0, &motor1, &motor2, &motor3);

// オブザベーションの設定
Observation observation({&dt35_1, &dt35_2, &dt35_3});


Timer timer;

bool flag=false;

string str;


//ここで移動の関数をを組み合わせる
void drive(){


    //引数: 目標X座標[mm],目標Y座標[mm],目標方角[rad](絶対値はPI未満にする)
    //角度がワールドなのかロボットの向きなのかは要確認
    //自作localizationの値とライブラリの値の整合性も確認角度とか怖い
    //一旦R2の向きが0度としてみる
    //自己位置の計算再確認
    //goTo
    
    //10秒待機
    wait_us(1000000*10);

    //トッピング1回収
    driveBase.goTo(0, 1650, 0,true,true);
    act1.act(0.5,1,3);
    act1.act(0.5,0,3);

    //トッピング2回収
    driveBase.goTo(-150, 1300, 0,true,false);
    //arctan(150/350)=0.404891786
    driveBase.goCurveTo(-0.404891786, 0.404891786, -650, 1300, 0, false, 8);
    driveBase.goTo(-725, 1650, 0,true,true);
    act1.act(0.5,1,3);
    act1.act(0.5,0,3);

    //トッピングをトッピング!
    driveBase.goTo(-800, 1100, -PI/4,true,false);
    //arctan(75/550)=0.135527713
    driveBase.goCurveTo(-0.135527713, -PI/2, -2000, 950, -PI/2, true, 8);
    act2.act(0.5,1,3);

    //アーム戻している時間もったいないから　戻しつつ移動する方法を考える
    while(1){
        
     
    }
}

//シミュレーションすること

//1.各モーターが動くか　向きも確認
// motorSim1.act(0.5,1,3);

//2.各アクチュエータが動くか
// act1.act(0.5,1,3);

//3.BNO055の値が取れるか

void showBNO()
{
 while(1){
        printf("%d,%d\n",int(bno055.getDegrees()), int(bno055.getRadians()));
        wait_us(100000);
    }
}


//4.DT35の値が読めるか


void showDt35()
{
  while(1)
  {
    printf("%d,%d,%d\n",int(dt35_1.getObsDistance()), int(dt35_2.getObsDistance()), int(dt35_3.getObsDistance()));
    wait_us(100000);
  }
}


//5.ロータリエンコーダの値が取れるか


void showEncoder()
{
  while(1)
  {
    printf("%d,%d\n",int(encoder1.getMoveDistance()), int(encoder2.getMoveDistance()));
    wait_us(100000);
  }

}


//6.エンコーダー、ジャイロのみの自己位置推定ができるか
//まず,KALMAN_X,KALMAN_Y,KALMAN_THETAを0にする R2をわちゃわちゃ動かす
//角度はradでなくdegreeに注意

void showEncoderLocalization()
{
  while(1)
  {
    printf("%d,%d,%d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction));
    wait_us(100000);
  }
}


//7.DT35による自己位置推定ができているか
//角度はradでなくdegreeに注意

void showDT35Localization()
{
  while(1)
  {
    printf("%d,%d,%d\n",int(observation.getPoseObs().x), int(observation.getPoseObs().y), int(180/PI*observation.getPoseObs().theta));
    wait_us(100000);
  }
}


//8. カルマンフィルタ込みの自己位置推定ができているか
//カルマン定数を元に戻す　角度はradでなくdegreeに注意

void showLocalization()
{
  while(1)
  {
    printf("%d,%d,%d\n",int(driveBase.localization.posX), int(driveBase.localization.posY), int(180/PI*driveBase.localization.direction));
    wait_us(100000);
  }
}


//9. エンコーダーなしで移動できるか
//driveBase.runNoEncoder(0.5, 0.5, PI/4, 0.5, 3.0);

//10.目的地に移動できるか
//driveBase.goTo(1000, 1000, PI/2, true, true);



int main()
{
 drive();
//drive();の代わりにshow();を使うことで、位置情報をリアルタイムで表示できる
}