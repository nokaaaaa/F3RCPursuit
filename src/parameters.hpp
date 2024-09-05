//パラメーター

#pragma once


#define USING_R1 (1)
#define SIMULATION (0)

//定数
#define PI (3.141592653589793)
#define SQRT2 (1.414213562373095)

//全部まだ仮の値

//固定値
#define ENC_RES_MAX (2048) //ロータリエンコーダーの分解能
#define TRED_RADIUS (125.4f) //中心からオムニホイールの距離[mm]
#define WHEEL_DIAMETER (100.0f)	//オムニホイールの直径	[mm]
#define WHEEL_RADIUS	(WHEEL_DIAMETER/2.0f) //オムニホイールの半径[mm]
#define MMPP 		((WHEEL_DIAMETER*PI)/(ENC_RES_MAX))	//エンコーダ1パルスあたりに進む距離[mm]
#define RADPP 	(MMPP/TRED_RADIUS)	//エンコーダ1パルスあたりの回転角[rad]
#define WHEEL_ENCODER_RADIUS (100.0f)//エンコーダーの半径[mm]

#define DT_MIN_DISTANCE (10.0f) //DT35と物体の距離が近いときの距離[mm]
#define DT_MIN_ANALOG (0.1f) //DT35と物体の距離が近いときの電流 
#define DT_MAX_DISTANCE (2000.0f) //DT35と物体の距離が遠いときの距離[mm]
#define DT_MAX_ANALOG (3.0f) //DT35と物体の距離が遠いときの電流

#define DT_1_x (125.4f)//DT35_1のR2の中心を原点としたときのx座標[mm]
#define DT_1_y (0.0f)//DT35_1のR2中心を原点としたときのy座標[mm]
#define DT_1_degree (0.0) //壁に当てるDT35_2の角度[rad]

#define DT_2_x (125.4f)//DT35_2の中心を原点としたときのx座標[mm]
#define DT_2_y (-10.0f)//DT35_2の中心を原点としたときのy座標[mm]
#define DT_2_degree (330*PI/180) //壁に斜めに当てるDT35_2の角度[rad]

#define DT_3_x (0.0f)//DT35_3の中心を原点としたときのx座標[mm]
#define DT_3_y (-125.4f)//DT35_3の中心を原点としたときのy座標[mm]
#define DT_3_degree (270*PI/180) //壁に当てるDT35_3の角度[rad]

#define Distance_VERTICAL_WALL (125.4f) //縦の壁からR2の中心の初期位置[mm]
#define Distance_BESIDE_WALL (125.4f) //横(手前）の壁からR2の中心の初期位置[mm]

#define KALMAN_X (0.5f)//1に近づくほどObservationを信用する
#define KALMAN_Y (0.5f)//1に近づくほどObservationを信用する
#define KALMAN_THETA (0.5f)//1に近づくほどObservationを信用する

#define KALMAN_OFF (-975.0f) //カルマンフィルタをオフにするx座標[mm]

#define PWM_PERIOD (20.0f) //PWMの周期[s]



//モーターのPIDゲイン

#define MOTOR_0_KP_1 (1.3f)
#define MOTOR_0_KI_1 (0.06f)
#define MOTOR_0_KD_1 (0.0f)

#define MOTOR_1_KP_1 (1.3f)
#define MOTOR_1_KI_1 (0.06f)
#define MOTOR_1_KD_1 (0.0f)

#define MOTOR_2_KP_1 (1.3f)
#define MOTOR_2_KI_1 (0.06f)
#define MOTOR_2_KD_1 (0.0f)

#define MOTOR_3_KP_1 (1.3f)
#define MOTOR_3_KI_1 (0.06f)
#define MOTOR_3_KD_1 (0.0f)

#define MOTOR_0_KP_2 (0.00014f)
#define MOTOR_0_KI_2 (0.000000f)
#define MOTOR_0_KD_2 (0.00000f)

#define MOTOR_1_KP_2 (0.00016f)
#define MOTOR_1_KI_2 (0.00000f)
#define MOTOR_1_KD_2 (0.000000f)

#define MOTOR_2_KP_2 (0.00016f)
#define MOTOR_2_KI_2 (0.00000f)
#define MOTOR_2_KD_2 (0.00000f)

#define MOTOR_3_KP_2 (0.00016f)
#define MOTOR_3_KI_2 (0.000000f)
#define MOTOR_3_KD_2 (0.0f)

#define DRIVEBASE_KP (1.3f)
#define DRIVEBASE_KI (0.08f)
#define DRIVEBASE_KD (0.0f)

#define DRIVEBASE_ROTATE_KP (DRIVEBASE_KP)
#define DRIVEBASE_ROTATE_KI (DRIVEBASE_KI)
#define DRIVEBASE_ROTATE_KD (DRIVEBASE_KD)


//移動パラメータ
#define MAX_ACCELERATION (4000) //最大加速度 [mm/s^2]
#define MAX_SPEED (1000) //最高速度  [mm/s]
#define MAX_ROTATE_ACCELERATION (MAX_ACCELERATION/TRED_RADIUS) //最大回転速度  [rad/s^2]
#define MAX_ROTATE_SPEED (MAX_SPEED/TRED_RADIUS) //最大回転加速度  [rad/s]

//制御周期など
#define ENCODER_LOCALIZATION_FREQUENCY (200) //エンコーダーによる自己位置推定の頻度[Hz]
#define SPEED_ADJUSTMENT_FREQUENCY (20) //速度調整の頻度[Hz]

//目的地到着を判定する閾値
#define MOVEMENT_THRESHOLD (5) //目的地に到着したとみなす半径[mm]
#define MOVEMENT_THRESHOLD_RAD (0.04f) //目的地に到着したとみなす角度の誤差[rad]

#define MOVEMENT_SWITCH_THRESHOLD (MAX_SPEED/DRIVEBASE_KP)

#define MAX_DELTA_R ((MAX_SPEED)/(SPEED_ADJUSTMENT_FREQUENCY))
#define MAX_DELTA_D ((MAX_ROTATE_SPEED)/(SPEED_ADJUSTMENT_FREQUENCY))

//コントローラー等
#define ADJUSTMENT_SPEED_LEFT (100.0f)
#define ADJUSTMENT_SPEED_RIGHT (30.0f)
#define ADJUSTMENT_ROTATE_SPEED (0.5f)

#define HAND_SPEED_LEFT (1500.0f)
#define HAND_SPEED_RIGHT (100.0f)
#define HAND_ROTATE_SPEED (0.5f)



