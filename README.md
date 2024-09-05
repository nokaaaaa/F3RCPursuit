# F3RC2024

R2の制御とシミュレーションを行うプログラム 非制御の方は 使い方とパラメータ、ピンの設定 を見てもらえればとりあえず動かせると思います
## 使い方

main.cppにある void drive() 内で動かす
※目的地はワールド座標であり、R2の初期位置をx=0[mm],y=0[mm] 正面方向をΘ=0[rad]とする

### 1　目的地(x[mm],y[mm],Θ[rad]) に直線運動
```cpp
driveBase.goTo(x, y, Θ); 
```
### 2 目的地に曲線移動
```cpp
driveBase.goCurveTo(float start_dir, float end_dir, float X, float Y, float D, bool stop, int num);
```

[1],[2]を組み合わせ次のような経路を設計した
(画像)
## パラメーター、ピンの設定

### ピンの設定
[pins](src/pins.hpp)からピン名(PA_4とか)を変えてください

### パラメーターの設定
//固定値　にあって()の中身が数字のものを設定してください
わかりにくそうな定数の説明を以下に記します

#### DT35
```cpp
#define DT_1_x (125.4f)//DT35_1の中心を原点としたときのx座標[mm]
#define DT_1_y (0.0f)//DT35_1の中心を原点としたときのy座標[mm]
#define DT_1_degree (0.0) //壁に当てるDT35_1の角度[rad]

#define DT_2_x (125.4f)//DT35_2の中心を原点としたときのx座標[mm]
#define DT_2_y (-10.0f)//DT35_2の中心を原点としたときのy座標[mm]
#define DT_2_degree (330*PI/180) //壁に斜めに当てるDT35_2の角度[rad]

#define DT_3_x (0.0f)//DT35_3の中心を原点としたときのx座標[mm]
#define DT_3_y (-125.4f)//DT35_3の中心を原点としたときのy座標[mm]
#define DT_3_degree (270*PI/180) //壁に当てるDT35_3の角度[rad]
```
画像()

####　壁からの距離
```cpp
#define Distance_VERTICAL_WALL (125.4f) //縦の壁からR2の中心の初期位置[mm]
#define Distance_BESIDE_WALL (125.4f) //横(手前）の壁からR2の中心の初期位置[mm]
```
画像()


#### カルマンフィルタ
```cpp
#define KALMAN_X (0.5f)//1に近づくほどObservationを信用する
#define KALMAN_Y (0.5f)//1に近づくほどObservationを信用する
#define KALMAN_THETA (0.5f)//1に近づくほどObservationを信用する
#define KALMAN_OFF (-975.0f) //カルマンフィルタをオフにするx座標[mm]
```
ここは無視してください 最後の定数は2つめのトッピングを回収する場所のx座標です


## シミュレーション
以下制御用
```cpp
show();
```
リアルタイムで位置を取得

## 経路設計

こんな感じで移動させたい
画像()

## 9軸センサBNO055

[BNO055](src/BNO055/README.md)

## DT35

[DT35](src/DT35/README.md)

## Obseravtion
[Observation](src/Observation/README.md)


## Encoder
[Encoder](src/Encoder/README.md)

## localization
[localization](src/localization/README.md)

