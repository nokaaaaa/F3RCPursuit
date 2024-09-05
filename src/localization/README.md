# localizationクラス

## 自己位置推定
![IMG_0021](https://github.com/user-attachments/assets/427fe9f0-1b9d-4246-9a00-fe7e09002e24)
![IMG_0022](https://github.com/user-attachments/assets/c7d15dc1-eb2a-41bb-bba0-aea56d76df61)
![IMG_0023](https://github.com/user-attachments/assets/674d312a-2fb7-4e35-b18e-3d8e1a755d58)

##　各エンコーダーの速度[mm/s]
自己速度から各4つのモーターの速度を取得する。
(参考文献:https://qiita.com/IshitaTakeshi/items/740ac7e9b549eee4cc04)
ここで注意なのは、もとのライブラリの driveMotor.cppのrotate関数のspeedがモーターの移動距離[mm]/sより、角速度でないことに注意 これより、motorSpeedは[mm/s]である。
なので、最後の式の1/rはいらない α_i(i=0,1,2,3)=π/4 + i×π/2　を代入すればいい
motor0:右斜め前 motor1:左斜め前 motor2:左斜め後ろ motor3:右斜め後ろについてる
