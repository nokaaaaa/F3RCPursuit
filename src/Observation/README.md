# Observation
DT35のセンサから自己位置を取得するクラスです

##　使い方
```cpp
// オブザベーションの設定
Observation observation({&dt35_1, &dt35_2, &dt35_3});
```
##　値の取得
```cpp
observation.getPoseObs().x//x座標
observation.getPoseObs().y//y座標
observation.getPoseObs().theta//R2の角度(初期位置の正面をΘ=0とする)
```

##　解説
![IMG_0015](https://github.com/user-attachments/assets/e34c6ac8-4620-4a3a-aa7f-26a803a6cddd)
![IMG_0016](https://github.com/user-attachments/assets/242a7435-8ce9-4da4-8a6e-dee31aa4c18a)
![IMG_0017](https://github.com/user-attachments/assets/e9c8a814-d72a-4ea7-b5dc-973dc31827fd)
![IMG_0018](https://github.com/user-attachments/assets/41c4269c-2c5d-4dc8-b74d-9ddb84a59746)
![IMG_0019](https://github.com/user-attachments/assets/526caf2c-3824-40bd-be98-5d04a0071591)
![IMG_0020](https://github.com/user-attachments/assets/4289926c-20b4-43a7-bc8f-c38b3e2dc046) 
