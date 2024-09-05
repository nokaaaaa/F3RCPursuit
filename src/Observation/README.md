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
(画像)
