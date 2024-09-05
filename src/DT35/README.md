# DT35

## 使い方
```cpp
//初期化
DT35 dt35(PinName analog_in_pin);
```
## 値の取得
```cpp
float dt35.getObsDistance();
```
getObsdstance()で壁とDT35の距離[mm]を取得することができる　