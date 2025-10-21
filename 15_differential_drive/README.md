# 15: Differential Drive Robot

差動2輪ロボットの基本実装を学びます。

## 学習内容

- 差動2輪ロボットの運動学
- ヒンジジョイントによる車輪の実装
- モーター制御
- キーボード入力による制御

## 実行

```bash
cd build
make 15_differential_drive
./15_differential_drive/15_differential_drive
```

## 操作

- **W**: 前進
- **S**: 後退
- **A**: 左旋回
- **D**: 右旋回
- **X**: 停止

## 差動2輪ロボットとは

左右の車輪の速度を独立に制御することで、移動と旋回を実現するロボットです。

### 運動学

```cpp
linear_vel = R * (v_left + v_right) / 2
angular_vel = R * (v_right - v_left) / L
```

- R: 車輪半径
- L: 車輪間距離（ホイールベース）
- v_left, v_right: 左右の車輪速度
