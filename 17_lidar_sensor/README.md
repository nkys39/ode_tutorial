# 17: LiDAR Sensor Simulation

LiDARセンサのシミュレーション（レイキャスティング）を学びます。

## 学習内容

- レイキャスティングによる距離測定
- 360度スキャン
- 障害物検出
- センサデータの可視化

## 実行

```bash
cd build
make 17_lidar_sensor
./17_lidar_sensor/17_lidar_sensor
```

## LiDARとは

Light Detection and Ranging（光検出と測距）の略。
レーザーを使って周囲の物体までの距離を測定するセンサーです。

### レイキャスティング

ODEの`dCreateRay()`と`dCollide()`を使って、レーザー光線と物体の交差を検出します。

```cpp
LidarScan scan = lidar->scan(position, yaw);
```

360度の距離データを取得できます。
