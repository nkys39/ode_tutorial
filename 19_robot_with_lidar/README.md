# 19: Robot with LiDAR

差動2輪ロボットとLiDARセンサを統合したシミュレーションを学びます。

## 学習内容

- ロボットへのセンサー搭載
- LiDARデータを使った障害物検出
- 簡単な障害物回避アルゴリズム
- センサーとアクチュエーターの統合

## 実行

```bash
cd build
make 19_robot_with_lidar
./19_robot_with_lidar/19_robot_with_lidar
```

## 機能

### 自動障害物回避

前方0.8m以内に障害物を検出すると、自動的に左旋回します。

```cpp
// LiDARスキャン
LidarScan scan = lidar->scan(robot_pos, robot_yaw);

// 前方の最小距離をチェック
if (min_front_distance < 0.8) {
    target_angular_vel = 1.0;  // 左旋回
}
```

### 可視化

- 緑色のボックス: ロボット本体
- 黒い円柱: 車輪
- 赤いボックス: 障害物
- カラフルな線: LiDARレイ（距離に応じて色が変化）

## 操作

**W**キーで前進すると、障害物を自動的に避けながら移動します。
