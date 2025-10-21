# 22: Dynamic Obstacle Avoidance

動的障害物（歩行者）を避けるロボット制御を学びます。

## 学習内容

- 静的障害物と動的障害物の統合
- LiDARデータを使った動的障害物検出
- リアルタイム障害物回避アルゴリズム
- センサーベース自律ナビゲーション

## 実行

```bash
cd build
make 22_dynamic_obstacle_avoidance
./22_dynamic_obstacle_avoidance/22_dynamic_obstacle_avoidance
```

## システム構成

### センサー
- **LiDAR**: 360度スキャン、最大6m範囲

### アクチュエーター
- **差動2輪駆動**: 移動と旋回

### 環境
- **静的障害物**: 赤いボックス（固定）
- **動的障害物**: ピンク/紫のカプセル（歩行者）

## 障害物回避アルゴリズム

```cpp
// LiDARスキャンを3つのセクターに分割
left_sector   = 210-270° (左側)
front_sector  = 150-210° (正面)
right_sector  = 90-150°  (右側)

// 決定ロジック
if (front_distance < 1.5m) {
    // 前方に障害物 - より空いている方向へ旋回
    if (left_distance > right_distance)
        turn_left()
    else
        turn_right()
} else if (front_distance < 2.5m) {
    // 減速して接近
    slow_down()
} else {
    // 直進
    move_forward()
}
```

## 操作

- **T**: 自動運転モードのON/OFF
- **W/A/S/D**: 手動制御（自動運転OFF時）
- **X**: 停止

デフォルトで自動運転モードが有効です。ロボットが静的・動的障害物を避けながら移動する様子を観察してください！

## 可視化

- **緑のボックス**: ロボット本体
- **赤いボックス**: 静的障害物
- **ピンク/紫のカプセル**: 動的障害物（歩行者）
- **色付きの線**: LiDARレイ（距離により色が変化）

## 発展課題

1. より高度な経路計画アルゴリズムの実装（DWA, VFHなど）
2. 歩行者の動きを予測する機能の追加
3. 複数のロボットによる協調回避
