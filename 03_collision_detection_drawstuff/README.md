# 03: Collision Detection (DrawStuff版)

## 説明

球が地面に衝突してバウンドするシミュレーションです。衝突検出と接触ジョイントの使い方を学びます。

## 実行方法

```bash
./03_collision_detection_drawstuff
```

## 操作

- **R**: 球をリセット
- **スペース**: 一時停止/再開
- **ESC**: 終了

## 学べること

- 衝突空間（dSpaceID）の作成と使用
- 衝突検出コールバック（nearCallback）
- 接触ジョイントの作成
- 反発係数（bounce）と摩擦（mu）の設定
- dSpaceCollideによる衝突検出
