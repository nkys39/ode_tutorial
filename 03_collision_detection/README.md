# 03: Collision Detection

地面との衝突検出と反発のシミュレーションを学びます。

## 学習内容

- 衝突検出のコールバック関数
- 接触点（Contact）の設定
- 反発係数と摩擦係数の設定
- 地面（Plane）ジオメトリの作成

## コードのポイント

### 衝突検出

```cpp
dSpaceCollide(space, 0, &nearCallback);
```

空間内のオブジェクト間の衝突をチェックします。

### 接触パラメータ

```cpp
contact[i].surface.bounce = 0.8;  // 反発係数
contact[i].surface.mu = 0.5;      // 摩擦係数
```
