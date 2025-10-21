# 02: Falling Sphere

重力下で落下する球体のシミュレーションを学びます。

## 学習内容

- 剛体（Rigid Body）の作成
- 質量（Mass）の設定
- ジオメトリ（Geometry）の作成と剛体への関連付け
- 物体の可視化

## 実行

```bash
cd build
make 02_falling_sphere
./02_falling_sphere/02_falling_sphere
```

## コードのポイント

### 球体の作成

```cpp
sphere = createSphere(world, space, 0, 0, 3.0, 0.2, 1.0);
```

`utils.h`のヘルパー関数を使って球体を作成します。
パラメータ: 位置(x,y,z), 半径, 質量

### 位置の取得

```cpp
const dReal* pos = dBodyGetPosition(sphere);
```

剛体の現在位置を取得します。
