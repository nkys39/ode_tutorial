# 01: Hello World

ODEの初期化と基本的な世界の作成を学びます。

## 学習内容

- ODEライブラリの初期化 (`dInitODE()`)
- 物理世界の作成 (`dWorldCreate()`)
- 重力の設定 (`dWorldSetGravity()`)
- 衝突検出空間の作成 (`dHashSpaceCreate()`)
- シミュレーションステップの実行 (`dWorldStep()`)

## ビルドと実行

```bash
cd build
make 01_hello_world
./01_hello_world/01_hello_world
```

## 操作方法

- **マウスドラッグ**: カメラ回転
- **マウスホイール**: ズームイン/アウト
- **スペースキー**: シミュレーション一時停止/再開
- **R キー**: リセット
- **Q/ESC キー**: 終了

## コードのポイント

### ODEの初期化

```cpp
dInitODE();
```

ODEライブラリを初期化します。プログラムの最初に1回だけ呼び出します。

### 物理世界の作成

```cpp
world = dWorldCreate();
dWorldSetGravity(world, 0, 0, -9.81);
```

物理シミュレーションの世界を作成し、重力を設定します。
重力は(x, y, z)方向のベクトルで指定します。

### 衝突検出空間の作成

```cpp
space = dHashSpaceCreate(0);
```

オブジェクト間の衝突を検出するための空間を作成します。

### クリーンアップ

```cpp
dSpaceDestroy(space);
dWorldDestroy(world);
dCloseODE();
```

プログラム終了時に、作成したリソースを解放します。

## 次のステップ

次のチュートリアル [02_falling_sphere](../02_falling_sphere/) では、
実際にオブジェクトを作成してシミュレーションします。
