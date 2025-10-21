# ODE チュートリアル

ODE (Open Dynamics Engine) を使った3D物理シミュレーションのチュートリアルプロジェクトです。

## ODEとは

ODE（Open Dynamics Engine）は、オープンソースの高性能な3D剛体力学シミュレーションライブラリです。ゲーム開発、ロボティクス、物理シミュレーションなど、様々な分野で使用されています。

### 主な機能
- 剛体の動力学シミュレーション
- 衝突検出
- ジョイント（関節）のシミュレーション
- 摩擦、反発係数などの物理パラメータ設定
- 様々な形状のサポート（球、箱、カプセル、平面など）

## 環境要件

### 必要なソフトウェア
- C++コンパイラ（g++またはclang）
- ODE ライブラリ（libode-dev）
- OpenGL / GLUT（可視化用）
- CMake（ビルドシステム）

### Ubuntu/Debianの場合
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libode-dev
sudo apt-get install freeglut3-dev
sudo apt-get install cmake
```

### macOSの場合
```bash
brew install ode
brew install freeglut
brew install cmake
```

## チュートリアル内容

このチュートリアルでは、基礎から応用まで段階的にODEの使い方を学びます。

### 1. 基礎編
- **01_hello_world**: ODEの初期化と基本的な世界の作成
- **02_falling_sphere**: 重力下で落下する球体のシミュレーション
- **03_collision_detection**: 地面との衝突検出と反発

### 2. 形状とオブジェクト
- **04_multiple_objects**: 複数のオブジェクトのシミュレーション
- **05_different_shapes**: 様々な形状（箱、円柱、カプセルなど）の使用
- **06_compound_objects**: 複合オブジェクトの作成

### 3. ジョイント（関節）
- **07_hinge_joint**: ヒンジジョイント（蝶番）の実装
- **08_slider_joint**: スライダージョイントの実装
- **09_ball_joint**: ボールジョイント（球面関節）の実装
- **10_motor**: モーター付きジョイントの制御

### 4. 応用編
- **11_ragdoll**: ラグドール（人形）のシミュレーション
- **12_vehicle**: 車のシミュレーション
- **13_robot_arm**: ロボットアームのシミュレーション
- **14_physics_playground**: インタラクティブな物理シミュレーション環境

## プロジェクト構成

```
ode_tutorial/
├── README.md
├── CMakeLists.txt
├── common/                 # 共通ユーティリティ
│   ├── viewer.h           # OpenGL可視化ヘルパー
│   ├── viewer.cpp
│   └── utils.h            # 共通ユーティリティ関数
├── 01_hello_world/
│   ├── main.cpp
│   └── README.md
├── 02_falling_sphere/
│   ├── main.cpp
│   └── README.md
├── 03_collision_detection/
│   ├── main.cpp
│   └── README.md
...
└── docs/                   # ドキュメント
    ├── ode_basics.md      # ODE基礎知識
    ├── collision.md       # 衝突検出の詳細
    └── joints.md          # ジョイントの詳細
```

## ビルド方法

### 全サンプルをビルド
```bash
mkdir build
cd build
cmake ..
make
```

### 個別のサンプルをビルド
```bash
cd build
make 01_hello_world
```

## 実行方法

```bash
cd build
./01_hello_world/01_hello_world
```

### 操作方法（OpenGL可視化版）
- **マウスドラッグ**: カメラ回転
- **マウスホイール**: ズームイン/アウト
- **スペースキー**: シミュレーション一時停止/再開
- **R キー**: リセット
- **Q/ESC キー**: 終了

## 学習の進め方

1. **順番に実行する**: 01から順に実行して、段階的に理解を深めてください
2. **コードを読む**: 各サンプルのmain.cppとREADME.mdを読んで、実装を理解してください
3. **パラメータを変更する**: 物理パラメータ（質量、摩擦係数など）を変更して、動作の違いを観察してください
4. **改造してみる**: 既存のコードを改造して、独自のシミュレーションを作成してください

## 参考資料

- [ODE公式サイト](https://www.ode.org/)
- [ODE公式ドキュメント](https://ode.org/wiki/index.php/Manual)
- [ODE APIリファレンス](https://ode.org/wiki/index.php/HOWTO)

## トラブルシューティング

### ODEライブラリが見つからない
```bash
# ライブラリのパスを確認
pkg-config --cflags --libs ode

# 環境変数を設定
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### OpenGLウィンドウが表示されない
- GLUTが正しくインストールされているか確認してください
- リモート環境の場合、X11フォワーディングが有効か確認してください

## ライセンス

このチュートリアルプロジェクトはMITライセンスの下で公開されています。
ODEライブラリ自体はBSD/LGPLライセンスです。

## 貢献

バグ報告、機能追加のリクエスト、プルリクエストを歓迎します。

## 作者

ODEチュートリアルプロジェクト

---

**Have fun with physics simulation!**
