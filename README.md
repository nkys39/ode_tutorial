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

### 5. ロボティクス編
- **15_differential_drive**: 差動2輪ロボットの基本実装
- **16_odometry**: オドメトリ（車輪エンコーダによる自己位置推定）
- **17_lidar_sensor**: LiDARセンサのシミュレーション（レイキャスティング）
- **18_camera_sensor**: カメラセンサのシミュレーション（深度カメラ含む）
- **19_robot_with_lidar**: 差動2輪ロボット + LiDARセンサの統合
- **20_walking_pedestrian**: 歩行する人のシミュレーション（移動障害物）
- **21_crowd_simulation**: 複数の歩行者のシミュレーション（人群）
- **22_dynamic_obstacle_avoidance**: 動的障害物（歩行者）を避けるロボット制御
- **23_slam_basics**: SLAM（自己位置推定と地図作成）の基礎
- **24_social_navigation**: 人混みの中でのソーシャルナビゲーション
- **25_autonomous_navigation**: 動的環境での自律ナビゲーションシステム

#### ロボティクス編で学べること
- **差動2輪ロボット**: 左右の車輪速度を制御して移動・旋回する基本的なロボット
- **オドメトリ**: 車輪の回転から位置・姿勢を推定する技術（累積誤差の影響も体験）
- **LiDARセンサ**: レーザーレイキャスティングによる距離測定と環境マップ作成
- **カメラセンサ**: RGB画像と深度情報の取得
- **歩行者シミュレーション**: カプセルやラグドールベースの移動障害物
- **人群シミュレーション**: Social Force Modelなどを使った複数歩行者の動き
- **動的障害物回避**: 移動する障害物を予測して回避する制御アルゴリズム
- **ソーシャルナビゲーション**: 人に配慮した自然な経路計画と動作
- **センサ融合**: 複数センサを組み合わせた高度なロボットシステム
- **動的環境での自律制御**: 静的・動的障害物が混在する環境でのナビゲーション

## プロジェクト構成

```
ode_tutorial/
├── README.md
├── CMakeLists.txt
├── common/                 # 共通ユーティリティ
│   ├── viewer.h           # OpenGL可視化ヘルパー
│   ├── viewer.cpp
│   ├── utils.h            # 共通ユーティリティ関数
│   ├── sensors.h          # センサーシミュレーション
│   ├── robot_utils.h      # ロボット関連ユーティリティ
│   └── pedestrian.h       # 歩行者シミュレーション
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
├── 15_differential_drive/
│   ├── main.cpp
│   └── README.md
├── 19_robot_with_lidar/
│   ├── main.cpp
│   └── README.md
├── 20_walking_pedestrian/
│   ├── main.cpp
│   └── README.md
├── 22_dynamic_obstacle_avoidance/
│   ├── main.cpp
│   └── README.md
...
└── docs/                   # ドキュメント
    ├── ode_basics.md      # ODE基礎知識
    ├── collision.md       # 衝突検出の詳細
    ├── joints.md          # ジョイントの詳細
    ├── robotics.md        # ロボティクス応用
    ├── sensors.md         # センサーシミュレーション
    ├── odometry.md        # オドメトリの理論と実装
    ├── pedestrian.md      # 歩行者シミュレーション
    └── dynamic_obstacles.md  # 動的障害物回避の理論
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

### ODE関連
- [ODE公式サイト](https://www.ode.org/)
- [ODE公式ドキュメント](https://ode.org/wiki/index.php/Manual)
- [ODE APIリファレンス](https://ode.org/wiki/index.php/HOWTO)

### ロボティクス関連
- 差動2輪ロボットの運動学（Differential Drive Kinematics）
- オドメトリの理論と誤差解析
- LiDARセンサの原理とレイキャスティング
- SLAM（Simultaneous Localization and Mapping）の基礎
- 歩行者シミュレーション（Social Force Model）
- 動的障害物回避（Dynamic Window Approach, Velocity Obstacles）
- ソーシャルナビゲーション（人に配慮したロボット行動）
- ROS（Robot Operating System）との連携可能性

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
