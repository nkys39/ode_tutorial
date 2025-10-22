// ===================================================================
// 17: LiDAR Sensor - LiDARセンサー（レーザー距離センサー）
// ===================================================================
// このチュートリアルでは、LiDARセンサーの基本を学びます。
// レイキャスティングで障害物までの距離を測定します。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;
dSpaceID space;
dGeomID ground_geom;

// ===================================================================
// 障害物の定義
// ===================================================================
// 静的な障害物（動かない物体）
//   bodyは持たず、geomのみ（衝突判定だけ必要）
struct Obstacle {
    dGeomID geom;           // ジオメトリ（形状）
    dReal x, y, z;          // 位置
    dReal lx, ly, lz;       // サイズ（長さ × 幅 × 高さ）
};
std::vector<Obstacle> obstacles;

// ===================================================================
// LiDARセンサー
// ===================================================================
LidarSensor* lidar;                     // LiDARセンサーオブジェクト
dReal sensor_pos[3] = {0, 0, 0.5};      // センサー位置（高さ0.5m）
dReal sensor_yaw = 0.0;                 // センサーの向き（ヨー角）

// ===================================================================
// 障害物作成関数
// ===================================================================
void createObstacles() {
    // 複数の箱型障害物を配置
    //   静的なので剛体(body)は不要、ジオメトリ(geom)のみ作成
    Obstacle obs;

    // 障害物1：前方2mに0.5×0.5×0.5の箱
    obs.x = 2; obs.y = 0; obs.z = 0.25;
    obs.lx = 0.5; obs.ly = 0.5; obs.lz = 0.5;
    obs.geom = dCreateBox(space, obs.lx, obs.ly, obs.lz);
    dGeomSetPosition(obs.geom, obs.x, obs.y, obs.z);
    obstacles.push_back(obs);

    // 障害物2：左前に0.3×0.3×0.5の箱
    obs.x = -1; obs.y = 1.5; obs.z = 0.25;
    obs.lx = 0.3; obs.ly = 0.3; obs.lz = 0.5;
    obs.geom = dCreateBox(space, obs.lx, obs.ly, obs.lz);
    dGeomSetPosition(obs.geom, obs.x, obs.y, obs.z);
    obstacles.push_back(obs);

    // 障害物3：左後ろに0.4×0.4×0.5の箱
    obs.x = -1; obs.y = -1.5; obs.z = 0.25;
    obs.lx = 0.4; obs.ly = 0.4; obs.lz = 0.5;
    obs.geom = dCreateBox(space, obs.lx, obs.ly, obs.lz);
    dGeomSetPosition(obs.geom, obs.x, obs.y, obs.z);
    obstacles.push_back(obs);

    // 障害物4：遠くに0.6×0.2×0.6の箱
    obs.x = 0; obs.y = 2.5; obs.z = 0.3;
    obs.lx = 0.6; obs.ly = 0.2; obs.lz = 0.6;
    obs.geom = dCreateBox(space, obs.lx, obs.ly, obs.lz);
    dGeomSetPosition(obs.geom, obs.x, obs.y, obs.z);
    obstacles.push_back(obs);
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    // ===================================================================
    // 1. センサーをゆっくり回転させる
    // ===================================================================
    // 0.5 rad/s（約28.6度/秒）で回転
    //   実際のLiDARは高速回転（5-20Hz）しますが、
    //   可視化のためゆっくり回転させています
    sensor_yaw += dt * 0.5;

    // ===================================================================
    // 2. LiDARスキャンを実行
    // ===================================================================
    // LidarSensor::scan(): レイキャスティングで距離測定
    //
    // LiDAR（Light Detection and Ranging）とは？
    //   レーザー光を放射し、反射して戻ってくるまでの時間から
    //   物体までの距離を測定するセンサーです
    //
    // 仕組み：
    //   1. センサーから多方向にレーザービームを照射
    //   2. 障害物に当たったビームが反射して戻る
    //   3. 往復時間から距離を計算（光速 × 時間 / 2）
    //   4. 角度ごとの距離データ（スキャンデータ）を取得
    //
    // シミュレーションでの実装：
    //   レイキャスティング（仮想的な光線を飛ばす）を使用
    //   dSpaceCollide系の関数で光線と障害物の交点を検出
    //
    // 2D LiDAR vs 3D LiDAR：
    //   2D: 1平面上のスキャン（このチュートリアル）
    //   3D: 複数平面または全方位スキャン
    //
    // 用途：
    //   - ロボットナビゲーション（障害物回避）
    //   - SLAM（地図作成と自己位置推定）
    //   - 自動運転車の周辺認識
    LidarScan scan = lidar->scan(sensor_pos, sensor_yaw);

    // ===================================================================
    // 3. スキャンデータの表示
    // ===================================================================
    // 50ステップごとに主要方向の距離を表示
    static int counter = 0;
    if (counter++ % 50 == 0) {
        std::cout << "LiDAR scan at yaw=" << radToDeg(sensor_yaw) << "°" << std::endl;

        // 4方向の距離を表示
        //   ranges[i]: i番目の光線の測定距離
        //   angles[i]: i番目の光線の角度
        //
        // インデックスと方向の対応：
        //   180: 前方（0度）
        //    90: 右側（90度）
        //     0: 後方（180度）
        //   270: 左側（270度）
        std::cout << "  Front (0°): " << scan.ranges[180] << " m" << std::endl;
        std::cout << "  Right (90°): " << scan.ranges[90] << " m" << std::endl;
        std::cout << "  Back (180°): " << scan.ranges[0] << " m" << std::endl;
        std::cout << "  Left (270°): " << scan.ranges[270] << " m" << std::endl;
    }

    // 物理シミュレーション（この例では動く物体はない）
    dWorldStep(world, dt);
}

// ===================================================================
// 描画コールバック関数
// ===================================================================
void drawScene() {
    // 地面を描画
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // 障害物を描画（赤色）
    for (const auto& obs : obstacles) {
        const dReal* pos = dGeomGetPosition(obs.geom);
        const dReal* R = dGeomGetRotation(obs.geom);
        dReal sides[3] = {obs.lx, obs.ly, obs.lz};

        Viewer::setColor(0.7f, 0.3f, 0.3f);
        Viewer::drawBox(pos, R, sides);
    }

    // センサー位置を描画（青い球）
    dMatrix3 R;
    dRSetIdentity(R);
    Viewer::setColor(0.3f, 0.7f, 0.9f);
    Viewer::drawSphere(sensor_pos, R, 0.1);

    // ===================================================================
    // LiDAR光線を描画（可視化）
    // ===================================================================
    const LidarScan& scan = lidar->getLastScan();

    // 10本に1本だけ描画（全部描くと重いため）
    for (size_t i = 0; i < scan.ranges.size(); i += 10) {
        // i番目の光線の角度と距離
        dReal angle = sensor_yaw + scan.angles[i];
        dReal range = scan.ranges[i];

        // 光線の終点座標を計算
        //   始点：sensor_pos
        //   終点：始点 + (距離 × 方向ベクトル)
        dReal end_x = sensor_pos[0] + range * std::cos(angle);
        dReal end_y = sensor_pos[1] + range * std::sin(angle);
        dReal end_z = sensor_pos[2];

        dReal end[3] = {end_x, end_y, end_z};

        // 距離に応じて色を変える
        //   近い（range小） → 青っぽい（color小）
        //   遠い（range大） → 黄色っぽい（color大）
        float color = 1.0f - (range / scan.max_range);
        Viewer::drawLine(sensor_pos, end, color, color, 1.0f);
    }
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 17: LiDAR Sensor（LiDARセンサー） ===" << std::endl;
    std::cout << "レイキャスティングで2D LiDARをシミュレート" << std::endl;
    std::cout << "センサーが回転して障害物を検出します" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);

    // 地面を作成
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // 障害物を作成
    createObstacles();

    // ===================================================================
    // LiDARセンサーを作成
    // ===================================================================
    // パラメーター：
    //   space: 衝突検出空間（障害物が含まれる）
    //   -M_PI: スキャン開始角度（-180度 = 後方）
    //   M_PI: スキャン終了角度（+180度 = 後方）
    //   360: 光線の本数（360本 = 1度ごと）
    //   10.0: 最大測定距離（10m）
    //   0.1: 最小測定距離（0.1m = 10cm、センサー近くは測定しない）
    //
    // 結果：
    //   360度全方位をカバーする2D LiDARセンサー
    //   角度分解能：1度
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 10.0, 0.1);

    // ビューワーの作成
    Viewer viewer(argc, argv, "17: LiDAR Sensor - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    // カメラ位置を調整（上から見下ろす）
    Camera& cam = Viewer::getCamera();
    cam.distance = 8.0;   // 距離：8m
    cam.pitch = 60.0;     // ピッチ：60度（斜め上から）

    viewer.start();

    // クリーンアップ
    delete lidar;
    for (auto& obs : obstacles) {
        dGeomDestroy(obs.geom);
    }
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
