// ===================================================================
// 19: Robot with LiDAR - LiDAR搭載ロボット（障害物回避）
// ===================================================================
// このチュートリアルでは、LiDARセンサーを搭載したロボットを作ります。
// センサーデータを使って簡単な障害物回避を実装します。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"
#include "sensors.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// ===================================================================
// ロボットコンポーネント
// ===================================================================
dBodyID robot_body;              // ロボット本体
dBodyID left_wheel, right_wheel; // 左右の車輪
dJointID left_hinge, right_hinge;// 車輪のヒンジジョイント

// ロボットパラメーター
const dReal WHEEL_RADIUS = 0.05;   // 車輪半径：5cm
const dReal WHEEL_WIDTH = 0.03;    // 車輪幅：3cm
const dReal WHEEL_BASE = 0.3;      // 車輪間隔：30cm
const dReal BODY_LENGTH = 0.4;     // 本体長さ：40cm
const dReal BODY_WIDTH = 0.3;      // 本体幅：30cm
const dReal BODY_HEIGHT = 0.15;    // 本体高さ：15cm

// LiDARセンサー
LidarSensor* lidar;

// 障害物
struct Obstacle {
    dBodyID body;
    dGeomID geom;
    dReal lx, ly, lz;  // サイズ
};
std::vector<Obstacle> obstacles;

// ロボット制御
DifferentialDrive* diff_drive;     // 差動駆動制御
dReal target_linear_vel = 0.0;     // 目標直進速度
dReal target_angular_vel = 0.0;    // 目標角速度

// ===================================================================
// 衝突コールバック関数
// ===================================================================
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;

    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        // mu = 10.0: 高摩擦で車輪が滑りにくい
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 10.0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// 障害物作成関数
// ===================================================================
void createObstacles() {
    Obstacle obs;

    // 障害物を複数配置
    obs.lx = 0.5; obs.ly = 0.5; obs.lz = 0.5;
    obs.body = createBox(world, space, 2, 0, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.3; obs.ly = 0.3; obs.lz = 0.5;
    obs.body = createBox(world, space, -1, 1.5, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.4; obs.ly = 0.4; obs.lz = 0.5;
    obs.body = createBox(world, space, -1, -1.5, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.6; obs.ly = 0.2; obs.lz = 0.6;
    obs.body = createBox(world, space, 1, 2, 0.3, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.4; obs.ly = 0.4; obs.lz = 0.4;
    obs.body = createBox(world, space, -2, -1, 0.2, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);
}

// ===================================================================
// ロボット作成関数
// ===================================================================
void createRobot(dReal x, dReal y, dReal z) {
    // 1. ロボット本体を作成
    robot_body = createBox(world, space, x, y, z,
                          BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, 5.0);

    // 2. 車輪を作成（本体の下部に配置）
    dReal wheel_center_z = z - BODY_HEIGHT / 2;
    left_wheel = createCylinder(world, space,
                               x - WHEEL_BASE / 2, y, wheel_center_z,
                               WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);

    right_wheel = createCylinder(world, space,
                                x + WHEEL_BASE / 2, y, wheel_center_z,
                                WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dBodySetRotation(right_wheel, R);

    // 3. ヒンジジョイントで車輪を接続
    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, x - WHEEL_BASE / 2, y, wheel_center_z);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, x + WHEEL_BASE / 2, y, wheel_center_z);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    // 4. 差動駆動制御を初期化
    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
}

// ===================================================================
// リセット関数
// ===================================================================
void reset() {
    // ロボットを初期位置に戻す
    dReal correct_height = WHEEL_RADIUS + BODY_HEIGHT / 2;
    dBodySetPosition(robot_body, 0, 0, correct_height);
    dBodySetLinearVel(robot_body, 0, 0, 0);
    dBodySetAngularVel(robot_body, 0, 0, 0);

    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 1, 0);
    dBodySetRotation(robot_body, R);
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    // リセット処理
    if (Viewer::shouldReset()) {
        reset();
        Viewer::setShouldReset(false);
    }

    // ===================================================================
    // 1. ロボットの現在位置と姿勢を取得
    // ===================================================================
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal yaw = getYawFromRotation(R);

    // ===================================================================
    // 2. LiDARスキャンを実行
    // ===================================================================
    // ロボット位置からLiDARスキャン
    dReal lidar_pos[3] = {pos[0], pos[1], pos[2]};
    LidarScan scan = lidar->scan(lidar_pos, yaw);

    // ===================================================================
    // 3. 簡易障害物回避アルゴリズム
    // ===================================================================
    // 前方60度範囲（インデックス150〜210）の最小距離を計算
    dReal min_front_distance = scan.max_range;
    for (int i = 150; i < 210; i++) {
        if (scan.ranges[i] < min_front_distance) {
            min_front_distance = scan.ranges[i];
        }
    }

    // 障害物が0.8m以内にあれば旋回
    //
    // 障害物回避アルゴリズム（リアクティブ方式）：
    //   1. LiDARで前方をスキャン
    //   2. 前方に障害物があるか判定
    //   3. 障害物があれば旋回（回避行動）
    //   4. なければ直進
    //
    // この手法の特徴：
    //   利点：
    //     - シンプルで実装が簡単
    //     - リアルタイム動作
    //     - センサーノイズに強い
    //   欠点：
    //     - 局所最適（行き止まりで動けなくなる）
    //     - 目標位置への最短経路は見つけられない
    //     - U字型の障害物で往復してしまう
    //
    // より高度な手法：
    //   - DWA（Dynamic Window Approach）
    //   - VFH（Vector Field Histogram）
    //   - ポテンシャル場法
    if (min_front_distance < 0.8 && target_linear_vel > 0) {
        std::cout << "Obstacle ahead at " << min_front_distance << "m - turning!" << std::endl;
        target_angular_vel = 1.0;  // 左旋回
    }

    // ===================================================================
    // 4. モーター制御を適用
    // ===================================================================
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel,
                                      left_vel, right_vel);

    dJointSetHingeParam(left_hinge, dParamVel, left_vel);
    dJointSetHingeParam(left_hinge, dParamFMax, 10.0);

    dJointSetHingeParam(right_hinge, dParamVel, right_vel);
    dJointSetHingeParam(right_hinge, dParamFMax, 10.0);

    // 衝突判定
    dSpaceCollide(space, 0, &nearCallback);

    // 物理シミュレーション
    dWorldStep(world, dt);

    // 接触ジョイントをクリア
    dJointGroupEmpty(contact_group);
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
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);
        dReal sides[3] = {obs.lx, obs.ly, obs.lz};

        Viewer::setColor(0.7f, 0.3f, 0.3f);
        Viewer::drawBox(pos, R, sides);
    }

    // ロボット本体を描画（緑色）
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);

    // 車輪を描画（黒色）
    const dReal* lw_pos = dBodyGetPosition(left_wheel);
    const dReal* lw_R = dBodyGetRotation(left_wheel);
    Viewer::setColor(0.2f, 0.2f, 0.2f);
    Viewer::drawCylinder(lw_pos, lw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    const dReal* rw_pos = dBodyGetPosition(right_wheel);
    const dReal* rw_R = dBodyGetRotation(right_wheel);
    Viewer::drawCylinder(rw_pos, rw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    // ===================================================================
    // LiDAR光線を描画（可視化）
    // ===================================================================
    dReal yaw = getYawFromRotation(R);
    const LidarScan& scan = lidar->getLastScan();
    for (size_t i = 0; i < scan.ranges.size(); i += 10) {
        dReal angle = yaw + scan.angles[i];
        dReal range = scan.ranges[i];

        dReal end_x = pos[0] + range * std::cos(angle);
        dReal end_y = pos[1] + range * std::sin(angle);
        dReal end_z = pos[2];

        dReal start[3] = {pos[0], pos[1], pos[2]};
        dReal end[3] = {end_x, end_y, end_z};

        // 距離に応じて色を変える
        float color = 1.0f - (range / scan.max_range);
        Viewer::drawLine(start, end, color, 1.0f, color);
    }
}

// ===================================================================
// キーボードコールバック関数
// ===================================================================
void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w':
        case 'W':
            target_linear_vel = 0.5;
            target_angular_vel = 0.0;
            std::cout << "Move forward" << std::endl;
            break;
        case 's':
        case 'S':
            target_linear_vel = -0.3;
            target_angular_vel = 0.0;
            std::cout << "Move backward" << std::endl;
            break;
        case 'a':
        case 'A':
            target_linear_vel = 0.3;
            target_angular_vel = 1.0;
            std::cout << "Turn left" << std::endl;
            break;
        case 'd':
        case 'D':
            target_linear_vel = 0.3;
            target_angular_vel = -1.0;
            std::cout << "Turn right" << std::endl;
            break;
        case 'x':
        case 'X':
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
            std::cout << "Stop" << std::endl;
            break;
    }
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 19: Robot with LiDAR（LiDAR搭載ロボット） ===" << std::endl;
    std::cout << "差動駆動ロボットにLiDARセンサーを搭載" << std::endl;
    std::cout << "簡易障害物回避アルゴリズムを実装！" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  W: Move forward (with auto obstacle avoidance)" << std::endl;
    std::cout << "  S: Move backward" << std::endl;
    std::cout << "  A: Turn left" << std::endl;
    std::cout << "  D: Turn right" << std::endl;
    std::cout << "  X: Stop" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    // 地面を作成
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // 障害物を作成
    createObstacles();

    // ロボットを作成（正しい高さ = 車輪半径 + 本体高さの半分）
    dReal robot_height = WHEEL_RADIUS + BODY_HEIGHT / 2;
    createRobot(0, 0, robot_height);

    // LiDARセンサーを作成
    //   -180度〜+180度、360本の光線、最大距離5m
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 5.0, 0.1);

    // ビューワーの作成
    Viewer viewer(argc, argv, "19: Robot with LiDAR - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    Camera& cam = Viewer::getCamera();
    cam.distance = 6.0;
    cam.pitch = 50.0;

    viewer.start();

    // クリーンアップ
    delete lidar;
    delete diff_drive;
    for (auto& obs : obstacles) {
        dBodyDestroy(obs.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
