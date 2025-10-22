// ===================================================================
// 16: Odometry - オドメトリ（車輪エンコーダによる自己位置推定）
// ===================================================================
// このチュートリアルでは、オドメトリの基本を学びます。
// 車輪の回転量から移動距離と姿勢を推定する手法です。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"
#include <iostream>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

dBodyID robot_body, left_wheel, right_wheel;
dJointID left_hinge, right_hinge;

// ===================================================================
// ロボットの物理パラメーター
// ===================================================================
const dReal WHEEL_RADIUS = 0.05;   // 車輪半径：5cm
const dReal WHEEL_WIDTH = 0.03;    // 車輪幅：3cm
const dReal WHEEL_BASE = 0.3;      // 車輪間隔：30cm（トレッド幅）

// ===================================================================
// 制御とセンサー
// ===================================================================
DifferentialDrive* diff_drive;  // 差動駆動制御
Odometry* odometry;              // オドメトリ（位置推定）

dReal target_linear_vel = 0.5;   // 目標直進速度：0.5 m/s
dReal target_angular_vel = 0.0;  // 目標角速度：0 rad/s

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
// ロボット作成関数
// ===================================================================
void createRobot() {
    // 1. ロボット本体（箱型）を作成
    //    位置：(0, 0, 0.2)、サイズ：40cm × 30cm × 15cm、質量：5kg
    dGeomID geom;
    robot_body = createBox(world, space, 0, 0, 0.2, 0.4, 0.3, 0.15, 5.0, &geom);

    // 2. 左右の車輪（円柱）を作成
    //    左車輪：x = -0.15（ロボット左側）
    //    右車輪：x = +0.15（ロボット右側）
    left_wheel = createCylinder(world, space, -0.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &geom);
    right_wheel = createCylinder(world, space, 0.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &geom);

    // 3. 車輪を横向き（X軸周りに90度回転）にする
    //    円柱はデフォルトでZ軸方向なので、X軸周りに回転
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);
    dBodySetRotation(right_wheel, R);

    // 4. ヒンジジョイントで車輪をロボット本体に接続
    //    左車輪用ヒンジ
    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, -0.15, 0, 0.125);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    //    右車輪用ヒンジ
    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, 0.15, 0, 0.125);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    // 5. 差動駆動制御とオドメトリを初期化
    //
    // DifferentialDrive: 目標速度 → 左右車輪速度の変換
    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);

    // Odometry: 車輪エンコーダー → ロボット位置の推定
    //
    // オドメトリとは？
    //   車輪の回転角度（エンコーダー値）を測定し、そこから
    //   ロボットの移動距離と姿勢（位置と向き）を推定する手法です
    //
    // 仕組み：
    //   1. 各車輪の回転角度を測定（ジョイント角度を使用）
    //   2. 回転角度 × 車輪半径 = 各車輪の移動距離
    //   3. 左右の移動距離差から旋回角度を計算
    //   4. 移動距離と旋回角度から新しい位置を計算
    //
    // 利点：
    //   - GPSが使えない屋内でも使える
    //   - リアルタイムで高頻度に更新できる
    //   - 安価（エンコーダーは安い）
    //
    // 欠点：
    //   - 誤差が蓄積する（累積誤差）
    //   - 車輪が滑ると大きくズレる
    //   - 長時間使うと大きくずれる → 他のセンサーで補正が必要
    odometry = new Odometry(WHEEL_BASE, WHEEL_RADIUS);
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    // ===================================================================
    // 1. 目標速度から左右の車輪速度を計算
    // ===================================================================
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel, left_vel, right_vel);

    // ===================================================================
    // 2. モーター速度を設定
    // ===================================================================
    dJointSetHingeParam(left_hinge, dParamVel, left_vel);
    dJointSetHingeParam(left_hinge, dParamFMax, 10.0);
    dJointSetHingeParam(right_hinge, dParamVel, right_vel);
    dJointSetHingeParam(right_hinge, dParamFMax, 10.0);

    // ===================================================================
    // 3. オドメトリを更新（車輪エンコーダーを読み取って位置推定）
    // ===================================================================
    // dJointGetHingeAngle(): ヒンジジョイントの現在角度を取得
    //   = エンコーダーの値に相当
    dReal left_angle = dJointGetHingeAngle(left_hinge);
    dReal right_angle = dJointGetHingeAngle(right_hinge);

    // Odometry::update(): エンコーダー値から位置を推定
    //   前回の角度と今回の角度の差分から移動量を計算し、
    //   推定位置を更新します（デッドレコニング）
    odometry->update(left_angle, right_angle);

    // ===================================================================
    // 4. 100ステップごとにオドメトリと実際の位置を比較
    // ===================================================================
    static int counter = 0;
    if (counter++ % 100 == 0) {
        // オドメトリで推定した位置と姿勢
        dReal odo_x, odo_y, odo_theta;
        odometry->getPose(odo_x, odo_y, odo_theta);

        // 実際の物理シミュレーション上の位置と姿勢
        const dReal* actual_pos = dBodyGetPosition(robot_body);
        const dReal* actual_R = dBodyGetRotation(robot_body);
        dReal actual_yaw = getYawFromRotation(actual_R);

        // 推定値と実際の値を表示
        std::cout << "Odometry: (" << odo_x << ", " << odo_y << "), θ=" << radToDeg(odo_theta) << "°" << std::endl;
        std::cout << "Actual:   (" << actual_pos[0] << ", " << actual_pos[1] << "), θ=" << radToDeg(actual_yaw) << "°" << std::endl;

        // 誤差を表示
        //   理想的な環境（車輪が滑らない）でも、数値計算の誤差で
        //   少しずつズレが生じます。実世界ではもっと大きくズレます
        std::cout << "Error: " << (actual_pos[0] - odo_x) << ", " << (actual_pos[1] - odo_y) << std::endl;
    }

    // ===================================================================
    // 5. 物理シミュレーション
    // ===================================================================
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
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

    // ロボット本体を描画（緑色）
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {0.4, 0.3, 0.15};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);
}

// ===================================================================
// キーボードコールバック関数
// ===================================================================
// キー操作でロボットを操縦します
//   W: 前進（0.5 m/s）
//   S: 後退（-0.5 m/s）
//   A: 左旋回（直進 0.3 m/s + 左回転 1.0 rad/s）
//   D: 右旋回（直進 0.3 m/s + 右回転 -1.0 rad/s）
//   X: 停止
void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w': target_linear_vel = 0.5; target_angular_vel = 0.0; break;
        case 's': target_linear_vel = -0.5; target_angular_vel = 0.0; break;
        case 'a': target_linear_vel = 0.3; target_angular_vel = 1.0; break;
        case 'd': target_linear_vel = 0.3; target_angular_vel = -1.0; break;
        case 'x': target_linear_vel = 0.0; target_angular_vel = 0.0; break;
    }
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 16: Odometry（オドメトリ） ===" << std::endl;
    std::cout << "車輪エンコーダーによる自己位置推定" << std::endl;
    std::cout << "Controls: W/A/S/D/X" << std::endl;

    // ODE初期化
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // ロボットを作成
    createRobot();

    // ビューワーの作成
    Viewer viewer(argc, argv, "16: Odometry - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);
    viewer.start();

    // クリーンアップ
    delete diff_drive;
    delete odometry;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
