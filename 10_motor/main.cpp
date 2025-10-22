// ===================================================================
// 10: Motor - モーター（動力付きジョイント）
// ===================================================================
// このチュートリアルでは、モーター機能の基本を学びます。
// ヒンジジョイントにモーターを追加して、車輪を回転させます。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// ===================================================================
// モーターのデモ：台車と回転する車輪
// ===================================================================
// 構成：
//   base_body: 台車（箱）
//   wheel_body: 車輪（円柱、モーターで回転）
//   motor_joint: ヒンジジョイント + モーター機能
//
// 実例：
//   - 車のタイヤ（エンジンで回転）
//   - 扇風機の羽根
//   - ロボットの関節（サーボモーター）
dBodyID base_body, wheel_body;  // 台車と車輪
dGeomID base_geom, wheel_geom;  // それぞれのジオメトリ
dJointID motor_joint;            // モーター付きヒンジジョイント

// ===================================================================
// 衝突コールバック関数
// ===================================================================
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    // ジョイントで接続されているオブジェクト同士は衝突判定しない
    if (b1 && b2 && dAreConnected(b1, b2)) return;

    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.8;          // 車輪用にやや高い摩擦
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// モーター作成関数
// ===================================================================
void createMotor() {
    // ===================================================================
    // 1. 台車を作成
    // ===================================================================
    // 位置：(0, 0, 0.3) = 地面から30cm上
    // サイズ：0.5 × 0.5 × 0.2
    base_body = createBox(world, space, 0, 0, 0.3, 0.5, 0.5, 0.2, 5.0, &base_geom);

    // ===================================================================
    // 2. 車輪を作成
    // ===================================================================
    // 位置：(0.3, 0, 0.3) = 台車の横
    // 半径：0.2m、幅：0.1m
    wheel_body = createCylinder(world, space, 0.3, 0, 0.3, 0.2, 0.1, 0.5, &wheel_geom);

    // 車輪を横向きにする（Y軸周りに90度回転）
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, M_PI / 2);
    dBodySetRotation(wheel_body, R);

    // ===================================================================
    // 3. ヒンジジョイントを作成（後でモーターとして使用）
    // ===================================================================
    motor_joint = dJointCreateHinge(world, 0);

    // ジョイントを台車と車輪に取り付ける
    dJointAttach(motor_joint, base_body, wheel_body);

    // ヒンジの中心位置：車輪の中心
    dJointSetHingeAnchor(motor_joint, 0.3, 0, 0.3);

    // ヒンジの回転軸：Y軸
    // 車輪はY軸周りに回転します
    dJointSetHingeAxis(motor_joint, 0, 1, 0);

    // ===================================================================
    // 4. モーターパラメータを設定
    // ===================================================================
    // ヒンジジョイントに「モーター機能」を追加します
    //
    // dParamVel: 目標角速度を設定
    //   値：10.0 rad/s（1秒間に約1.6回転）
    //   モーターはこの速度を維持しようとします
    //
    // なぜ速度制御？
    //   現実のモーターも通常は速度制御です
    //   トルクは自動的に調整されます
    dJointSetHingeParam(motor_joint, dParamVel, 10.0);

    // dParamFMax: 最大トルク（力）を設定
    //   値：5.0 Nm（ニュートンメートル）
    //   モーターが出せる最大の力です
    //
    // なぜ制限が必要？
    //   - 無限のトルクは現実的でない
    //   - シミュレーションが不安定になる
    //   - 現実のモーター性能を再現
    //
    // バランスが重要：
    //   FMax大：強力だが不安定になりがち
    //   FMax小：安定だが力不足で目標速度に達しない
    dJointSetHingeParam(motor_joint, dParamFMax, 5.0);

    // 結果：
    //   車輪が継続的に回転します
    //   摩擦や負荷があっても、モーターが速度を維持しようとします
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
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

    // 台車を描画（青色）
    const dReal* pos1 = dBodyGetPosition(base_body);
    const dReal* R1 = dBodyGetRotation(base_body);
    dReal sides[3] = {0.5, 0.5, 0.2};
    Viewer::setColor(0.3f, 0.3f, 0.7f);
    Viewer::drawBox(pos1, R1, sides);

    // 車輪を描画（赤色）
    const dReal* pos2 = dBodyGetPosition(wheel_body);
    const dReal* R2 = dBodyGetRotation(wheel_body);
    Viewer::setColor(0.7f, 0.3f, 0.3f);
    Viewer::drawCylinder(pos2, R2, 0.1, 0.2);
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 10: Motor ===" << std::endl;
    std::cout << "モーター制御で回転する車輪" << std::endl;
    std::cout << "目標速度：10 rad/s、最大トルク：5 Nm" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // モーターシステムを作成
    createMotor();

    // ビューワーの作成
    Viewer viewer(argc, argv, "10: Motor - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    // クリーンアップ
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
