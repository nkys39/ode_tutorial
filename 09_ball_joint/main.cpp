// ===================================================================
// 09: Ball Joint - ボールジョイント（球関節）
// ===================================================================
// このチュートリアルでは、ボールジョイントの基本を学びます。
// 全方向に自由に回転できる関節です（肩関節やキャスター）。
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
// ボールジョイントのデモ：固定球と揺れる棒
// ===================================================================
// 構成：
//   fixed_body: 固定された球（動かない支点）
//   ball_body: 揺れる棒（ボールジョイントで接続、全方向に動く）
//   ball_joint: 2つを接続するボールジョイント
//
// 実例：
//   - 肩関節（上腕 = ball_body、肩甲骨 = fixed_body）
//   - キャスター（車輪 = ball_body、台車 = fixed_body）
//   - ぶら下がるランプ
dBodyID fixed_body, ball_body;  // 固定球と揺れる棒
dGeomID fixed_geom, ball_geom;  // それぞれのジオメトリ
dJointID ball_joint;             // ボールジョイント

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
        contact[i].surface.mu = 0.5;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// ボールジョイント作成関数
// ===================================================================
void createBallJoint() {
    // ===================================================================
    // 1. 固定球を作成
    // ===================================================================
    // 位置：(0, 0, 2.0) = 地面から2m上
    // 半径：0.2m
    fixed_body = createSphere(world, space, 0, 0, 2.0, 0.2, 10.0, &fixed_geom);
    dBodySetKinematic(fixed_body);  // 固定（動かない支点）

    // ===================================================================
    // 2. 揺れる棒を作成
    // ===================================================================
    // 位置：(0, 0, 1.5) = 固定球の下
    // サイズ：0.3 × 0.3 × 0.8（縦長の棒）
    ball_body = createBox(world, space, 0, 0, 1.5, 0.3, 0.3, 0.8, 1.0, &ball_geom);

    // ===================================================================
    // 3. ボールジョイントを作成して接続
    // ===================================================================
    // ボールジョイント（Ball Joint）とは？
    //   2つの剛体を1点で接続し、その点を中心に全方向に回転できる
    //   ようにするジョイントです。肩関節と同じ仕組みです
    //
    // 自由度：3自由度（3つの軸周りの回転が可能）
    //   - X軸周りの回転（ピッチ）
    //   - Y軸周りの回転（ヨー）
    //   - Z軸周りの回転（ロール）
    //
    // 他のジョイントとの比較：
    //   ヒンジ：1自由度（1軸周りの回転のみ）
    //   スライダー：1自由度（1軸方向の並進のみ）
    //   ボール：3自由度（全方向の回転が可能）
    ball_joint = dJointCreateBall(world, 0);

    // ジョイントを2つの剛体に取り付ける
    dJointAttach(ball_joint, fixed_body, ball_body);

    // dJointSetBallAnchor(): ボールジョイントの中心位置を設定
    // 位置：(0, 0, 2.0) = 固定球の中心
    //
    // この点が回転の中心となります
    //   - 棒はこの点を中心に全方向に回転できます
    //   - 重力により棒が揺れ動きます
    //   - ヒンジと違い、回転方向に制限がありません
    dJointSetBallAnchor(ball_joint, 0, 0, 2.0);
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

    // 固定球を描画（グレー）
    const dReal* pos1 = dBodyGetPosition(fixed_body);
    const dReal* R1 = dBodyGetRotation(fixed_body);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawSphere(pos1, R1, 0.2);

    // 揺れる棒を描画（オレンジ色）
    const dReal* pos2 = dBodyGetPosition(ball_body);
    const dReal* R2 = dBodyGetRotation(ball_body);
    dReal sides[3] = {0.3, 0.3, 0.8};
    Viewer::setColor(0.9f, 0.4f, 0.2f);
    Viewer::drawBox(pos2, R2, sides);
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 09: Ball Joint ===" << std::endl;
    std::cout << "ボールジョイントで全方向に揺れる棒" << std::endl;
    std::cout << "肩関節のように3自由度で回転します" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // ボールジョイントシステムを作成
    createBallJoint();

    // ビューワーの作成
    Viewer viewer(argc, argv, "09: Ball Joint - ODE Tutorial");
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
