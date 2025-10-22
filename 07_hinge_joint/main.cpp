// ===================================================================
// 07: Hinge Joint - ヒンジジョイント（蝶番）
// ===================================================================
// このチュートリアルでは、ヒンジジョイントの基本を学びます。
// ドアの蝶番や振り子のように、1つの軸周りにのみ回転する関節です。
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
// ヒンジジョイントのデモ：固定台と揺れる板
// ===================================================================
// 構成：
//   fixed_body: 固定された台（動かない）
//   hinge_body: 揺れる板（ヒンジで接続、重力で振り子のように動く）
//   hinge_joint: 2つを接続するヒンジジョイント
//
// 実例：
//   - ドアの蝶番（ドア = hinge_body, ドア枠 = fixed_body）
//   - 振り子（おもり = hinge_body, 支点 = fixed_body）
dBodyID fixed_body, hinge_body;  // 固定台と揺れる板
dGeomID fixed_geom, hinge_geom;  // それぞれのジオメトリ
dJointID hinge_joint;             // ヒンジジョイント

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
        contact[i].surface.mu = 0.5;          // 摩擦係数
        contact[i].surface.bounce = 0.1;      // 低反発
        contact[i].surface.soft_cfm = 0.01;   // ソフト接触

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// ヒンジジョイント作成関数
// ===================================================================
void createHinge() {
    // ===================================================================
    // 1. 固定台を作成
    // ===================================================================
    // 位置：(0, 0, 1.0) = 地面から1m上
    // サイズ：0.3 × 0.3 × 0.3 の立方体
    // 質量：10kg（重め）
    fixed_body = createBox(world, space, 0, 0, 1.0, 0.3, 0.3, 0.3, 10.0, &fixed_geom);

    // dBodySetKinematic(): キネマティックボディに設定
    //
    // キネマティックボディとは？
    //   物理演算の影響を受けず、位置が固定される特殊な剛体です
    //   他のオブジェクトと衝突しますが、自身は動きません
    //
    // 通常の剛体との違い：
    //   通常の剛体：重力や力の影響を受けて動く
    //   キネマティック：完全に固定される（無限の質量のようなもの）
    //
    // 用途：
    //   - 固定された壁や柱
    //   - ジョイントの固定点
    //   - スクリプトで動かすプラットフォーム
    dBodySetKinematic(fixed_body);

    // ===================================================================
    // 2. 揺れる板を作成
    // ===================================================================
    // 位置：(0, 0.5, 1.0) = 固定台から Y方向に 0.5m オフセット
    // サイズ：0.2 × 1.0 × 0.1 の薄い板（Y方向に長い）
    // 質量：1kg
    hinge_body = createBox(world, space, 0, 0.5, 1.0, 0.2, 1.0, 0.1, 1.0, &hinge_geom);

    // ===================================================================
    // 3. ヒンジジョイントを作成して接続
    // ===================================================================
    // ヒンジジョイントとは？
    //   2つの剛体を1つの軸周りに回転できるように接続するジョイントです
    //   ドアの蝶番や車輪の軸と同じ仕組みです
    //
    // 自由度：1自由度（1つの軸周りの回転のみ）
    hinge_joint = dJointCreateHinge(world, 0);

    // dJointAttach(): ジョイントを2つの剛体に取り付ける
    // 引数：ジョイント, 剛体1, 剛体2
    dJointAttach(hinge_joint, fixed_body, hinge_body);

    // dJointSetHingeAnchor(): ヒンジの中心位置（回転軸の位置）を設定
    // 位置：(0, 0, 1.0) = 固定台の中心
    // この点が回転の中心となります
    dJointSetHingeAnchor(hinge_joint, 0, 0, 1.0);

    // dJointSetHingeAxis(): ヒンジの回転軸を設定
    // 軸：(1, 0, 0) = X軸
    // この軸周りに回転します
    //
    // 結果：
    //   板はX軸周りに回転 = 前後に揺れる（Y-Z平面内で回転）
    //   重力により振り子のように動きます
    dJointSetHingeAxis(hinge_joint, 1, 0, 0);
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

    // 固定台を描画（グレー）
    const dReal* pos1 = dBodyGetPosition(fixed_body);
    const dReal* R1 = dBodyGetRotation(fixed_body);
    dReal sides1[3] = {0.3, 0.3, 0.3};
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawBox(pos1, R1, sides1);

    // 揺れる板を描画（紫色）
    const dReal* pos2 = dBodyGetPosition(hinge_body);
    const dReal* R2 = dBodyGetRotation(hinge_body);
    dReal sides2[3] = {0.2, 1.0, 0.1};
    Viewer::setColor(0.7f, 0.3f, 0.9f);
    Viewer::drawBox(pos2, R2, sides2);
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 07: Hinge Joint ===" << std::endl;
    std::cout << "ヒンジジョイントで揺れるドア/振り子を実現" << std::endl;
    std::cout << "重力により板が振り子のように揺れます" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // ヒンジジョイントシステムを作成
    createHinge();

    // ビューワーの作成
    Viewer viewer(argc, argv, "07: Hinge Joint - ODE Tutorial");
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
