// ===================================================================
// 08: Slider Joint - スライダージョイント（直動関節）
// ===================================================================
// このチュートリアルでは、スライダージョイントの基本を学びます。
// 1つの軸に沿って直線的にのみ動く関節です（ピストンやエレベーター）。
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
// スライダージョイントのデモ：固定台と上下する箱
// ===================================================================
// 構成：
//   fixed_body: 固定された台（動かない）
//   slider_body: 上下する箱（スライダーで接続、重力で落下）
//   slider_joint: 2つを接続するスライダージョイント
//
// 実例：
//   - エレベーター（箱 = slider_body、シャフト = fixed_body）
//   - ピストン（ピストンヘッド = slider_body、シリンダー = fixed_body）
//   - 引き出し
dBodyID fixed_body, slider_body;  // 固定台と上下する箱
dGeomID fixed_geom, slider_geom;  // それぞれのジオメトリ
dJointID slider_joint;             // スライダージョイント

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
// スライダージョイント作成関数
// ===================================================================
void createSlider() {
    // ===================================================================
    // 1. 固定台を作成
    // ===================================================================
    // 位置：(0, 0, 1.5) = 地面から1.5m上
    // サイズ：0.3 × 0.3 × 0.3 の立方体
    fixed_body = createBox(world, space, 0, 0, 1.5, 0.3, 0.3, 0.3, 10.0, &fixed_geom);
    dBodySetKinematic(fixed_body);  // 固定（動かない）

    // ===================================================================
    // 2. 上下する箱を作成
    // ===================================================================
    // 位置：(0, 0, 1.0) = 固定台の下
    // サイズ：0.2 × 0.2 × 0.4（縦長）
    slider_body = createBox(world, space, 0, 0, 1.0, 0.2, 0.2, 0.4, 1.0, &slider_geom);

    // ===================================================================
    // 3. スライダージョイントを作成して接続
    // ===================================================================
    // スライダージョイント（Slider Joint）とは？
    //   2つの剛体を1つの軸に沿って直線的に動けるように接続するジョイントです
    //   回転は許されず、並進（スライド）のみが可能です
    //
    // 自由度：1自由度（1つの軸に沿った並進のみ）
    //
    // ヒンジジョイントとの違い：
    //   ヒンジ：回転のみ（ドアの蝶番）
    //   スライダー：並進のみ（エレベーター、引き出し）
    slider_joint = dJointCreateSlider(world, 0);

    // ジョイントを2つの剛体に取り付ける
    dJointAttach(slider_joint, fixed_body, slider_body);

    // dJointSetSliderAxis(): スライダーの軸を設定
    // 軸：(0, 0, 1) = Z軸
    // この軸に沿ってのみ動きます
    //
    // 結果：
    //   箱はZ軸に沿って上下に動く（エレベーターのような動き）
    //   重力により下に落ちますが、X-Y方向には動けません
    dJointSetSliderAxis(slider_joint, 0, 0, 1);
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

    // 上下する箱を描画（緑色）
    const dReal* pos2 = dBodyGetPosition(slider_body);
    const dReal* R2 = dBodyGetRotation(slider_body);
    dReal sides2[3] = {0.2, 0.2, 0.4};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos2, R2, sides2);
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 08: Slider Joint ===" << std::endl;
    std::cout << "スライダージョイントで上下するエレベーター" << std::endl;
    std::cout << "箱は垂直方向（Z軸）にのみ動きます" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // スライダージョイントシステムを作成
    createSlider();

    // ビューワーの作成
    Viewer viewer(argc, argv, "08: Slider Joint - ODE Tutorial");
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
