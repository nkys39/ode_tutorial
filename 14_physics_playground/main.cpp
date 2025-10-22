// ===================================================================
// 14: Physics Playground - 物理遊び場
// ===================================================================
// 【注意】このチュートリアルは簡易実装です。
// 完全な物理遊び場実装は将来追加予定です。
//
// 物理遊び場とは？
//   ユーザーが自由にオブジェクトを配置・操作できる対話的な環境です
//   - リアルタイムでオブジェクトを追加・削除
//   - マウスやキーボードでオブジェクトを操作
//   - 様々な形状や素材のオブジェクト
//   - 力や衝撃を加えられる
//
// 実装要素：
//   - キーボード/マウス入力処理
//   - オブジェクト動的生成・削除
//   - ドラッグ＆ドロップ
//   - 力の可視化
//   - パラメーター調整UI
//
// 用途：
//   - 物理エンジンのデモ
//   - ゲームプロトタイピング
//   - 教育用ツール
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

// グローバル変数
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

std::vector<dBodyID> bodies;
std::vector<dGeomID> geoms;

// 衝突コールバック関数
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.5;
        contact[i].surface.bounce = 0.3;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// 簡易デモオブジェクトを作成
void create() {
    // 現在は単純な箱のみ（プレースホルダー）
    dGeomID geom;
    dBodyID body = createBox(world, space, 0, 0, 2.0, 0.5, 0.5, 0.5, 1.0, &geom);
    bodies.push_back(body);
    geoms.push_back(geom);
}

// シミュレーションステップ
void simulationStep(double dt) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

// 描画コールバック
void drawScene() {
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    for (size_t i = 0; i < bodies.size(); i++) {
        const dReal* pos = dBodyGetPosition(bodies[i]);
        const dReal* R = dBodyGetRotation(bodies[i]);
        dReal sides[3] = {0.5, 0.5, 0.5};
        Viewer::setColor(0.7f, 0.4f, 0.3f);
        Viewer::drawBox(pos, R, sides);
    }
}

// メイン関数
int main(int argc, char** argv) {
    std::cout << "=== 14: Physics Playground（物理遊び場） ===" << std::endl;
    std::cout << "【注意】簡易実装です。完全実装は将来追加予定。" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    create();

    Viewer viewer(argc, argv, "14: Physics Playground - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    for (auto body : bodies) dBodyDestroy(body);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
