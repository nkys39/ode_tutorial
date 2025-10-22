// ===================================================================
// 11: Ragdoll - ラグドール（人形物理）
// ===================================================================
// 【注意】このチュートリアルは簡易実装です。
// 完全なラグドール実装は将来追加予定です。
//
// ラグドールとは？
//   人体を複数の剛体とジョイントで表現した物理モデルです
//   - 頭、胴体、腕、脚などのパーツを剛体で表現
//   - 関節をボールジョイントやヒンジジョイントで接続
//   - 重力や衝撃でリアルに崩れ落ちる動きを実現
//
// 用途：
//   - ゲームのキャラクター物理
//   - 転倒シミュレーション
//   - アニメーション補助
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

std::vector<dBodyID> bodies;  // 剛体のリスト
std::vector<dGeomID> geoms;   // ジオメトリのリスト

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
    std::cout << "=== 11: Ragdoll（ラグドール） ===" << std::endl;
    std::cout << "【注意】簡易実装です。完全実装は将来追加予定。" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    create();

    Viewer viewer(argc, argv, "11: Ragdoll - ODE Tutorial");
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
