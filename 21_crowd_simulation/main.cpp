// ===================================================================
// 21: Crowd Simulation - 群衆シミュレーション
// ===================================================================
// このチュートリアルでは、複数の歩行者が相互作用する群衆を扱います。
// Social Force Modelで自然な群衆行動を再現します。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "pedestrian.h"
#include <iostream>

using namespace ode_tutorial;

// グローバル変数
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// 群衆管理
CrowdManager* crowd_manager;

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
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// 群衆作成関数
void createCrowd() {
    // 円形に配置された大規模な群衆を作成
    //
    // 群衆シミュレーションとは？
    //   多数の歩行者が互いに影響し合いながら移動する様子を再現します
    //
    // 群衆現象の例：
    //   - レーンフォーメーション（自然に列ができる）
    //   - アーチング効果（出口で混雑）
    //   - 渦流（混雑時に渦ができる）
    //   - ボトルネック（狭い場所で詰まる）
    //
    // 用途：
    //   - イベント会場の設計
    //   - 緊急避難計画
    //   - 公共交通機関の設計
    //   - ゲームのモブ（大量のNPC）

    for (int i = 0; i < 8; i++) {
        // 円周上に均等に配置（円の中心に向かって移動）
        dReal angle = i * 2 * M_PI / 8;
        dReal radius = 3.0;
        dReal x = radius * std::cos(angle);
        dReal y = radius * std::sin(angle);

        Pedestrian* ped = crowd_manager->addPedestrian(x, y, 0.8);
        // 反対側に向かって移動（互いにすれ違う）
        ped->setGoal(-x * 0.8, -y * 0.8);
        // 歩行者ごとに異なる速度（0.8〜1.2 m/s）
        ped->setDesiredSpeed(0.8 + (i % 3) * 0.2);
    }
}

// シミュレーションステップ関数
void simulationStep(double dt) {
    // 群衆行動を更新
    //   全歩行者のSocial Force Modelを計算し、相互作用を考慮
    crowd_manager->update(dt);

    // 目標到達時にリセット（往復運動を継続）
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            ped->setGoal(-pos[0], -pos[1]);
        }
    }

    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

// 描画コールバック関数
void drawScene() {
    // 地面を描画
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // 歩行者を描画
    int ped_num = 0;
    for (auto ped : crowd_manager->getPedestrians()) {
        const dReal* pos = ped->getPosition();
        dGeomID geom = ped->getGeom();
        const dReal* R = dGeomGetRotation(geom);
        dReal radius, length;
        dGeomCapsuleGetParams(geom, &radius, &length);

        // 歩行者ごとに色を変える
        float hue = (ped_num++ * 0.13f);
        Viewer::setColor(0.8f - hue * 0.3f, 0.3f + hue * 0.5f, 0.9f - hue * 0.4f);
        Viewer::drawCapsule(pos, R, length, radius);
    }
}

// メイン関数
int main(int argc, char** argv) {
    std::cout << "=== 21: Crowd Simulation（群衆シミュレーション） ===" << std::endl;
    std::cout << "Social Force Modelで複数の歩行者が相互作用" << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    crowd_manager = new CrowdManager(world, space);
    createCrowd();

    Viewer viewer(argc, argv, "21: Crowd Simulation - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    delete crowd_manager;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
