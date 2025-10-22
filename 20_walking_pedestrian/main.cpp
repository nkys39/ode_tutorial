// ===================================================================
// 20: Walking Pedestrian - 歩行者シミュレーション
// ===================================================================
// このチュートリアルでは、歩行者を動的障害物として扱います。
// Social Force Model（社会力モデル）で自然な動きを再現します。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "pedestrian.h"
#include <iostream>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// 歩行者管理
CrowdManager* crowd_manager;

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
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.5;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// 歩行者作成関数
// ===================================================================
void createPedestrians() {
    // 異なる目標を持つ複数の歩行者を作成
    //
    // Social Force Model（社会力モデル）とは？
    //   人間の歩行行動を物理的な力として表現するモデルです
    //
    // 主な力：
    //   1. 目標への駆動力：目標方向に歩こうとする力
    //   2. 他者回避力：他の歩行者を避けようとする力
    //   3. 障害物回避力：壁や物体を避けようとする力
    //   4. グループ凝集力：グループで歩く場合の力
    //
    // 利点：
    //   - 自然な歩行パターン
    //   - 混雑時の流動を再現
    //   - 緊急避難のシミュレーション
    //
    // 用途：
    //   - 避難計画
    //   - 建築設計（混雑解析）
    //   - ゲームのNPC（非プレイヤーキャラクター）

    // 歩行者1：左下から右上へ
    Pedestrian* p1 = crowd_manager->addPedestrian(-3, -2, 0.8);
    p1->setGoal(3, 2);
    p1->setDesiredSpeed(1.2);  // 希望速度：1.2 m/s（やや速い）

    // 歩行者2：右上から左下へ
    Pedestrian* p2 = crowd_manager->addPedestrian(3, 2, 0.8);
    p2->setGoal(-3, -2);
    p2->setDesiredSpeed(1.0);  // 希望速度：1.0 m/s（平均的）

    // 歩行者3：左上から右下へ
    Pedestrian* p3 = crowd_manager->addPedestrian(-2, 2, 0.8);
    p3->setGoal(2, -2);
    p3->setDesiredSpeed(1.1);

    // 歩行者4：右下から左上へ
    Pedestrian* p4 = crowd_manager->addPedestrian(2, -2, 0.8);
    p4->setGoal(-2, 2);
    p4->setDesiredSpeed(0.9);  // 希望速度：0.9 m/s（ゆっくり）

    std::cout << "Created " << crowd_manager->getCount() << " pedestrians" << std::endl;
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    // 歩行者の行動を更新
    //   Social Force Modelに基づいて各歩行者の速度と位置を計算
    crowd_manager->update(dt);

    // 目標に到達した歩行者をリセット
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            // 逆方向を新しい目標に設定（往復運動）
            dReal new_goal_x = -pos[0];
            dReal new_goal_y = -pos[1];
            ped->setGoal(new_goal_x, new_goal_y);
            std::cout << "Pedestrian reached goal, heading back!" << std::endl;
        }
    }

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

    // 歩行者を描画
    int ped_num = 0;
    for (auto ped : crowd_manager->getPedestrians()) {
        const dReal* pos = ped->getPosition();
        const dReal* vel = ped->getVelocity();

        dGeomID geom = ped->getGeom();
        const dReal* R = dGeomGetRotation(geom);
        dReal radius, length;
        dGeomCapsuleGetParams(geom, &radius, &length);

        // 歩行者ごとに色を変える
        float hue = (ped_num++ * 0.25f);
        Viewer::setColor(0.8f - hue * 0.3f, 0.3f + hue * 0.5f, 0.9f - hue * 0.4f);
        Viewer::drawCapsule(pos, R, length, radius);

        // 速度ベクトルを描画（黄色い線）
        //   歩行者の移動方向を可視化
        dReal vel_end[3] = {
            pos[0] + vel[0] * 0.5,  // 速度の0.5倍の長さで表示
            pos[1] + vel[1] * 0.5,
            pos[2]
        };
        Viewer::drawLine(pos, vel_end, 1.0f, 1.0f, 0.0f);
    }
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 20: Walking Pedestrian（歩行者シミュレーション） ===" << std::endl;
    std::cout << "歩行者を動的障害物としてシミュレート" << std::endl;
    std::cout << "Social Force Modelで自然な動きを再現" << std::endl;
    std::cout << std::endl;

    // ODE初期化
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    // 地面を作成
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // 歩行者管理を作成
    crowd_manager = new CrowdManager(world, space);

    // 歩行者を作成
    createPedestrians();

    // ビューワーの作成
    Viewer viewer(argc, argv, "20: Walking Pedestrian - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    Camera& cam = Viewer::getCamera();
    cam.distance = 10.0;
    cam.pitch = 60.0;

    viewer.start();

    // クリーンアップ
    delete crowd_manager;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
