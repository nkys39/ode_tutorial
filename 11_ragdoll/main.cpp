// ===================================================================
// 11: Ragdoll - ラグドール（人形物理）
// ===================================================================
// 完全実装：人体を複数の剛体とジョイントで表現した物理モデル
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

// ラグドールの各パーツ
struct RagdollPart {
    dBodyID body;
    dGeomID geom;
};

// ラグドール全体の構造
struct Ragdoll {
    RagdollPart head;           // 頭
    RagdollPart chest;          // 胸部
    RagdollPart abdomen;        // 腹部
    RagdollPart upper_arm_l;    // 左上腕
    RagdollPart upper_arm_r;    // 右上腕
    RagdollPart lower_arm_l;    // 左前腕
    RagdollPart lower_arm_r;    // 右前腕
    RagdollPart upper_leg_l;    // 左太もも
    RagdollPart upper_leg_r;    // 右太もも
    RagdollPart lower_leg_l;    // 左すね
    RagdollPart lower_leg_r;    // 右すね

    std::vector<dJointID> joints;  // 全関節のリスト
};

Ragdoll ragdoll;

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
        contact[i].surface.mu = 0.8;        // 摩擦係数（人体）
        contact[i].surface.bounce = 0.1;    // 反発係数（低め）
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// カプセル（capsule）を作成するヘルパー関数
RagdollPart createRagdollPart(dReal x, dReal y, dReal z,
                               dReal length, dReal radius, dReal mass) {
    RagdollPart part;

    // 剛体を作成
    part.body = dBodyCreate(world);
    dBodySetPosition(part.body, x, y, z);

    // 質量を設定
    dMass m;
    dMassSetCapsule(&m, 1.0, 3, radius, length);  // 3 = Z軸方向
    dMassAdjust(&m, mass);
    dBodySetMass(part.body, &m);

    // ジオメトリを作成
    part.geom = dCreateCapsule(space, radius, length);
    dGeomSetBody(part.geom, part.body);

    return part;
}

// ボールジョイントを作成（肩、股関節など）
dJointID createBallJoint(dBodyID body1, dBodyID body2, dReal x, dReal y, dReal z) {
    dJointID joint = dJointCreateBall(world, 0);
    dJointAttach(joint, body1, body2);
    dJointSetBallAnchor(joint, x, y, z);
    return joint;
}

// ヒンジジョイントを作成（肘、膝など）
dJointID createHingeJoint(dBodyID body1, dBodyID body2,
                          dReal x, dReal y, dReal z,
                          dReal axis_x, dReal axis_y, dReal axis_z,
                          dReal lo_stop, dReal hi_stop) {
    dJointID joint = dJointCreateHinge(world, 0);
    dJointAttach(joint, body1, body2);
    dJointSetHingeAnchor(joint, x, y, z);
    dJointSetHingeAxis(joint, axis_x, axis_y, axis_z);

    // 関節の可動範囲を制限
    dJointSetHingeParam(joint, dParamLoStop, lo_stop);
    dJointSetHingeParam(joint, dParamHiStop, hi_stop);

    return joint;
}

// ラグドールを作成
void createRagdoll(dReal x, dReal y, dReal z) {
    // 人体のおおよそのサイズ（メートル単位）
    const dReal head_radius = 0.12;
    const dReal head_length = 0.15;

    const dReal chest_radius = 0.15;
    const dReal chest_length = 0.35;

    const dReal abdomen_radius = 0.13;
    const dReal abdomen_length = 0.25;

    const dReal upper_arm_radius = 0.05;
    const dReal upper_arm_length = 0.28;

    const dReal lower_arm_radius = 0.04;
    const dReal lower_arm_length = 0.25;

    const dReal upper_leg_radius = 0.07;
    const dReal upper_leg_length = 0.45;

    const dReal lower_leg_radius = 0.06;
    const dReal lower_leg_length = 0.43;

    // 各パーツを作成（立った状態で配置）
    dReal current_z = z;

    // 脚（下から積み上げる）
    current_z += lower_leg_length / 2;
    ragdoll.lower_leg_l = createRagdollPart(x - 0.1, y, current_z, lower_leg_length, lower_leg_radius, 4.0);
    ragdoll.lower_leg_r = createRagdollPart(x + 0.1, y, current_z, lower_leg_length, lower_leg_radius, 4.0);

    current_z += lower_leg_length / 2 + upper_leg_length / 2;
    ragdoll.upper_leg_l = createRagdollPart(x - 0.1, y, current_z, upper_leg_length, upper_leg_radius, 7.0);
    ragdoll.upper_leg_r = createRagdollPart(x + 0.1, y, current_z, upper_leg_length, upper_leg_radius, 7.0);

    // 腹部
    current_z += upper_leg_length / 2 + abdomen_length / 2;
    ragdoll.abdomen = createRagdollPart(x, y, current_z, abdomen_length, abdomen_radius, 10.0);

    // 胸部
    current_z += abdomen_length / 2 + chest_length / 2;
    ragdoll.chest = createRagdollPart(x, y, current_z, chest_length, chest_radius, 15.0);

    // 頭
    current_z += chest_length / 2 + head_length / 2;
    ragdoll.head = createRagdollPart(x, y, current_z, head_length, head_radius, 5.0);

    // 腕（胸部の高さから）
    dReal arm_z = z + lower_leg_length + upper_leg_length + abdomen_length + chest_length * 0.7;

    ragdoll.upper_arm_l = createRagdollPart(x - (chest_radius + upper_arm_length / 2), y, arm_z,
                                             upper_arm_length, upper_arm_radius, 2.5);
    ragdoll.upper_arm_r = createRagdollPart(x + (chest_radius + upper_arm_length / 2), y, arm_z,
                                             upper_arm_length, upper_arm_radius, 2.5);

    // 腕を横向きに回転
    dMatrix3 R_arm;
    dRFromAxisAndAngle(R_arm, 0, 1, 0, M_PI / 2);  // Y軸周りに90度回転
    dBodySetRotation(ragdoll.upper_arm_l.body, R_arm);
    dBodySetRotation(ragdoll.upper_arm_r.body, R_arm);

    ragdoll.lower_arm_l = createRagdollPart(x - (chest_radius + upper_arm_length + lower_arm_length / 2), y, arm_z,
                                             lower_arm_length, lower_arm_radius, 1.5);
    ragdoll.lower_arm_r = createRagdollPart(x + (chest_radius + upper_arm_length + lower_arm_length / 2), y, arm_z,
                                             lower_arm_length, lower_arm_radius, 1.5);

    dBodySetRotation(ragdoll.lower_arm_l.body, R_arm);
    dBodySetRotation(ragdoll.lower_arm_r.body, R_arm);

    // ===================================================================
    // ジョイントを作成（身体を接続）
    // ===================================================================

    // 膝（ヒンジジョイント：1軸回転のみ）
    dReal knee_z = z + lower_leg_length;
    ragdoll.joints.push_back(
        createHingeJoint(ragdoll.upper_leg_l.body, ragdoll.lower_leg_l.body,
                        x - 0.1, y, knee_z, 1, 0, 0, 0, M_PI * 0.7)
    );
    ragdoll.joints.push_back(
        createHingeJoint(ragdoll.upper_leg_r.body, ragdoll.lower_leg_r.body,
                        x + 0.1, y, knee_z, 1, 0, 0, 0, M_PI * 0.7)
    );

    // 股関節（ボールジョイント：3軸回転）
    dReal hip_z = z + lower_leg_length + upper_leg_length;
    ragdoll.joints.push_back(
        createBallJoint(ragdoll.abdomen.body, ragdoll.upper_leg_l.body, x - 0.1, y, hip_z)
    );
    ragdoll.joints.push_back(
        createBallJoint(ragdoll.abdomen.body, ragdoll.upper_leg_r.body, x + 0.1, y, hip_z)
    );

    // 腰（ボールジョイント）
    dReal waist_z = z + lower_leg_length + upper_leg_length + abdomen_length / 2;
    ragdoll.joints.push_back(
        createBallJoint(ragdoll.chest.body, ragdoll.abdomen.body, x, y, waist_z + abdomen_length / 2)
    );

    // 首（ボールジョイント）
    dReal neck_z = z + lower_leg_length + upper_leg_length + abdomen_length + chest_length;
    ragdoll.joints.push_back(
        createBallJoint(ragdoll.head.body, ragdoll.chest.body, x, y, neck_z)
    );

    // 肩（ボールジョイント）
    dReal shoulder_z = arm_z;
    ragdoll.joints.push_back(
        createBallJoint(ragdoll.chest.body, ragdoll.upper_arm_l.body, x - chest_radius, y, shoulder_z)
    );
    ragdoll.joints.push_back(
        createBallJoint(ragdoll.chest.body, ragdoll.upper_arm_r.body, x + chest_radius, y, shoulder_z)
    );

    // 肘（ヒンジジョイント）
    ragdoll.joints.push_back(
        createHingeJoint(ragdoll.upper_arm_l.body, ragdoll.lower_arm_l.body,
                        x - (chest_radius + upper_arm_length), y, arm_z,
                        0, 1, 0, 0, M_PI * 0.8)
    );
    ragdoll.joints.push_back(
        createHingeJoint(ragdoll.upper_arm_r.body, ragdoll.lower_arm_r.body,
                        x + (chest_radius + upper_arm_length), y, arm_z,
                        0, 1, 0, 0, M_PI * 0.8)
    );

    std::cout << "ラグドール作成完了: " << ragdoll.joints.size() << "個のジョイント" << std::endl;
}

// シミュレーションステップ
void simulationStep(double dt) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

// ラグドールパーツを描画するヘルパー関数
void drawRagdollPart(const RagdollPart& part, float r, float g, float b) {
    const dReal* pos = dBodyGetPosition(part.body);
    const dReal* R = dBodyGetRotation(part.body);
    dReal radius, length;
    dGeomCapsuleGetParams(part.geom, &radius, &length);

    Viewer::setColor(r, g, b);
    Viewer::drawCapsule(pos, R, length, radius);
}

// 描画コールバック
void drawScene() {
    // 地面を描画
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // ラグドールを描画（パーツごとに色分け）
    drawRagdollPart(ragdoll.head, 0.9f, 0.7f, 0.6f);          // 肌色（頭）
    drawRagdollPart(ragdoll.chest, 0.3f, 0.5f, 0.8f);         // 青（胸）
    drawRagdollPart(ragdoll.abdomen, 0.3f, 0.5f, 0.8f);       // 青（腹）
    drawRagdollPart(ragdoll.upper_arm_l, 0.9f, 0.7f, 0.6f);   // 肌色（左上腕）
    drawRagdollPart(ragdoll.upper_arm_r, 0.9f, 0.7f, 0.6f);   // 肌色（右上腕）
    drawRagdollPart(ragdoll.lower_arm_l, 0.9f, 0.7f, 0.6f);   // 肌色（左前腕）
    drawRagdollPart(ragdoll.lower_arm_r, 0.9f, 0.7f, 0.6f);   // 肌色（右前腕）
    drawRagdollPart(ragdoll.upper_leg_l, 0.2f, 0.3f, 0.6f);   // 濃い青（左太もも）
    drawRagdollPart(ragdoll.upper_leg_r, 0.2f, 0.3f, 0.6f);   // 濃い青（右太もも）
    drawRagdollPart(ragdoll.lower_leg_l, 0.9f, 0.7f, 0.6f);   // 肌色（左すね）
    drawRagdollPart(ragdoll.lower_leg_r, 0.9f, 0.7f, 0.6f);   // 肌色（右すね）
}

// メイン関数
int main(int argc, char** argv) {
    std::cout << "=== 11: Ragdoll（ラグドール）- 完全実装 ===" << std::endl;
    std::cout << "人体を11個の剛体と10個のジョイントで表現" << std::endl;
    std::cout << "ボールジョイント（肩、股関節など）とヒンジジョイント（肘、膝）を使用" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // ラグドールを作成（地面から少し浮いた位置に立たせる）
    createRagdoll(0, 0, 0.5);

    Viewer viewer(argc, argv, "11: Ragdoll (Full) - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    // クリーンアップ
    for (auto joint : ragdoll.joints) {
        dJointDestroy(joint);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
