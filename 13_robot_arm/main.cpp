// ===================================================================
// 13: Robot Arm - ロボットアーム（完全実装）
// ===================================================================
// 完全実装：3自由度の多関節ロボットアーム
//
// ロボットアームとは？
//   複数のリンクとジョイントで構成される多関節ロボットです
//   - ベース：固定台
//   - リンク：腕のセグメント（剛体）
//   - ジョイント：ヒンジ（モーター付き）
//   - エンドエフェクター：先端の作業部
//
// 用途：
//   - 産業用ロボット
//   - 手術支援ロボット
//   - サービスロボット
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include <iostream>
#include <cmath>

using namespace ode_tutorial;

// グローバル変数
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// ロボットアームの構成要素
dBodyID base;               // ベース（固定）
dBodyID link1, link2, link3; // 3つのリンク
dJointID joint1, joint2, joint3; // 3つのヒンジジョイント

// リンクのサイズ
const dReal LINK1_LENGTH = 0.8;
const dReal LINK2_LENGTH = 0.6;
const dReal LINK3_LENGTH = 0.4;
const dReal LINK_RADIUS = 0.05;

// 関節角度（制御用）
dReal target_angle1 = 0.0;  // 第1関節の目標角度
dReal target_angle2 = 0.0;  // 第2関節の目標角度
dReal target_angle3 = 0.0;  // 第3関節の目標角度

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

// ロボットアームを作成
void createRobotArm() {
    // ===================================================================
    // 1. ベース（固定台）を作成
    // ===================================================================
    // Kinematicボディ：物理演算の影響を受けないが、衝突判定は行う
    dGeomID base_geom;
    base = createBox(world, space, 0, 0, 0.2,
                    0.3, 0.3, 0.4, 10.0, &base_geom);
    dBodySetKinematic(base);  // 固定（動かない）

    // ===================================================================
    // 2. リンク1（ベースから上に伸びる）
    // ===================================================================
    dGeomID link1_geom;
    link1 = createCapsule(world, space,
                         0, 0, 0.4 + LINK1_LENGTH / 2,
                         LINK1_LENGTH, LINK_RADIUS, 1.0, &link1_geom);

    // ===================================================================
    // 3. リンク2（リンク1の先端から伸びる）
    // ===================================================================
    dGeomID link2_geom;
    link2 = createCapsule(world, space,
                         0, 0, 0.4 + LINK1_LENGTH + LINK2_LENGTH / 2,
                         LINK2_LENGTH, LINK_RADIUS, 0.8, &link2_geom);

    // ===================================================================
    // 4. リンク3（リンク2の先端から伸びる）
    // ===================================================================
    dGeomID link3_geom;
    link3 = createCapsule(world, space,
                         0, 0, 0.4 + LINK1_LENGTH + LINK2_LENGTH + LINK3_LENGTH / 2,
                         LINK3_LENGTH, LINK_RADIUS, 0.5, &link3_geom);

    // ===================================================================
    // 5. ジョイント1（ベース ← → リンク1）
    // ===================================================================
    // ヒンジジョイント：1軸の回転を許可
    joint1 = dJointCreateHinge(world, 0);
    dJointAttach(joint1, base, link1);
    dJointSetHingeAnchor(joint1, 0, 0, 0.4);  // ベース上端
    dJointSetHingeAxis(joint1, 0, 0, 1);      // Z軸周りの回転（ヨー）

    // 可動範囲を制限（-180度〜+180度）
    dJointSetHingeParam(joint1, dParamLoStop, -M_PI);
    dJointSetHingeParam(joint1, dParamHiStop, M_PI);

    // ===================================================================
    // 6. ジョイント2（リンク1 ← → リンク2）
    // ===================================================================
    joint2 = dJointCreateHinge(world, 0);
    dJointAttach(joint2, link1, link2);
    dJointSetHingeAnchor(joint2, 0, 0, 0.4 + LINK1_LENGTH);
    dJointSetHingeAxis(joint2, 1, 0, 0);      // X軸周りの回転（ピッチ）

    // 可動範囲を制限（-90度〜+90度）
    dJointSetHingeParam(joint2, dParamLoStop, -M_PI / 2);
    dJointSetHingeParam(joint2, dParamHiStop, M_PI / 2);

    // ===================================================================
    // 7. ジョイント3（リンク2 ← → リンク3）
    // ===================================================================
    joint3 = dJointCreateHinge(world, 0);
    dJointAttach(joint3, link2, link3);
    dJointSetHingeAnchor(joint3, 0, 0, 0.4 + LINK1_LENGTH + LINK2_LENGTH);
    dJointSetHingeAxis(joint3, 1, 0, 0);      // X軸周りの回転（ピッチ）

    // 可動範囲を制限（-90度〜+90度）
    dJointSetHingeParam(joint3, dParamLoStop, -M_PI / 2);
    dJointSetHingeParam(joint3, dParamHiStop, M_PI / 2);

    std::cout << "ロボットアーム作成完了！" << std::endl;
    std::cout << "  自由度: 3（ヨー1軸、ピッチ2軸）" << std::endl;
    std::cout << "  リンク長: " << LINK1_LENGTH << "m, "
              << LINK2_LENGTH << "m, "
              << LINK3_LENGTH << "m" << std::endl;
    std::cout << "  最大到達距離: 約" << (LINK1_LENGTH + LINK2_LENGTH + LINK3_LENGTH) << "m" << std::endl;
}

// PD制御でジョイント角度を制御
void controlJoint(dJointID joint, dReal target_angle, dReal kp, dReal kd) {
    // PD制御とは？
    //   比例（P）制御と微分（D）制御を組み合わせた制御方式
    //   P: 目標値との誤差に比例した制御
    //   D: 速度に基づく減衰（振動を抑える）

    // 現在の角度と角速度を取得
    dReal current_angle = dJointGetHingeAngle(joint);
    dReal current_rate = dJointGetHingeAngleRate(joint);

    // 誤差を計算
    dReal angle_error = target_angle - current_angle;

    // PD制御の出力（目標角速度）
    dReal desired_velocity = kp * angle_error - kd * current_rate;

    // モーターに速度指令を送る
    dJointSetHingeParam(joint, dParamVel, desired_velocity);
    dJointSetHingeParam(joint, dParamFMax, 50.0);  // 最大トルク
}

// シミュレーションステップ
void simulationStep(double dt) {
    // ===================================================================
    // 各ジョイントをPD制御で目標角度に制御
    // ===================================================================
    // kp: 比例ゲイン（大きいほど速く動く、大きすぎると振動）
    // kd: 微分ゲイン（大きいほど減衰が強い）
    controlJoint(joint1, target_angle1, 5.0, 1.0);
    controlJoint(joint2, target_angle2, 5.0, 1.0);
    controlJoint(joint3, target_angle3, 5.0, 1.0);

    // 物理シミュレーション
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

// エンドエフェクター（先端）の位置を計算（順運動学）
void computeForwardKinematics() {
    // 順運動学（Forward Kinematics）とは？
    //   各関節の角度から、エンドエフェクターの位置を計算すること
    //
    // 実際のODEでは、link3の位置を直接取得できる
    const dReal* end_pos = dBodyGetPosition(link3);
    const dReal* end_vel = dBodyGetLinearVel(link3);

    std::cout << "エンドエフェクター位置: ("
              << end_pos[0] << ", " << end_pos[1] << ", " << end_pos[2] << ")" << std::endl;
    std::cout << "関節角度: ("
              << radToDeg(dJointGetHingeAngle(joint1)) << "°, "
              << radToDeg(dJointGetHingeAngle(joint2)) << "°, "
              << radToDeg(dJointGetHingeAngle(joint3)) << "°)" << std::endl;
}

// 描画コールバック
void drawScene() {
    // 地面を描画
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 5.0);

    // ベースを描画（濃い灰色）
    const dReal* base_pos = dBodyGetPosition(base);
    const dReal* base_R = dBodyGetRotation(base);
    dReal base_sides[3] = {0.3, 0.3, 0.4};
    Viewer::setColor(0.3f, 0.3f, 0.3f);
    Viewer::drawBox(base_pos, base_R, base_sides);

    // リンク1を描画（赤）
    const dReal* link1_pos = dBodyGetPosition(link1);
    const dReal* link1_R = dBodyGetRotation(link1);
    Viewer::setColor(0.8f, 0.2f, 0.2f);
    Viewer::drawCapsule(link1_pos, link1_R, LINK1_LENGTH, LINK_RADIUS);

    // リンク2を描画（緑）
    const dReal* link2_pos = dBodyGetPosition(link2);
    const dReal* link2_R = dBodyGetRotation(link2);
    Viewer::setColor(0.2f, 0.8f, 0.2f);
    Viewer::drawCapsule(link2_pos, link2_R, LINK2_LENGTH, LINK_RADIUS);

    // リンク3を描画（青）
    const dReal* link3_pos = dBodyGetPosition(link3);
    const dReal* link3_R = dBodyGetRotation(link3);
    Viewer::setColor(0.2f, 0.2f, 0.8f);
    Viewer::drawCapsule(link3_pos, link3_R, LINK3_LENGTH, LINK_RADIUS);

    // エンドエフェクター（先端）に球を描画（黄色）
    dMatrix3 R_end;
    dRSetIdentity(R_end);
    Viewer::setColor(1.0f, 1.0f, 0.0f);
    Viewer::drawSphere(link3_pos, R_end, 0.08);
}

// キーボードコールバック
void keyboardCallback(unsigned char key, int x, int y) {
    const dReal angle_step = 0.1;  // 約5.7度

    switch (key) {
        // ===================================================================
        // ジョイント1（ベース回転、Z軸周り）の制御
        // ===================================================================
        case '1':
            target_angle1 += angle_step;
            std::cout << "ジョイント1: " << radToDeg(target_angle1) << "°" << std::endl;
            break;
        case '2':
            target_angle1 -= angle_step;
            std::cout << "ジョイント1: " << radToDeg(target_angle1) << "°" << std::endl;
            break;

        // ===================================================================
        // ジョイント2（リンク1-2接続、X軸周り）の制御
        // ===================================================================
        case '3':
            target_angle2 += angle_step;
            if (target_angle2 > M_PI / 2) target_angle2 = M_PI / 2;
            std::cout << "ジョイント2: " << radToDeg(target_angle2) << "°" << std::endl;
            break;
        case '4':
            target_angle2 -= angle_step;
            if (target_angle2 < -M_PI / 2) target_angle2 = -M_PI / 2;
            std::cout << "ジョイント2: " << radToDeg(target_angle2) << "°" << std::endl;
            break;

        // ===================================================================
        // ジョイント3（リンク2-3接続、X軸周り）の制御
        // ===================================================================
        case '5':
            target_angle3 += angle_step;
            if (target_angle3 > M_PI / 2) target_angle3 = M_PI / 2;
            std::cout << "ジョイント3: " << radToDeg(target_angle3) << "°" << std::endl;
            break;
        case '6':
            target_angle3 -= angle_step;
            if (target_angle3 < -M_PI / 2) target_angle3 = -M_PI / 2;
            std::cout << "ジョイント3: " << radToDeg(target_angle3) << "°" << std::endl;
            break;

        // ===================================================================
        // リセット（全関節を0度に戻す）
        // ===================================================================
        case '0':
        case 'r':
        case 'R':
            target_angle1 = 0.0;
            target_angle2 = 0.0;
            target_angle3 = 0.0;
            std::cout << "リセット: 全関節を0度に" << std::endl;
            break;

        // ===================================================================
        // 順運動学の計算結果を表示
        // ===================================================================
        case 'f':
        case 'F':
            computeForwardKinematics();
            break;
    }
}

// メイン関数
int main(int argc, char** argv) {
    std::cout << "=== 13: Robot Arm（ロボットアーム）- 完全実装 ===" << std::endl;
    std::cout << "3自由度の多関節ロボットアーム" << std::endl;
    std::cout << "PD制御でモーター角度制御" << std::endl;
    std::cout << std::endl;
    std::cout << "操作方法:" << std::endl;
    std::cout << "  1/2: ジョイント1（ベース回転）" << std::endl;
    std::cout << "  3/4: ジョイント2（第1リンク）" << std::endl;
    std::cout << "  5/6: ジョイント3（第2リンク）" << std::endl;
    std::cout << "  0/R: リセット（全関節0度）" << std::endl;
    std::cout << "  F:   順運動学の結果を表示" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    createRobotArm();

    Viewer viewer(argc, argv, "13: Robot Arm (Full) - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);
    viewer.start();

    // クリーンアップ
    dJointDestroy(joint1);
    dJointDestroy(joint2);
    dJointDestroy(joint3);
    dBodyDestroy(base);
    dBodyDestroy(link1);
    dBodyDestroy(link2);
    dBodyDestroy(link3);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
