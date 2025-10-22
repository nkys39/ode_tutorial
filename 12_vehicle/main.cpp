// ===================================================================
// 12: Vehicle - 車両シミュレーション（完全実装）
// ===================================================================
// 完全実装：車体、4輪、サスペンション、ステアリングを含む車両
//
// 車両シミュレーションとは？
//   車体、4輪、サスペンションを物理的に再現します
//   - 車体：箱型の剛体
//   - 4輪：円柱（Hinge2ジョイント）
//   - サスペンション：スプリングとダンパー
//   - ステアリング：前輪の角度制御
//
// 用途：
//   - レーシングゲーム
//   - 運転シミュレーター
//   - 自動運転研究
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

// グローバル変数
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// 車両の構成要素
dBodyID chassis;                   // 車体
dBodyID wheels[4];                 // 車輪（0:左前, 1:右前, 2:左後, 3:右後）
dJointID joints[4];                // Hinge2ジョイント（サスペンション付き）

// 車両のパラメーター
const dReal CHASSIS_LENGTH = 2.0;  // 車体長さ
const dReal CHASSIS_WIDTH = 1.0;   // 車体幅
const dReal CHASSIS_HEIGHT = 0.5;  // 車体高さ
const dReal CHASSIS_MASS = 800.0;  // 車体質量：800kg

const dReal WHEEL_RADIUS = 0.35;   // 車輪半径：35cm
const dReal WHEEL_WIDTH = 0.2;     // 車輪幅：20cm
const dReal WHEEL_MASS = 20.0;     // 車輪質量：20kg/個

const dReal STEER_LIMIT = 0.5;     // ステアリング最大角度（ラジアン、約30度）

// 制御入力
dReal steer = 0.0;                 // ステアリング角度
dReal speed = 0.0;                 // 駆動速度
dReal brake = 0.0;                 // ブレーキ

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
        contact[i].surface.mu = 1.5;        // タイヤの摩擦係数（高め）
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// 車両を作成
void createVehicle() {
    // ===================================================================
    // 1. 車体（chassis）を作成
    // ===================================================================
    dGeomID chassis_geom;
    chassis = createBox(world, space, 0, 0, 1.0,
                       CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT,
                       CHASSIS_MASS, &chassis_geom);

    // ===================================================================
    // 2. 車輪を作成
    // ===================================================================
    // 車輪の位置（車体からのオフセット）
    dReal wheel_x_offset = CHASSIS_LENGTH / 2 - 0.4;  // 前後の位置
    dReal wheel_y_offset = CHASSIS_WIDTH / 2 + 0.1;    // 左右の位置
    dReal wheel_z = 0.5;                                // 高さ

    // 車輪の位置配列
    dReal wheel_positions[4][3] = {
        {-wheel_x_offset, -wheel_y_offset, wheel_z},  // 0: 左前
        {-wheel_x_offset,  wheel_x_offset, wheel_z},  // 1: 右前
        { wheel_x_offset, -wheel_y_offset, wheel_z},  // 2: 左後
        { wheel_x_offset,  wheel_y_offset, wheel_z}   // 3: 右後
    };

    for (int i = 0; i < 4; i++) {
        dGeomID wheel_geom;
        wheels[i] = createCylinder(world, space,
                                  wheel_positions[i][0], wheel_positions[i][1], wheel_positions[i][2],
                                  WHEEL_RADIUS, WHEEL_WIDTH, WHEEL_MASS, &wheel_geom);

        // 車輪を横向きに回転（Y軸周りに90度）
        dMatrix3 R;
        dRFromAxisAndAngle(R, 0, 1, 0, M_PI / 2);
        dBodySetRotation(wheels[i], R);
    }

    // ===================================================================
    // 3. Hinge2ジョイントでサスペンションを作成
    // ===================================================================
    // Hinge2ジョイントとは？
    //   2つの回転軸を持つジョイントです
    //   Axis1: ステアリング軸（垂直、Z軸）
    //   Axis2: 車輪回転軸（水平、Y軸）
    //   サスペンション効果も持たせられます

    for (int i = 0; i < 4; i++) {
        joints[i] = dJointCreateHinge2(world, 0);
        dJointAttach(joints[i], chassis, wheels[i]);

        // ジョイント位置を設定
        const dReal* pos = dBodyGetPosition(wheels[i]);
        dJointSetHinge2Anchor(joints[i], pos[0], pos[1], pos[2]);

        // Axis1: ステアリング軸（Z軸）
        dJointSetHinge2Axis1(joints[i], 0, 0, 1);
        // Axis2: 車輪回転軸（Y軸）
        dJointSetHinge2Axis2(joints[i], 0, 1, 0);

        // ===================================================================
        // サスペンションのパラメーター設定
        // ===================================================================
        // サスペンションとは？
        //   車輪と車体を柔軟に接続し、路面の凸凹を吸収する機構です
        //   スプリング（バネ）とダンパー（減衰器）で構成されます

        // ERP (Error Reduction Parameter): スプリングの強さ
        //   値が大きいほど硬いスプリング（0.2〜0.8が一般的）
        dJointSetHinge2Param(joints[i], dParamSuspensionERP, 0.4);

        // CFM (Constraint Force Mixing): ダンパーの強さ
        //   値が大きいほど柔らかい（0.01〜0.8が一般的）
        dJointSetHinge2Param(joints[i], dParamSuspensionCFM, 0.3);

        // ステアリング軸の可動範囲
        // 前輪（0, 1）のみステアリング可能、後輪（2, 3）は固定
        if (i < 2) {
            // 前輪：ステアリング可能（±30度程度）
            dJointSetHinge2Param(joints[i], dParamLoStop, -STEER_LIMIT);
            dJointSetHinge2Param(joints[i], dParamHiStop, STEER_LIMIT);
        } else {
            // 後輪：ステアリング不可（0度で固定）
            dJointSetHinge2Param(joints[i], dParamLoStop, 0);
            dJointSetHinge2Param(joints[i], dParamHiStop, 0);
        }
    }

    std::cout << "車両作成完了！" << std::endl;
    std::cout << "  車体質量: " << CHASSIS_MASS << "kg" << std::endl;
    std::cout << "  車輪: 4個 × " << WHEEL_MASS << "kg = " << WHEEL_MASS * 4 << "kg" << std::endl;
    std::cout << "  合計質量: " << CHASSIS_MASS + WHEEL_MASS * 4 << "kg" << std::endl;
}

// シミュレーションステップ
void simulationStep(double dt) {
    // ===================================================================
    // 1. ステアリング制御（前輪のみ）
    // ===================================================================
    // Hinge2ジョイントのAxis1（ステアリング軸）の角度を制御
    for (int i = 0; i < 2; i++) {  // 前輪のみ（0, 1）
        dReal current_angle = dJointGetHinge2Angle1(joints[i]);
        dReal angle_error = steer - current_angle;

        // 簡易PD制御でステアリング角度を目標値に近づける
        dReal steer_velocity = angle_error * 5.0;  // 比例ゲイン
        dJointSetHinge2Param(joints[i], dParamVel, steer_velocity);
        dJointSetHinge2Param(joints[i], dParamFMax, 50.0);  // 最大トルク
    }

    // ===================================================================
    // 2. 駆動制御（後輪駆動）
    // ===================================================================
    // Hinge2ジョイントのAxis2（車輪回転軸）の速度を制御
    for (int i = 2; i < 4; i++) {  // 後輪のみ（2, 3）
        dReal wheel_velocity = speed / WHEEL_RADIUS;  // 線速度 → 角速度

        dJointSetHinge2Param(joints[i], dParamVel2, wheel_velocity);
        dJointSetHinge2Param(joints[i], dParamFMax2, 200.0);  // 駆動トルク
    }

    // ===================================================================
    // 3. ブレーキ制御（全輪）
    // ===================================================================
    if (brake > 0.01) {
        for (int i = 0; i < 4; i++) {
            // ブレーキは車輪の回転を止める方向に力を加える
            dJointSetHinge2Param(joints[i], dParamVel2, 0.0);
            dJointSetHinge2Param(joints[i], dParamFMax2, brake * 500.0);
        }
    }

    // 物理シミュレーション
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

// 描画コールバック
void drawScene() {
    // 地面を描画
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.4f, 0.6f, 0.4f);  // 緑の地面
    Viewer::drawPlane(plane, plane[3], 20.0);

    // 車体を描画（赤色）
    const dReal* chassis_pos = dBodyGetPosition(chassis);
    const dReal* chassis_R = dBodyGetRotation(chassis);
    dReal chassis_sides[3] = {CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT};
    Viewer::setColor(0.8f, 0.2f, 0.2f);
    Viewer::drawBox(chassis_pos, chassis_R, chassis_sides);

    // 車輪を描画（黒色）
    for (int i = 0; i < 4; i++) {
        const dReal* wheel_pos = dBodyGetPosition(wheels[i]);
        const dReal* wheel_R = dBodyGetRotation(wheels[i]);
        Viewer::setColor(0.2f, 0.2f, 0.2f);
        Viewer::drawCylinder(wheel_pos, wheel_R, WHEEL_WIDTH, WHEEL_RADIUS);
    }
}

// キーボードコールバック
void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w':
        case 'W':
            // 前進
            speed = 10.0;  // 10 m/s
            brake = 0.0;
            std::cout << "前進: " << speed << " m/s" << std::endl;
            break;

        case 's':
        case 'S':
            // 後退
            speed = -5.0;  // -5 m/s
            brake = 0.0;
            std::cout << "後退: " << speed << " m/s" << std::endl;
            break;

        case 'a':
        case 'A':
            // 左旋回
            steer += 0.1;
            if (steer > STEER_LIMIT) steer = STEER_LIMIT;
            std::cout << "左旋回: " << radToDeg(steer) << "度" << std::endl;
            break;

        case 'd':
        case 'D':
            // 右旋回
            steer -= 0.1;
            if (steer < -STEER_LIMIT) steer = -STEER_LIMIT;
            std::cout << "右旋回: " << radToDeg(steer) << "度" << std::endl;
            break;

        case 'x':
        case 'X':
            // ブレーキ
            speed = 0.0;
            brake = 1.0;
            std::cout << "ブレーキ" << std::endl;
            break;

        case ' ':
            // ニュートラル（ステアリングを中央に戻す）
            steer = 0.0;
            std::cout << "ステアリング中央" << std::endl;
            break;
    }
}

// メイン関数
int main(int argc, char** argv) {
    std::cout << "=== 12: Vehicle（車両シミュレーション）- 完全実装 ===" << std::endl;
    std::cout << "4輪車両（後輪駆動、前輪ステアリング）" << std::endl;
    std::cout << "Hinge2ジョイントでサスペンション実装" << std::endl;
    std::cout << std::endl;
    std::cout << "操作方法:" << std::endl;
    std::cout << "  W: 前進" << std::endl;
    std::cout << "  S: 後退" << std::endl;
    std::cout << "  A: 左旋回" << std::endl;
    std::cout << "  D: 右旋回" << std::endl;
    std::cout << "  X: ブレーキ" << std::endl;
    std::cout << "  Space: ステアリング中央" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    createVehicle();

    Viewer viewer(argc, argv, "12: Vehicle (Full) - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);
    viewer.start();

    // クリーンアップ
    for (int i = 0; i < 4; i++) {
        dJointDestroy(joints[i]);
        dBodyDestroy(wheels[i]);
    }
    dBodyDestroy(chassis);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
