// ===================================================================
// 15: Differential Drive Robot - 差動2輪ロボット
// ===================================================================
// このチュートリアルでは、差動2輪駆動ロボットを作成します。
// 2つの独立制御可能な車輪とキャスターを使った移動ロボットの基礎を学びます。
// キーボード（WASD）で操作できます！
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"  // DifferentialDriveクラス
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
// 差動2輪駆動ロボットとは？
// ===================================================================
// 2つの駆動輪（左右）を独立に制御することで移動するロボットです
//
// 動作原理：
//   - 両輪同速度：直進
//   - 左輪 > 右輪：右回転
//   - 左輪 < 右輪：左回転
//   - 左輪と右輪が逆方向：その場回転
//
// 構成要素：
//   - robot_body：ロボットの本体（箱）
//   - left_wheel, right_wheel：駆動輪（円柱、独立に回転）
//   - front_caster, rear_caster：キャスター（球、自由に回転）
//   - left_hinge, right_hinge：ヒンジジョイント（駆動輪を本体に接続）
//   - front/rear_caster_joint：ボールジョイント（キャスターを本体に接続）
//
// キャスターとは？
//   補助輪です。駆動しないが、ロボットを安定させます
//   低摩擦で自由に回転するため、進行方向を妨げません

// ロボットの剛体
dBodyID robot_body;                    // ロボット本体
dBodyID left_wheel, right_wheel;       // 左右の駆動輪（円柱）
dBodyID front_caster, rear_caster;     // 前後のキャスター（球）

// ジョイント（関節）
dJointID left_hinge, right_hinge;      // 駆動輪用ヒンジジョイント
dJointID front_caster_joint, rear_caster_joint;  // キャスター用ボールジョイント

// ===================================================================
// ロボットのパラメータ（寸法）
// ===================================================================
const dReal WHEEL_RADIUS = 0.05;   // 駆動輪の半径：5cm
const dReal WHEEL_WIDTH = 0.03;    // 駆動輪の幅：3cm
const dReal WHEEL_BASE = 0.3;      // 左右輪の間隔：30cm（重要！回転半径に影響）
const dReal BODY_LENGTH = 0.4;     // 本体の長さ（X方向）：40cm
const dReal BODY_WIDTH = 0.3;      // 本体の幅（Y方向）：30cm
const dReal BODY_HEIGHT = 0.15;    // 本体の高さ（Z方向）：15cm
const dReal CASTER_RADIUS = 0.03;  // キャスターの半径：3cm

// ===================================================================
// ロボット制御
// ===================================================================
// DifferentialDrive：差動2輪駆動の運動学計算を行うヘルパークラス
// 目標速度（直進速度と角速度）から各車輪の速度を計算します
DifferentialDrive* diff_drive;

// 目標速度
dReal target_linear_vel = 0.0;   // 直進速度（m/s）初期値：停止
dReal target_angular_vel = 0.0;  // 角速度（rad/s）初期値：回転なし

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

        // ===================================================================
        // 重要：駆動輪とキャスターで摩擦係数を変える
        // ===================================================================
        // なぜ？
        //   駆動輪：高摩擦が必要（滑らずに地面を蹴って前進する）
        //   キャスター：低摩擦が必要（自由に回転して方向転換を妨げない）
        //
        // 摩擦係数の設定：
        //   駆動輪：mu = 10.0（高摩擦、滑りにくい）
        //   キャスター：mu = 0.1（低摩擦、滑りやすい）
        bool is_caster = (b1 == front_caster || b1 == rear_caster ||
                         b2 == front_caster || b2 == rear_caster);
        contact[i].surface.mu = is_caster ? 0.1 : 10.0;

        contact[i].surface.bounce = 0.1;      // 低反発（跳ねない）
        contact[i].surface.soft_cfm = 0.01;   // ソフト接触

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// ロボット作成関数
// ===================================================================
// 差動2輪ロボットを構築します：本体 + 駆動輪2個 + キャスター2個
void createRobot(dReal x, dReal y, dReal z) {
    // ===================================================================
    // 1. ロボット本体の作成
    // ===================================================================
    // 引数のzは本体の中心高さです
    robot_body = createBox(world, space, x, y, z,
                          BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, 5.0);

    // ===================================================================
    // 2. 左右の駆動輪を作成
    // ===================================================================
    // 駆動輪の配置：
    //   - X座標：本体の左右（WHEEL_BASE間隔）
    //   - Y座標：本体と同じ
    //   - Z座標：本体の底面と同じ高さ
    //
    // 駆動輪の向き：
    //   createCylinder()で作成される円柱は、デフォルトでZ軸方向に伸びています
    //   車輪としては、Y軸方向（横向き）に回転する必要があります
    //   そのため、X軸周りに90度回転させます

    // 駆動輪の中心高さを計算
    // 本体の底面 = 本体中心 - 本体高さの半分
    dReal wheel_center_z = z - BODY_HEIGHT / 2;

    // 左側の駆動輪を作成
    // 位置：本体中心から左に WHEEL_BASE/2 オフセット
    left_wheel = createCylinder(world, space,
                               x - WHEEL_BASE / 2, y, wheel_center_z,
                               WHEEL_RADIUS, WHEEL_WIDTH, 0.5);

    // 回転行列の作成と設定
    // dRFromAxisAndAngle(回転行列, 軸x, 軸y, 軸z, 角度)
    // X軸(1,0,0)周りに90度(π/2)回転 → 円柱が横向きになる
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);

    // 右側の駆動輪を作成
    // 位置：本体中心から右に WHEEL_BASE/2 オフセット
    // 回転は左輪と同じ
    right_wheel = createCylinder(world, space,
                                x + WHEEL_BASE / 2, y, wheel_center_z,
                                WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dBodySetRotation(right_wheel, R);

    // ===================================================================
    // 3. ヒンジジョイントで駆動輪を本体に接続
    // ===================================================================
    // ヒンジジョイント（Hinge Joint）とは？
    //   1軸周りにのみ回転できるジョイントです（ドアの蝶番のようなもの）
    //
    // なぜヒンジジョイント？
    //   駆動輪はX軸周りにのみ回転し、本体との相対位置は固定です
    //   これがまさにヒンジジョイントの特性です

    // 左輪用のヒンジジョイントを作成
    left_hinge = dJointCreateHinge(world, 0);

    // dJointAttach(): ジョイントを2つの剛体に取り付ける
    // 引数：ジョイント, 剛体1, 剛体2
    dJointAttach(left_hinge, robot_body, left_wheel);

    // dJointSetHingeAnchor(): ヒンジの中心位置（回転軸の位置）を設定
    // 車輪の中心位置を指定します
    dJointSetHingeAnchor(left_hinge, x - WHEEL_BASE / 2, y, wheel_center_z);

    // dJointSetHingeAxis(): ヒンジの回転軸を設定
    // X軸(1,0,0)周りに回転 → 車輪が前後に転がる
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    // 右輪用のヒンジジョイントを作成（左輪と同様）
    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, x + WHEEL_BASE / 2, y, wheel_center_z);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    // ===================================================================
    // 4. 前後のキャスターを作成
    // ===================================================================
    // キャスターは球体で、地面との接触高さは駆動輪と同じです
    //
    // キャスターの配置：
    //   - 前方：本体の前端から少し飛び出す
    //   - 後方：本体の後端から少し飛び出す
    //   - 高さ：球の中心がCASTER_RADIUS（地面に接する高さ）

    dReal caster_center_z = CASTER_RADIUS;  // 球の中心高さ = 半径

    // 前方キャスターを作成
    // Y座標：本体の前端 + キャスター半径/2
    front_caster = createSphere(world, space,
                               x, y + BODY_WIDTH / 2 + CASTER_RADIUS / 2, caster_center_z,
                               CASTER_RADIUS, 0.1);  // 質量は軽め（0.1kg）

    // 後方キャスターを作成
    // Y座標：本体の後端 - キャスター半径/2
    rear_caster = createSphere(world, space,
                              x, y - BODY_WIDTH / 2 - CASTER_RADIUS / 2, caster_center_z,
                              CASTER_RADIUS, 0.1);

    // ===================================================================
    // 5. ボールジョイントでキャスターを本体に接続
    // ===================================================================
    // ボールジョイント（Ball Joint）とは？
    //   全方向に自由に回転できるジョイントです（肩関節のようなもの）
    //
    // なぜボールジョイント？
    //   キャスターは進行方向に応じて自由に向きを変える必要があります
    //   ボールジョイントなら全方向に回転できるので最適です

    // 前方キャスター用のボールジョイント
    front_caster_joint = dJointCreateBall(world, 0);
    dJointAttach(front_caster_joint, robot_body, front_caster);

    // dJointSetBallAnchor(): ボールジョイントの中心位置を設定
    // 本体の底面に接続点を配置
    dJointSetBallAnchor(front_caster_joint, x, y + BODY_WIDTH / 2 + CASTER_RADIUS / 2,
                       z - BODY_HEIGHT / 2);

    // 後方キャスター用のボールジョイント（前方と同様）
    rear_caster_joint = dJointCreateBall(world, 0);
    dJointAttach(rear_caster_joint, robot_body, rear_caster);
    dJointSetBallAnchor(rear_caster_joint, x, y - BODY_WIDTH / 2 - CASTER_RADIUS / 2,
                       z - BODY_HEIGHT / 2);

    // ===================================================================
    // 6. 差動2輪駆動コントローラーを初期化
    // ===================================================================
    // DifferentialDriveクラス：運動学計算を行うヘルパークラス
    // 引数：車輪間隔（WHEEL_BASE）、車輪半径（WHEEL_RADIUS）
    //
    // このクラスが行うこと：
    //   目標速度（直進速度 v, 角速度 ω）→ 各車輪の速度（左輪、右輪）
    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
}

// ===================================================================
// リセット関数
// ===================================================================
void reset() {
    // 正しい高さの計算：車輪半径 + 本体高さの半分
    // これで本体の底面が車輪の中心と同じ高さになり、地面に接地します
    dReal correct_height = WHEEL_RADIUS + BODY_HEIGHT / 2;

    dBodySetPosition(robot_body, 0, 0, correct_height);
    dBodySetLinearVel(robot_body, 0, 0, 0);
    dBodySetAngularVel(robot_body, 0, 0, 0);

    // 回転をリセット（Z軸周りに0度 = 前を向く）
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 1, 0);
    dBodySetRotation(robot_body, R);
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    // リセット処理
    if (Viewer::shouldReset()) {
        reset();
        Viewer::setShouldReset(false);
    }

    // ===================================================================
    // 差動2輪駆動の運動学計算
    // ===================================================================
    // 目標：直進速度 v [m/s] と 角速度 ω [rad/s]
    // 求める：左輪速度 v_L [rad/s] と 右輪速度 v_R [rad/s]
    //
    // 運動学の式：
    //   v = r * (v_L + v_R) / 2     （直進速度）
    //   ω = r * (v_R - v_L) / L     （角速度）
    //
    // ここで：
    //   r = 車輪半径、L = 車輪間隔
    //
    // 逆算すると：
    //   v_L = (v - ω*L/2) / r
    //   v_R = (v + ω*L/2) / r
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel,
                                      left_vel, right_vel);

    // ===================================================================
    // モーターの制御
    // ===================================================================
    // dJointSetHingeParam(): ヒンジジョイントのパラメータを設定
    //
    // dParamVel: 目標速度を設定
    //   この速度を実現するように力が自動的に計算されます
    //
    // dParamFMax: 最大トルク（力）を設定
    //   モーターが出せる最大の力です
    //   大きすぎると不安定、小さすぎると力不足
    dJointSetHingeParam(left_hinge, dParamVel, left_vel);
    dJointSetHingeParam(left_hinge, dParamFMax, 10.0);  // 最大トルク 10Nm

    dJointSetHingeParam(right_hinge, dParamVel, right_vel);
    dJointSetHingeParam(right_hinge, dParamFMax, 10.0);

    // 衝突検出
    dSpaceCollide(space, 0, &nearCallback);

    // 物理演算を1ステップ進める
    dWorldStep(world, dt);

    // 接触ジョイントをクリア
    dJointGroupEmpty(contact_group);

    // ロボットの位置を表示（ログが溢れないよう500フレームごと）
    static int counter = 0;
    if (counter++ % 500 == 0) {
        const dReal* pos = dBodyGetPosition(robot_body);
        const dReal* R = dBodyGetRotation(robot_body);
        dReal yaw = getYawFromRotation(R);  // Z軸周りの回転角（ヨー角）
        std::cout << "Robot pos: (" << pos[0] << ", " << pos[1]
                  << "), yaw: " << radToDeg(yaw) << "°" << std::endl;
    }
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

    // ロボット本体を描画（緑色）
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);

    // 左駆動輪を描画（黒色）
    const dReal* lw_pos = dBodyGetPosition(left_wheel);
    const dReal* lw_R = dBodyGetRotation(left_wheel);
    Viewer::setColor(0.2f, 0.2f, 0.2f);
    Viewer::drawCylinder(lw_pos, lw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    // 右駆動輪を描画（黒色）
    const dReal* rw_pos = dBodyGetPosition(right_wheel);
    const dReal* rw_R = dBodyGetRotation(right_wheel);
    Viewer::drawCylinder(rw_pos, rw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    // 前方キャスターを描画（グレー）
    const dReal* fc_pos = dBodyGetPosition(front_caster);
    const dReal* fc_R = dBodyGetRotation(front_caster);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawSphere(fc_pos, fc_R, CASTER_RADIUS);

    // 後方キャスターを描画（グレー）
    const dReal* rc_pos = dBodyGetPosition(rear_caster);
    const dReal* rc_R = dBodyGetRotation(rear_caster);
    Viewer::drawSphere(rc_pos, rc_R, CASTER_RADIUS);
}

// ===================================================================
// キーボードコールバック関数
// ===================================================================
// WASDキーでロボットを操作します
void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w':
        case 'W':
            // 前進：直進速度 0.5 m/s、角速度 0 rad/s
            target_linear_vel = 0.5;
            target_angular_vel = 0.0;
            std::cout << "前進" << std::endl;
            break;

        case 's':
        case 'S':
            // 後退：直進速度 -0.5 m/s（負の値）、角速度 0 rad/s
            target_linear_vel = -0.5;
            target_angular_vel = 0.0;
            std::cout << "後退" << std::endl;
            break;

        case 'a':
        case 'A':
            // 左旋回：ゆっくり前進しながら左に回転
            // 直進速度 0.3 m/s、角速度 1.0 rad/s（正 = 左回転）
            target_linear_vel = 0.3;
            target_angular_vel = 1.0;
            std::cout << "左旋回" << std::endl;
            break;

        case 'd':
        case 'D':
            // 右旋回：ゆっくり前進しながら右に回転
            // 直進速度 0.3 m/s、角速度 -1.0 rad/s（負 = 右回転）
            target_linear_vel = 0.3;
            target_angular_vel = -1.0;
            std::cout << "右旋回" << std::endl;
            break;

        case 'x':
        case 'X':
            // 停止：すべての速度をゼロに
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
            std::cout << "停止" << std::endl;
            break;
    }
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 15: Differential Drive Robot ===" << std::endl;
    std::cout << "差動2輪ロボットを操作しよう！" << std::endl;
    std::cout << std::endl;
    std::cout << "操作方法：" << std::endl;
    std::cout << "  W: 前進" << std::endl;
    std::cout << "  S: 後退" << std::endl;
    std::cout << "  A: 左旋回" << std::endl;
    std::cout << "  D: 右旋回" << std::endl;
    std::cout << "  X: 停止" << std::endl;
    std::cout << std::endl;
    std::cout << "※ 詳細な操作方法は CONTROLS.txt を参照してください" << std::endl;
    std::cout << std::endl;

    // ===================================================================
    // ODE初期化
    // ===================================================================
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);  // 地球の重力

    space = dHashSpaceCreate(0);           // 衝突検出空間
    contact_group = dJointGroupCreate(0);  // 接触ジョイントグループ

    // ===================================================================
    // 地面の作成
    // ===================================================================
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // ===================================================================
    // ロボットの作成
    // ===================================================================
    // 正しい高さを計算：車輪半径 + 本体高さの半分
    // これにより、車輪が地面に正しく接地します
    dReal robot_height = WHEEL_RADIUS + BODY_HEIGHT / 2;
    createRobot(0, 0, robot_height);

    // ===================================================================
    // ビューワーの作成
    // ===================================================================
    Viewer viewer(argc, argv, "15: Differential Drive Robot - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);  // キーボード入力を設定

    viewer.start();

    // ===================================================================
    // クリーンアップ
    // ===================================================================
    delete diff_drive;  // DifferentialDriveコントローラーを削除
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
