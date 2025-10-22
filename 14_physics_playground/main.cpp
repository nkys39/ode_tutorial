// ===================================================================
// 14: Physics Playground - 物理遊び場（完全実装）
// ===================================================================
// 完全実装：対話的な物理シミュレーション環境
//
// 物理遊び場とは？
//   ユーザーが自由にオブジェクトを配置・操作できる対話的な環境です
//   - リアルタイムでオブジェクトを追加・削除
//   - 様々な形状や素材のオブジェクト
//   - 重力や物理パラメーターの変更
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
#include <random>

using namespace ode_tutorial;

// グローバル変数
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// オブジェクト情報
struct GameObject {
    dBodyID body;
    dGeomID geom;
    int shape_type;  // 0:箱, 1:球, 2:円柱, 3:カプセル
    float r, g, b;   // 色
};

std::vector<GameObject> objects;

// 物理パラメーター
bool gravity_enabled = true;
dReal default_bounce = 0.5;
dReal default_friction = 0.8;

// ランダム数生成器
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> pos_dis(-2.0, 2.0);
std::uniform_real_distribution<> size_dis(0.2, 0.6);
std::uniform_real_distribution<> color_dis(0.3, 1.0);

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
        contact[i].surface.mu = default_friction;
        contact[i].surface.bounce = default_bounce;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// 箱を追加
void addBox() {
    dReal x = pos_dis(gen);
    dReal y = pos_dis(gen);
    dReal z = 3.0 + pos_dis(gen);  // 高い位置から落とす

    dReal lx = size_dis(gen);
    dReal ly = size_dis(gen);
    dReal lz = size_dis(gen);

    dGeomID geom;
    dBodyID body = createBox(world, space, x, y, z, lx, ly, lz, 1.0, &geom);

    GameObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.shape_type = 0;  // 箱
    obj.r = color_dis(gen);
    obj.g = color_dis(gen);
    obj.b = color_dis(gen);

    objects.push_back(obj);

    std::cout << "箱を追加（合計 " << objects.size() << " 個）" << std::endl;
}

// 球を追加
void addSphere() {
    dReal x = pos_dis(gen);
    dReal y = pos_dis(gen);
    dReal z = 3.0 + pos_dis(gen);

    dReal radius = size_dis(gen) / 2;

    dGeomID geom;
    dBodyID body = createSphere(world, space, x, y, z, radius, 1.0, &geom);

    GameObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.shape_type = 1;  // 球
    obj.r = color_dis(gen);
    obj.g = color_dis(gen);
    obj.b = color_dis(gen);

    objects.push_back(obj);

    std::cout << "球を追加（合計 " << objects.size() << " 個）" << std::endl;
}

// 円柱を追加
void addCylinder() {
    dReal x = pos_dis(gen);
    dReal y = pos_dis(gen);
    dReal z = 3.0 + pos_dis(gen);

    dReal radius = size_dis(gen) / 3;
    dReal length = size_dis(gen);

    dGeomID geom;
    dBodyID body = createCylinder(world, space, x, y, z, radius, length, 1.0, &geom);

    GameObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.shape_type = 2;  // 円柱
    obj.r = color_dis(gen);
    obj.g = color_dis(gen);
    obj.b = color_dis(gen);

    objects.push_back(obj);

    std::cout << "円柱を追加（合計 " << objects.size() << " 個）" << std::endl;
}

// カプセルを追加
void addCapsule() {
    dReal x = pos_dis(gen);
    dReal y = pos_dis(gen);
    dReal z = 3.0 + pos_dis(gen);

    dReal radius = size_dis(gen) / 4;
    dReal length = size_dis(gen);

    dGeomID geom;
    dBodyID body = createCapsule(world, space, x, y, z, length, radius, 1.0, &geom);

    GameObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.shape_type = 3;  // カプセル
    obj.r = color_dis(gen);
    obj.g = color_dis(gen);
    obj.b = color_dis(gen);

    objects.push_back(obj);

    std::cout << "カプセルを追加（合計 " << objects.size() << " 個）" << std::endl;
}

// 全オブジェクトを削除
void clearAllObjects() {
    for (auto& obj : objects) {
        dBodyDestroy(obj.body);
        dGeomDestroy(obj.geom);
    }
    objects.clear();

    std::cout << "全オブジェクトを削除しました" << std::endl;
}

// 重力のトグル
void toggleGravity() {
    gravity_enabled = !gravity_enabled;

    if (gravity_enabled) {
        dWorldSetGravity(world, 0, 0, -9.81);
        std::cout << "重力: ON" << std::endl;
    } else {
        dWorldSetGravity(world, 0, 0, 0);
        std::cout << "重力: OFF" << std::endl;
    }
}

// 反発係数を変更
void changeBounce(dReal delta) {
    default_bounce += delta;
    if (default_bounce < 0.0) default_bounce = 0.0;
    if (default_bounce > 1.0) default_bounce = 1.0;

    std::cout << "反発係数: " << default_bounce << std::endl;
}

// 摩擦係数を変更
void changeFriction(dReal delta) {
    default_friction += delta;
    if (default_friction < 0.0) default_friction = 0.0;
    if (default_friction > 2.0) default_friction = 2.0;

    std::cout << "摩擦係数: " << default_friction << std::endl;
}

// シミュレーションステップ
void simulationStep(double dt) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

// 描画コールバック
void drawScene() {
    // 地面を描画
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // 全オブジェクトを描画
    for (const auto& obj : objects) {
        const dReal* pos = dBodyGetPosition(obj.body);
        const dReal* R = dBodyGetRotation(obj.body);

        Viewer::setColor(obj.r, obj.g, obj.b);

        switch (obj.shape_type) {
            case 0: {  // 箱
                dVector3 sides;
                dGeomBoxGetLengths(obj.geom, sides);
                Viewer::drawBox(pos, R, sides);
                break;
            }
            case 1: {  // 球
                dReal radius = dGeomSphereGetRadius(obj.geom);
                Viewer::drawSphere(pos, R, radius);
                break;
            }
            case 2: {  // 円柱
                dReal radius, length;
                dGeomCylinderGetParams(obj.geom, &radius, &length);
                Viewer::drawCylinder(pos, R, length, radius);
                break;
            }
            case 3: {  // カプセル
                dReal radius, length;
                dGeomCapsuleGetParams(obj.geom, &radius, &length);
                Viewer::drawCapsule(pos, R, length, radius);
                break;
            }
        }
    }
}

// キーボードコールバック
void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        // ===================================================================
        // オブジェクト追加
        // ===================================================================
        case '1':
            addBox();
            break;

        case '2':
            addSphere();
            break;

        case '3':
            addCylinder();
            break;

        case '4':
            addCapsule();
            break;

        // ===================================================================
        // オブジェクト削除・リセット
        // ===================================================================
        case 'c':
        case 'C':
            clearAllObjects();
            break;

        // ===================================================================
        // 物理パラメーター調整
        // ===================================================================
        case 'g':
        case 'G':
            toggleGravity();
            break;

        case '+':
        case '=':
            changeBounce(0.1);
            break;

        case '-':
        case '_':
            changeBounce(-0.1);
            break;

        case ']':
            changeFriction(0.1);
            break;

        case '[':
            changeFriction(-0.1);
            break;

        // ===================================================================
        // ヘルプ表示
        // ===================================================================
        case 'h':
        case 'H':
        case '?':
            std::cout << "\n=== 操作方法 ===" << std::endl;
            std::cout << "オブジェクト追加:" << std::endl;
            std::cout << "  1: 箱を追加" << std::endl;
            std::cout << "  2: 球を追加" << std::endl;
            std::cout << "  3: 円柱を追加" << std::endl;
            std::cout << "  4: カプセルを追加" << std::endl;
            std::cout << std::endl;
            std::cout << "環境制御:" << std::endl;
            std::cout << "  G: 重力ON/OFF" << std::endl;
            std::cout << "  C: 全削除" << std::endl;
            std::cout << std::endl;
            std::cout << "物理パラメーター:" << std::endl;
            std::cout << "  +/-: 反発係数（現在: " << default_bounce << "）" << std::endl;
            std::cout << "  [/]: 摩擦係数（現在: " << default_friction << "）" << std::endl;
            std::cout << std::endl;
            break;
    }
}

// メイン関数
int main(int argc, char** argv) {
    std::cout << "=== 14: Physics Playground（物理遊び場）- 完全実装 ===" << std::endl;
    std::cout << "対話的な物理シミュレーション環境" << std::endl;
    std::cout << "様々な形状のオブジェクトを追加して遊べます！" << std::endl;
    std::cout << std::endl;
    std::cout << "操作方法:" << std::endl;
    std::cout << "  1: 箱を追加" << std::endl;
    std::cout << "  2: 球を追加" << std::endl;
    std::cout << "  3: 円柱を追加" << std::endl;
    std::cout << "  4: カプセルを追加" << std::endl;
    std::cout << "  G: 重力ON/OFF（現在: ON）" << std::endl;
    std::cout << "  C: 全削除" << std::endl;
    std::cout << "  +/-: 反発係数調整（現在: " << default_bounce << "）" << std::endl;
    std::cout << "  [/]: 摩擦係数調整（現在: " << default_friction << "）" << std::endl;
    std::cout << "  H: ヘルプ表示" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // 初期オブジェクトをいくつか追加
    addBox();
    addSphere();
    addCylinder();

    Viewer viewer(argc, argv, "14: Physics Playground (Full) - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);
    viewer.start();

    // クリーンアップ
    clearAllObjects();
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
