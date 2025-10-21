// ===================================================================
// 05: Different Shapes - さまざまな形状
// ===================================================================
// このチュートリアルでは、ODEがサポートするさまざまな形状を学びます。
// 箱、球、カプセル、円柱の4種類の基本形状を理解しましょう。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;           // 物理世界
dSpaceID space;           // 衝突検出空間
dJointGroupID contact_group;  // 接触ジョイントグループ
dGeomID ground_geom;      // 地面のジオメトリ

// ===================================================================
// 形状データ構造
// ===================================================================
// Shape構造体：さまざまな形状を統一的に管理するためのデータ構造
//
// ODEがサポートする基本形状：
//   - Box（箱）：直方体、パラメータは [幅, 奥行き, 高さ]
//   - Sphere（球）：完全な球体、パラメータは [半径]
//   - Capsule（カプセル）：円柱の両端に半球を付けた形状
//                          パラメータは [半径, 円柱部分の長さ]
//   - Cylinder（円柱）：円柱、パラメータは [半径, 長さ]
//
// カプセルとは？
//   薬のカプセルのような形状です。転がりやすく、キャラクターの
//   当たり判定などによく使われます（上下に丸みがあるため引っかかりにくい）
struct Shape {
    dBodyID body;      // 剛体
    dGeomID geom;      // ジオメトリ
    int type;          // 形状の種類
                       //   0 = Box（箱）
                       //   1 = Sphere（球）
                       //   2 = Capsule（カプセル）
                       //   3 = Cylinder（円柱）
    dReal params[3];   // 形状パラメータ（形状ごとに意味が異なる）
                       //   Box:      [幅, 奥行き, 高さ]
                       //   Sphere:   [半径, 未使用, 未使用]
                       //   Capsule:  [半径, 長さ, 未使用]
                       //   Cylinder: [半径, 長さ, 未使用]
};

std::vector<Shape> shapes;  // すべての形状を格納する配列

// ===================================================================
// 衝突コールバック関数
// ===================================================================
// どんな形状でも、ODEが自動的に衝突判定を行います
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;

    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];

    // dCollide(): 形状の組み合わせを自動判別
    // 例：箱と球、カプセルと円柱、など
    //     ODEが内部で最適なアルゴリズムを選択します
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.7;          // 摩擦係数（やや高め）
        contact[i].surface.bounce = 0.2;      // 反発係数（あまり跳ねない）
        contact[i].surface.soft_cfm = 0.01;   // ソフト接触

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// 形状作成関数
// ===================================================================
// 4種類の基本形状を作成します
void createShapes() {
    Shape shape;

    // ===================================================================
    // 1. Box（箱）
    // ===================================================================
    shape.type = 0;
    shape.params[0] = 0.4;  // X方向の幅
    shape.params[1] = 0.3;  // Y方向の奥行き
    shape.params[2] = 0.2;  // Z方向の高さ

    // createBox(): 箱を作成
    // 引数：world, space, x, y, z, 幅, 奥行き, 高さ, 質量, geom出力先
    shape.body = createBox(world, space, -1.5, 0, 2.0,
                          shape.params[0], shape.params[1], shape.params[2], 1.0, &shape.geom);
    shapes.push_back(shape);

    // ===================================================================
    // 2. Sphere（球）
    // ===================================================================
    shape.type = 1;
    shape.params[0] = 0.25;  // 半径

    // createSphere(): 球を作成
    // 引数：world, space, x, y, z, 半径, 質量, geom出力先
    shape.body = createSphere(world, space, -0.5, 0, 2.5, shape.params[0], 1.0, &shape.geom);
    shapes.push_back(shape);

    // ===================================================================
    // 3. Capsule（カプセル）
    // ===================================================================
    // カプセルの構造：
    //   [半球] - [円柱] - [半球]
    //   半球の半径 = params[0]
    //   円柱部分の長さ = params[1]
    //   全長 = params[1] + 2 * params[0]
    shape.type = 2;
    shape.params[0] = 0.15;  // 半径
    shape.params[1] = 0.5;   // 円柱部分の長さ
                             // 全長 = 0.5 + 2*0.15 = 0.8m

    // createCapsule(): カプセルを作成
    // 引数：world, space, x, y, z, 半径, 長さ（円柱部分）, 質量, geom出力先
    // 注意：長さは「全長」ではなく「円柱部分のみ」の長さです
    shape.body = createCapsule(world, space, 0.5, 0, 3.0,
                              shape.params[0], shape.params[1], 1.0, &shape.geom);
    shapes.push_back(shape);

    // ===================================================================
    // 4. Cylinder（円柱）
    // ===================================================================
    shape.type = 3;
    shape.params[0] = 0.2;  // 半径
    shape.params[1] = 0.4;  // 長さ

    // createCylinder(): 円柱を作成
    // 引数：world, space, x, y, z, 半径, 長さ, 質量, geom出力先
    // 注意：円柱は両端が平らです（カプセルと違い丸みがない）
    shape.body = createCylinder(world, space, 1.5, 0, 2.0,
                               shape.params[0], shape.params[1], 1.0, &shape.geom);
    shapes.push_back(shape);
}

// ===================================================================
// リセット関数
// ===================================================================
void reset() {
    // 配列の境界チェックをしながら各形状の位置をリセット
    // empty() と size() で安全に配列アクセス
    if (!shapes.empty()) dBodySetPosition(shapes[0].body, -1.5, 0, 2.0);  // Box
    if (shapes.size() > 1) dBodySetPosition(shapes[1].body, -0.5, 0, 2.5); // Sphere
    if (shapes.size() > 2) dBodySetPosition(shapes[2].body, 0.5, 0, 3.0);  // Capsule
    if (shapes.size() > 3) dBodySetPosition(shapes[3].body, 1.5, 0, 2.0);  // Cylinder

    // すべての形状の速度をリセット
    for (auto& shape : shapes) {
        dBodySetLinearVel(shape.body, 0, 0, 0);
        dBodySetAngularVel(shape.body, 0, 0, 0);
    }
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    if (Viewer::shouldReset()) {
        reset();
        Viewer::setShouldReset(false);
    }

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

    // すべての形状を描画
    for (const auto& shape : shapes) {
        const dReal* pos = dBodyGetPosition(shape.body);
        const dReal* R = dBodyGetRotation(shape.body);

        // ===================================================================
        // switch文による分岐処理
        // ===================================================================
        // switch (変数) { case 値: ... break; }
        //   変数の値に応じて処理を分岐します
        //
        // if-else との違い：
        //   if-else: 複雑な条件（< や && など）に対応
        //   switch: 単純な値の比較に特化（読みやすく高速）
        //
        // break とは？
        //   case の処理を終了し、switch文を抜けます
        //   忘れると次の case も実行されてしまうので注意
        switch (shape.type) {
            case 0: // Box（箱）
                Viewer::setColor(0.9f, 0.3f, 0.3f);  // 赤色
                Viewer::drawBox(pos, R, shape.params);
                break;

            case 1: // Sphere（球）
                Viewer::setColor(0.3f, 0.9f, 0.3f);  // 緑色
                Viewer::drawSphere(pos, R, shape.params[0]);
                break;

            case 2: // Capsule（カプセル）
                Viewer::setColor(0.3f, 0.3f, 0.9f);  // 青色
                // drawCapsule(位置, 回転, 長さ, 半径)
                // 注意：引数の順番が createCapsule と異なります
                Viewer::drawCapsule(pos, R, shape.params[1], shape.params[0]);
                break;

            case 3: // Cylinder（円柱）
                Viewer::setColor(0.9f, 0.9f, 0.3f);  // 黄色
                // drawCylinder(位置, 回転, 長さ, 半径)
                Viewer::drawCylinder(pos, R, shape.params[1], shape.params[0]);
                break;
        }
    }
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 05: Different Shapes ===" << std::endl;
    std::cout << "さまざまな形状が落下して衝突するシミュレーションです。" << std::endl;
    std::cout << "形状の種類：Box（赤）、Sphere（緑）、Capsule（青）、Cylinder（黄）" << std::endl;
    std::cout << std::endl;

    // ===================================================================
    // ODE初期化
    // ===================================================================
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // ===================================================================
    // 4種類の形状を作成
    // ===================================================================
    // Box, Sphere, Capsule, Cylinder の順に作成されます
    createShapes();

    // ===================================================================
    // ビューワーの作成
    // ===================================================================
    Viewer viewer(argc, argv, "05: Different Shapes - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    // ===================================================================
    // クリーンアップ
    // ===================================================================
    for (auto& shape : shapes) {
        dBodyDestroy(shape.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
