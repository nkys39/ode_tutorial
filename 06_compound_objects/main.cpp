// ===================================================================
// 06: Compound Objects - 複合オブジェクト
// ===================================================================
// このチュートリアルでは、複数の形状を組み合わせた複雑なオブジェクトを作成します。
// 1つの剛体に複数のジオメトリを取り付ける方法と、質量の計算方法を学びます。
// ===================================================================

#include "viewer.h"
#include "utils.h"
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
// 複合オブジェクトとは？
// ===================================================================
// 複数の形状を1つの剛体として動かすオブジェクトです
//
// 構成：
//   1つの剛体（Body）に、複数のジオメトリ（Geom）を取り付けます
//
// この例：中央の箱 + 左右の球
//   - 1つの剛体（compound_body）
//   - 3つのジオメトリ（box_geom, sphere1_geom, sphere2_geom）
//
// なぜ複合オブジェクトが必要？
//   ロボット、車、キャラクターなど、複雑な形状を持つオブジェクトは
//   単純な形状の組み合わせで表現します
dBodyID compound_body;                    // 1つの剛体（物理的性質）
dGeomID box_geom, sphere1_geom, sphere2_geom;  // 3つのジオメトリ（形状）

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
        contact[i].surface.bounce = 0.3;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// ===================================================================
// 複合オブジェクト作成関数
// ===================================================================
// 中央の箱と左右の球を組み合わせたダンベルのような形状を作ります
void createCompoundObject() {
    // ===================================================================
    // 1. 剛体の作成と初期位置設定
    // ===================================================================
    compound_body = dBodyCreate(world);
    dBodySetPosition(compound_body, 0, 0, 2.0);

    // ===================================================================
    // 2. 質量（dMass）の準備
    // ===================================================================
    // dMass構造体とは？
    //   質量、重心位置、慣性モーメントなどの情報を格納する構造体です
    //
    // なぜ質量を手動計算する必要がある？
    //   複数の形状を組み合わせるとき、それぞれの質量を合算し、
    //   正しい重心と慣性モーメントを計算する必要があります
    dMass m;        // 一時的な質量（各パーツごと）
    dMass m_total;  // 合計質量（最終的な質量）

    // dMassSetZero(): 質量をゼロで初期化
    // すべての値（質量、重心、慣性）をゼロにリセット
    dMassSetZero(&m_total);

    // ===================================================================
    // 3. 中央の箱を追加
    // ===================================================================
    // dMassSetBox(): 箱の質量を計算
    // 引数：dMass*, 密度, 幅, 奥行き, 高さ
    // 密度1で0.5×0.3×0.2の箱 → 質量 = 1 * 0.5 * 0.3 * 0.2 = 0.03kg
    // 慣性モーメントも自動計算されます
    dMassSetBox(&m, 1, 0.5, 0.3, 0.2);

    // dMassAdd(): 質量を合算
    // m_total に m を加算します（質量、重心、慣性を正しく合成）
    dMassAdd(&m_total, &m);

    // dCreateBox(): ジオメトリを作成
    // 注意：ここでは質量を設定しない（後でまとめて設定）
    box_geom = dCreateBox(space, 0.5, 0.3, 0.2);

    // dGeomSetBody(): ジオメトリを剛体に取り付ける
    // この時点で、box_geomはcompound_bodyの一部になります
    dGeomSetBody(box_geom, compound_body);

    // ===================================================================
    // 4. 左側の球を追加
    // ===================================================================
    // dMassSetSphere(): 球の質量を計算
    // 引数：dMass*, 密度, 半径
    // 密度1、半径0.15の球 → 質量 = (4/3)π * 0.15³ * 1 ≈ 0.014kg
    dMassSetSphere(&m, 1, 0.15);

    // dMassTranslate(): 質量の位置をオフセット
    // 球は剛体の中心から(-0.4, 0, 0)の位置にあります
    // この関数で重心位置と慣性モーメントを調整します
    dMassTranslate(&m, -0.4, 0, 0);

    // 合計質量に加算
    dMassAdd(&m_total, &m);

    // 球のジオメトリを作成
    sphere1_geom = dCreateSphere(space, 0.15);
    dGeomSetBody(sphere1_geom, compound_body);

    // dGeomSetOffsetPosition(): ジオメトリの相対位置を設定
    // ジオメトリを剛体の中心から(-0.4, 0, 0)オフセットした位置に配置
    // これにより、剛体が回転してもジオメトリは正しい相対位置を保ちます
    dGeomSetOffsetPosition(sphere1_geom, -0.4, 0, 0);

    // ===================================================================
    // 5. 右側の球を追加
    // ===================================================================
    dMassSetSphere(&m, 1, 0.15);
    dMassTranslate(&m, 0.4, 0, 0);  // 右側にオフセット
    dMassAdd(&m_total, &m);

    sphere2_geom = dCreateSphere(space, 0.15);
    dGeomSetBody(sphere2_geom, compound_body);
    dGeomSetOffsetPosition(sphere2_geom, 0.4, 0, 0);

    // ===================================================================
    // 6. 合計質量を剛体に設定
    // ===================================================================
    // dBodySetMass(): 剛体に質量を設定
    // m_total には以下が含まれています：
    //   - 合計質量：箱 + 球2個の質量
    //   - 重心位置：3つの形状の質量加重平均
    //   - 慣性モーメント：回転のしやすさ（正しく計算済み）
    dBodySetMass(compound_body, &m_total);
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
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

    // 剛体の位置と回転を取得
    const dReal* pos = dBodyGetPosition(compound_body);
    const dReal* R = dBodyGetRotation(compound_body);
    dReal box_sides[3] = {0.5, 0.3, 0.2};

    // ===================================================================
    // 中央の箱を描画
    // ===================================================================
    Viewer::setColor(0.7f, 0.4f, 0.2f);  // 茶色
    Viewer::drawBox(pos, R, box_sides);

    // ===================================================================
    // オフセットされた球を描画
    // ===================================================================
    // 問題：球は剛体の中心から(-0.4, 0, 0)と(0.4, 0, 0)にオフセットされています
    // しかし、剛体が回転すると、このオフセットも回転します
    //
    // 解決策：回転行列Rでオフセットを変換する必要があります
    dVector3 sphere_pos;
    dReal offset1[3] = {-0.4, 0, 0};  // 左側の球のオフセット
    dReal offset2[3] = {0.4, 0, 0};   // 右側の球のオフセット

    // ===================================================================
    // 回転行列による座標変換
    // ===================================================================
    // 回転行列 R（3x3）は、12要素の配列として格納されています：
    //   R = [R0  R1  R2  0]
    //       [R4  R5  R6  0]
    //       [R8  R9  R10 0]
    //
    // ワールド座標 = 剛体位置 + 回転行列 × ローカルオフセット
    //
    // 数式：
    //   sphere_pos[i] = pos[i] + R[i*4+0]*offset[0] + R[i*4+1]*offset[1] + R[i*4+2]*offset[2]
    //
    // これは行列とベクトルの掛け算です：
    //   [x']   [R0  R1  R2 ] [offset_x]   [pos_x]
    //   [y'] = [R4  R5  R6 ] [offset_y] + [pos_y]
    //   [z']   [R8  R9  R10] [offset_z]   [pos_z]

    // 左側の球の位置を計算
    for (int i = 0; i < 3; i++) {
        sphere_pos[i] = pos[i] + R[i*4] * offset1[0] + R[i*4+1] * offset1[1] + R[i*4+2] * offset1[2];
    }
    Viewer::setColor(0.3f, 0.7f, 0.9f);  // 水色
    Viewer::drawSphere(sphere_pos, R, 0.15);

    // 右側の球の位置を計算
    for (int i = 0; i < 3; i++) {
        sphere_pos[i] = pos[i] + R[i*4] * offset2[0] + R[i*4+1] * offset2[1] + R[i*4+2] * offset2[2];
    }
    Viewer::drawSphere(sphere_pos, R, 0.15);
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 06: Compound Objects ===" << std::endl;
    std::cout << "複合オブジェクト（ダンベル型）のシミュレーションです。" << std::endl;
    std::cout << "1つの剛体に3つの形状（箱+球2個）を組み合わせています。" << std::endl;
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
    // 複合オブジェクトを作成
    // ===================================================================
    // 中央の箱と左右の球を組み合わせたオブジェクトを作成
    // 質量の計算、オフセット設定など、複雑な処理が含まれます
    createCompoundObject();

    // ===================================================================
    // ビューワーの作成
    // ===================================================================
    Viewer viewer(argc, argv, "06: Compound Objects - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    // ===================================================================
    // クリーンアップ
    // ===================================================================
    // 剛体を破棄すると、取り付けられたすべてのジオメトリも自動的に破棄されます
    dBodyDestroy(compound_body);

    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
