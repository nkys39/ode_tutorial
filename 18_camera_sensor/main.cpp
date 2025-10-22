// ===================================================================
// 18: Camera Sensor - RGB-Dカメラセンサー（色+深度）
// ===================================================================
// このチュートリアルでは、RGB-Dカメラの基本を学びます。
// カラー画像と深度画像を同時に取得するセンサーです。
// ===================================================================

#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include <iostream>
#include <vector>
#include <GL/gl.h>
#include <GL/glext.h>

using namespace ode_tutorial;

// ===================================================================
// グローバル変数
// ===================================================================
dWorldID world;
dSpaceID space;
dGeomID ground_geom;

// ===================================================================
// RGB-Dカメラ
// ===================================================================
// RGB-D（Red Green Blue - Depth）とは？
//   カラー画像（RGB）と深度画像（Depth）を同時に取得できるセンサーです
//
// 仕組み：
//   RGBカメラ：通常のカラーカメラ（高解像度）
//   深度センサー：赤外線パターンやTOF（Time of Flight）で距離測定（低解像度）
//
// 用途：
//   - 3D物体認識（形と色の両方がわかる）
//   - ジェスチャー認識（Kinect等）
//   - 屋内SLAM（色情報で特徴点検出 + 深度で位置推定）
//   - ロボットの物体把持（対象物の位置と色を同時に取得）
//
// 実例：
//   - Microsoft Kinect
//   - Intel RealSense
//   - Apple TrueDepth（Face ID）
DepthCamera* camera;
dReal camera_pos[3] = {0, -3, 1.5};  // カメラ位置（少し後方、高さ1.5m）

// 障害物情報（色情報を含む）
struct ObstacleInfo {
    dBodyID body;       // 剛体
    dGeomID geom;       // 形状
    Color* color;       // 色情報（RGB画像用）
};
std::vector<ObstacleInfo> obstacles;

// 最新のRGB-D画像
RGBDImage current_rgbd_image;

// ===================================================================
// OpenGL FBO（フレームバッファオブジェクト）
// ===================================================================
// FBOとは？
//   画面に表示せず、テクスチャに直接レンダリングする仕組み
//   カメラ視点の画像を取得するために使用
GLuint fbo = 0;                      // フレームバッファID
GLuint rgb_texture = 0;              // RGBテクスチャ
GLuint depth_renderbuffer = 0;       // 深度バッファ
int current_rgb_width = 640;         // RGB解像度（幅）
int current_rgb_height = 480;        // RGB解像度（高さ）

// 解像度プリセット
struct ResolutionPreset {
    int width, height;
    const char* name;
};
ResolutionPreset resolutions[] = {
    {640, 480, "VGA (640x480)"},
    {1280, 720, "HD (1280x720)"},
    {1920, 1080, "Full HD (1920x1080)"}
};
int current_resolution = 0;

// ===================================================================
// FBO作成関数
// ===================================================================
void createFBO(int width, int height) {
    // 既存のFBOを削除
    if (fbo != 0) {
        glDeleteFramebuffers(1, &fbo);
        glDeleteTextures(1, &rgb_texture);
        glDeleteRenderbuffers(1, &depth_renderbuffer);
    }

    // FBO作成
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // RGBテクスチャ作成
    glGenTextures(1, &rgb_texture);
    glBindTexture(GL_TEXTURE_2D, rgb_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rgb_texture, 0);

    // 深度レンダーバッファ作成
    glGenRenderbuffers(1, &depth_renderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_renderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_renderbuffer);

    // FBO完全性チェック
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "FBO not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===================================================================
// カメラ視点からシーンをレンダリング（RGB画像取得用）
// ===================================================================
void renderSceneFromCamera() {
    if (fbo == 0) return;

    // FBOにバインド（画面ではなくテクスチャに描画）
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, current_rgb_width, current_rgb_height);

    // クリア（空色の背景）
    glClearColor(0.5f, 0.7f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // カメラのプロジェクション設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    dReal aspect = (dReal)current_rgb_width / current_rgb_height;
    dReal fov_v = camera->getVerticalFOV();
    dReal fov_h = camera->getHorizontalFOV();
    dReal near_clip = 0.1;
    dReal far_clip = camera->getLastImage().max_range;

    // 透視投影の手動設定
    dReal f = 1.0 / std::tan(fov_v / 2.0);
    glFrustum(-near_clip * aspect * std::tan(fov_h/2), near_clip * aspect * std::tan(fov_h/2),
              -near_clip * std::tan(fov_v/2), near_clip * std::tan(fov_v/2),
              near_clip, far_clip);

    // カメラビュー設定（カメラ位置から前方を見る）
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camera_pos[0], camera_pos[1], camera_pos[2],
              camera_pos[0] + 1, camera_pos[1] + 3, camera_pos[2],
              0, 0, 1);

    // ライティング有効化
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = {5.0f, -5.0f, 10.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    // 地面を描画
    glColor3f(0.5f, 0.5f, 0.5f);
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // 障害物を描画（色情報を使用）
    for (const auto& obs : obstacles) {
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);

        Viewer::setColor(obs.color->r, obs.color->g, obs.color->b);

        int geom_class = dGeomGetClass(obs.geom);
        if (geom_class == dBoxClass) {
            dVector3 sides;
            dGeomBoxGetLengths(obs.geom, sides);
            Viewer::drawBox(pos, R, sides);
        } else if (geom_class == dSphereClass) {
            dReal radius = dGeomSphereGetRadius(obs.geom);
            Viewer::drawSphere(pos, R, radius);
        }
    }

    // FBOのバインドを解除
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===================================================================
// FBOからRGB画像をキャプチャ
// ===================================================================
void captureRGBFromFBO() {
    if (fbo == 0) return;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // FBOからピクセルデータを読み取り
    std::vector<unsigned char> pixels(current_rgb_width * current_rgb_height * 3);
    glReadPixels(0, 0, current_rgb_width, current_rgb_height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // RGBDImageにコピー（OpenGLは原点が左下なので、上下反転）
    for (int y = 0; y < current_rgb_height; y++) {
        for (int x = 0; x < current_rgb_width; x++) {
            int src_idx = ((current_rgb_height - 1 - y) * current_rgb_width + x) * 3;
            current_rgbd_image.colors[y][x].r = pixels[src_idx] / 255.0f;
            current_rgbd_image.colors[y][x].g = pixels[src_idx + 1] / 255.0f;
            current_rgbd_image.colors[y][x].b = pixels[src_idx + 2] / 255.0f;
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// ===================================================================
// 障害物作成関数
// ===================================================================
void createObstacles() {
    ObstacleInfo obs;
    dGeomID geom;

    // 赤い箱
    obs.body = createBox(world, space, 1, 0, 0.5, 1.0, 0.5, 1.0, 1.0, &geom);
    obs.geom = geom;
    obs.color = new Color(0.8f, 0.2f, 0.2f);
    dGeomSetData(obs.geom, obs.color);
    obstacles.push_back(obs);

    // 緑の球
    obs.body = createSphere(world, space, -1, 1, 0.5, 0.5, 1.0, &geom);
    obs.geom = geom;
    obs.color = new Color(0.2f, 0.8f, 0.2f);
    dGeomSetData(obs.geom, obs.color);
    obstacles.push_back(obs);

    // 青い箱
    obs.body = createBox(world, space, 0, -1.5, 0.3, 0.6, 0.6, 0.6, 1.0, &geom);
    obs.geom = geom;
    obs.color = new Color(0.2f, 0.2f, 0.9f);
    dGeomSetData(obs.geom, obs.color);
    obstacles.push_back(obs);
}

// ===================================================================
// 解像度変更関数
// ===================================================================
void changeResolution(int preset_index) {
    if (preset_index < 0 || preset_index >= 3) return;

    current_resolution = preset_index;
    current_rgb_width = resolutions[preset_index].width;
    current_rgb_height = resolutions[preset_index].height;

    std::cout << "Resolution changed to: " << resolutions[preset_index].name << std::endl;

    // 新しい解像度でFBOを再作成
    createFBO(current_rgb_width, current_rgb_height);

    // 新しいRGB解像度でカメラを再作成
    //   RGB: current_rgb_width × current_rgb_height（高解像度）
    //   Depth: 64 × 48（低解像度、レイキャストは重いため）
    delete camera;
    camera = new DepthCamera(space, current_rgb_width, current_rgb_height, 64, 48, M_PI/2, M_PI/3, 10.0);
    current_rgbd_image = camera->getImage();
}

// ===================================================================
// キーボードコールバック関数
// ===================================================================
void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case '1':
            changeResolution(0); // 640x480
            break;
        case '2':
            changeResolution(1); // 1280x720
            break;
        case '3':
            changeResolution(2); // 1920x1080
            break;
        case 'h':
        case 'H':
            std::cout << "\n=== Keyboard Commands ===" << std::endl;
            std::cout << "1: VGA (640x480)" << std::endl;
            std::cout << "2: HD (1280x720)" << std::endl;
            std::cout << "3: Full HD (1920x1080)" << std::endl;
            std::cout << "h: Show this help" << std::endl;
            std::cout << "q: Quit" << std::endl;
            break;
    }
}

// ===================================================================
// シミュレーションステップ関数
// ===================================================================
void simulationStep(double dt) {
    dMatrix3 R;
    dRSetIdentity(R);

    // ===================================================================
    // 1. 深度画像をキャプチャ（レイキャスト、低解像度）
    // ===================================================================
    // DepthCamera::captureDepth(): レイキャストで深度測定
    //   レイキャストは計算コストが高いため、低解像度（64×48）で実行
    camera->captureDepth(camera_pos, R);
    current_rgbd_image = camera->getImage();

    // ===================================================================
    // 2. RGB画像をレンダリング（OpenGL FBO、高解像度）
    // ===================================================================
    // OpenGLでカメラ視点からシーンをレンダリング
    //   高解像度（640×480〜1920×1080）でも高速
    renderSceneFromCamera();
    captureRGBFromFBO();

    // 50ステップごとに画像情報を表示
    static int counter = 0;
    if (counter++ % 50 == 0) {
        std::cout << "RGB: " << current_rgbd_image.rgb_width << "x" << current_rgbd_image.rgb_height
                  << ", Depth: " << current_rgbd_image.depth_width << "x" << current_rgbd_image.depth_height << std::endl;
        std::cout << "Center pixel depth: "
                  << current_rgbd_image.depths[current_rgbd_image.depth_height/2][current_rgbd_image.depth_width/2] << "m" << std::endl;
    }

    dWorldStep(world, dt);
}

// ===================================================================
// カメラビュー描画（画面にオーバーレイ表示）
// ===================================================================
void drawCameraView() {
    if (current_rgbd_image.depths.empty() || current_rgbd_image.colors.empty()) return;

    // 現在の状態を保存
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // 2D描画のためライティング無効化
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    // ===================================================================
    // RGB画像を描画（大きいメインビュー）
    // ===================================================================
    float rgb_x = 0.03f;
    float rgb_y = 0.53f;
    float rgb_w = 0.45f;
    float rgb_h = 0.45f * current_rgbd_image.rgb_height / (float)current_rgbd_image.rgb_width;

    int rgb_w_pixels = current_rgbd_image.rgb_width;
    int rgb_h_pixels = current_rgbd_image.rgb_height;
    float pixel_w = rgb_w / rgb_w_pixels;
    float pixel_h = rgb_h / rgb_h_pixels;

    // パフォーマンスのため、高解像度の場合は間引いて表示
    int step = 1;
    if (rgb_w_pixels > 320) step = rgb_w_pixels / 320;

    glBegin(GL_QUADS);
    for (int y = 0; y < rgb_h_pixels; y += step) {
        for (int x = 0; x < rgb_w_pixels; x += step) {
            Color color = current_rgbd_image.colors[y][x];
            glColor3f(color.r, color.g, color.b);

            float px = rgb_x + x * pixel_w;
            float py = rgb_y + y * pixel_h;
            float pw = pixel_w * step;
            float ph = pixel_h * step;

            glVertex2f(px, py);
            glVertex2f(px + pw, py);
            glVertex2f(px + pw, py + ph);
            glVertex2f(px, py + ph);
        }
    }
    glEnd();

    // RGB画像の枠を描画
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(rgb_x, rgb_y);
    glVertex2f(rgb_x + rgb_w, rgb_y);
    glVertex2f(rgb_x + rgb_w, rgb_y + rgb_h);
    glVertex2f(rgb_x, rgb_y + rgb_h);
    glEnd();

    // RGB画像のラベル（解像度表示）
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(rgb_x + 0.01f, rgb_y + rgb_h - 0.03f);
    char label_rgb[64];
    snprintf(label_rgb, sizeof(label_rgb), "RGB Camera (%dx%d)", rgb_w_pixels, rgb_h_pixels);
    for (const char* c = label_rgb; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    }

    // ===================================================================
    // 深度画像を描画（小さいサブビュー）
    // ===================================================================
    float depth_x = 0.68f;
    float depth_y = 0.68f;
    float view_width = 0.28f;
    float view_height = 0.28f;

    int depth_w = current_rgbd_image.depth_width;
    int depth_h = current_rgbd_image.depth_height;
    pixel_w = view_width / depth_w;
    pixel_h = view_height / depth_h;

    glBegin(GL_QUADS);
    for (int y = 0; y < depth_h; y++) {
        for (int x = 0; x < depth_w; x++) {
            dReal depth = current_rgbd_image.depths[y][x];
            dReal max_range = current_rgbd_image.max_range;

            // 深度をグレースケールに変換（近い = 明るい）
            float intensity = 1.0f - (depth / max_range);
            if (depth >= max_range) {
                intensity = 0.0f; // 検出なし = 黒
            }

            glColor3f(intensity, intensity, intensity);

            float px = depth_x + x * pixel_w;
            float py = depth_y + y * pixel_h;

            glVertex2f(px, py);
            glVertex2f(px + pixel_w, py);
            glVertex2f(px + pixel_w, py + pixel_h);
            glVertex2f(px, py + pixel_h);
        }
    }
    glEnd();

    // 深度画像の枠を描画
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(depth_x, depth_y);
    glVertex2f(depth_x + view_width, depth_y);
    glVertex2f(depth_x + view_width, depth_y + view_height);
    glVertex2f(depth_x, depth_y + view_height);
    glEnd();

    // 深度画像のラベル
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(depth_x + 0.01f, depth_y + view_height - 0.03f);
    char label_depth[64];
    snprintf(label_depth, sizeof(label_depth), "Depth Sensor (%dx%d)", depth_w, depth_h);
    for (const char* c = label_depth; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    }

    // 状態を復元
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
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

    // 障害物を描画
    for (const auto& obs : obstacles) {
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);

        Viewer::setColor(obs.color->r, obs.color->g, obs.color->b);

        int geom_class = dGeomGetClass(obs.geom);
        if (geom_class == dBoxClass) {
            dVector3 sides;
            dGeomBoxGetLengths(obs.geom, sides);
            Viewer::drawBox(pos, R, sides);
        } else if (geom_class == dSphereClass) {
            dReal radius = dGeomSphereGetRadius(obs.geom);
            Viewer::drawSphere(pos, R, radius);
        }
    }

    // カメラ位置を描画
    dMatrix3 R;
    dRSetIdentity(R);
    Viewer::setColor(0.2f, 0.2f, 0.9f);
    Viewer::drawSphere(camera_pos, R, 0.1);

    // カメラの視錐台（視野範囲）を描画
    Viewer::setColor(0.9f, 0.9f, 0.2f);
    dReal fov_h = camera->getHorizontalFOV();
    dReal fov_v = camera->getVerticalFOV();
    dReal range = 3.0;

    dReal corners[4][3];
    // 視錐台の4隅を計算
    corners[0][0] = camera_pos[0] + range * std::cos(fov_h / 2) * std::cos(fov_v / 2);
    corners[0][1] = camera_pos[1] + range * std::sin(fov_h / 2) * std::cos(fov_v / 2);
    corners[0][2] = camera_pos[2] + range * std::sin(fov_v / 2);

    corners[1][0] = camera_pos[0] + range * std::cos(-fov_h / 2) * std::cos(fov_v / 2);
    corners[1][1] = camera_pos[1] + range * std::sin(-fov_h / 2) * std::cos(fov_v / 2);
    corners[1][2] = camera_pos[2] + range * std::sin(fov_v / 2);

    corners[2][0] = camera_pos[0] + range * std::cos(-fov_h / 2) * std::cos(-fov_v / 2);
    corners[2][1] = camera_pos[1] + range * std::sin(-fov_h / 2) * std::cos(-fov_v / 2);
    corners[2][2] = camera_pos[2] + range * std::sin(-fov_v / 2);

    corners[3][0] = camera_pos[0] + range * std::cos(fov_h / 2) * std::cos(-fov_v / 2);
    corners[3][1] = camera_pos[1] + range * std::sin(fov_h / 2) * std::cos(-fov_v / 2);
    corners[3][2] = camera_pos[2] + range * std::sin(-fov_v / 2);

    // カメラから4隅への線を描画
    for (int i = 0; i < 4; i++) {
        Viewer::drawLine(camera_pos, corners[i], 0.9f, 0.9f, 0.2f);
    }

    // カメラビューをオーバーレイ表示
    drawCameraView();
}

// ===================================================================
// メイン関数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << "=== 18: RGB-D Camera Sensor（RGB-Dカメラ） ===" << std::endl;
    std::cout << "Press '1', '2', '3' to change RGB resolution" << std::endl;
    std::cout << "Press 'h' for help" << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    createObstacles();

    // RGB-Dカメラを作成
    //   RGB: 640×480（高解像度、FBOで高速レンダリング）
    //   Depth: 64×48（低解像度、レイキャストは重いため）
    //   水平FOV: 90度、垂直FOV: 60度、最大距離: 10m
    camera = new DepthCamera(space, current_rgb_width, current_rgb_height, 64, 48, M_PI/2, M_PI/3, 10.0);
    current_rgbd_image = camera->getImage();

    // ビューワーの作成
    Viewer viewer(argc, argv, "18: RGB-D Camera - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    // FBOを作成（GLUTウィンドウ作成後に実行）
    createFBO(current_rgb_width, current_rgb_height);

    viewer.start();

    // クリーンアップ
    if (fbo != 0) {
        glDeleteFramebuffers(1, &fbo);
        glDeleteTextures(1, &rgb_texture);
        glDeleteRenderbuffers(1, &depth_renderbuffer);
    }

    delete camera;
    for (auto& obs : obstacles) {
        delete obs.color;
        dBodyDestroy(obs.body);
    }
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
