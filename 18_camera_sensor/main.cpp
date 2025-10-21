#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include <iostream>
#include <vector>
#include <GL/gl.h>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dGeomID ground_geom;

DepthCamera* camera;
dReal camera_pos[3] = {0, -3, 1.5};

struct ObstacleInfo {
    dBodyID body;
    dGeomID geom;
    Color* color;
};
std::vector<ObstacleInfo> obstacles;

// Store latest depth image for visualization
DepthImage current_depth_image;

void createObstacles() {
    ObstacleInfo obs;
    dGeomID geom;

    // Red box
    obs.body = createBox(world, space, 1, 0, 0.5, 1.0, 0.5, 1.0, 1.0, &geom);
    obs.geom = geom;
    obs.color = new Color(0.8f, 0.2f, 0.2f);
    dGeomSetData(obs.geom, obs.color);
    obstacles.push_back(obs);

    // Green sphere
    obs.body = createSphere(world, space, -1, 1, 0.5, 0.5, 1.0, &geom);
    obs.geom = geom;
    obs.color = new Color(0.2f, 0.8f, 0.2f);
    dGeomSetData(obs.geom, obs.color);
    obstacles.push_back(obs);

    // Blue box
    obs.body = createBox(world, space, 0, -1.5, 0.3, 0.6, 0.6, 0.6, 1.0, &geom);
    obs.geom = geom;
    obs.color = new Color(0.2f, 0.2f, 0.9f);
    dGeomSetData(obs.geom, obs.color);
    obstacles.push_back(obs);
}

void simulationStep(double dt) {
    dMatrix3 R;
    dRSetIdentity(R);
    current_depth_image = camera->capture(camera_pos, R);

    static int counter = 0;
    if (counter++ % 50 == 0) {
        std::cout << "Depth image captured: " << current_depth_image.width << "x" << current_depth_image.height << std::endl;
        std::cout << "Center pixel depth: " << current_depth_image.depths[current_depth_image.height/2][current_depth_image.width/2] << "m" << std::endl;
    }

    dWorldStep(world, dt);
}

void drawCameraView() {
    if (current_depth_image.depths.empty()) return;

    // Save current state
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Disable lighting for 2D drawing
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    int width = current_depth_image.width;
    int height = current_depth_image.height;
    float view_width = 0.35f;
    float view_height = 0.35f;

    // Draw RGB color view (larger, main view)
    float rgb_x = 0.03f;
    float rgb_y = 0.63f;
    float rgb_w = view_width * 1.3f;
    float rgb_h = view_height * 1.3f;

    // Draw RGB pixels
    float pixel_w = rgb_w / width;
    float pixel_h = rgb_h / height;

    glBegin(GL_QUADS);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Color color = current_depth_image.colors[y][x];
            glColor3f(color.r, color.g, color.b);

            float px = rgb_x + x * pixel_w;
            float py = rgb_y + y * pixel_h;

            glVertex2f(px, py);
            glVertex2f(px + pixel_w, py);
            glVertex2f(px + pixel_w, py + pixel_h);
            glVertex2f(px, py + pixel_h);
        }
    }
    glEnd();

    // Draw border for RGB view
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(rgb_x, rgb_y);
    glVertex2f(rgb_x + rgb_w, rgb_y);
    glVertex2f(rgb_x + rgb_w, rgb_y + rgb_h);
    glVertex2f(rgb_x, rgb_y + rgb_h);
    glEnd();

    // Draw label for RGB
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(rgb_x + 0.01f, rgb_y + rgb_h - 0.03f);
    const char* label_rgb = "Camera View (RGB)";
    for (const char* c = label_rgb; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    }

    // Draw depth view (smaller, secondary view)
    float depth_x = 0.68f;
    float depth_y = 0.68f;

    pixel_w = view_width / width;
    pixel_h = view_height / height;

    glBegin(GL_QUADS);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dReal depth = current_depth_image.depths[y][x];
            dReal max_range = current_depth_image.max_range;

            // Convert depth to grayscale (closer = brighter)
            float intensity = 1.0f - (depth / max_range);
            if (depth >= max_range) {
                intensity = 0.0f; // Black for no detection
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

    // Draw border for depth view
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(depth_x, depth_y);
    glVertex2f(depth_x + view_width, depth_y);
    glVertex2f(depth_x + view_width, depth_y + view_height);
    glVertex2f(depth_x, depth_y + view_height);
    glEnd();

    // Draw label for depth
    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(depth_x + 0.01f, depth_y + view_height - 0.03f);
    const char* label_depth = "Depth";
    for (const char* c = label_depth; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    }

    // Restore state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawScene() {
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);
    
    for (const auto& obs : obstacles) {
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);

        // Use the stored color
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
    
    // Draw camera position
    dMatrix3 R;
    dRSetIdentity(R);
    Viewer::setColor(0.2f, 0.2f, 0.9f);
    Viewer::drawSphere(camera_pos, R, 0.1);

    // Draw camera frustum (visualization of camera field of view)
    Viewer::setColor(0.9f, 0.9f, 0.2f);
    dReal fov_h = camera->getHorizontalFOV();
    dReal fov_v = camera->getVerticalFOV();
    dReal range = 3.0;

    dReal corners[4][3];
    // Top-left
    corners[0][0] = camera_pos[0] + range * std::cos(fov_h / 2) * std::cos(fov_v / 2);
    corners[0][1] = camera_pos[1] + range * std::sin(fov_h / 2) * std::cos(fov_v / 2);
    corners[0][2] = camera_pos[2] + range * std::sin(fov_v / 2);
    // Top-right
    corners[1][0] = camera_pos[0] + range * std::cos(-fov_h / 2) * std::cos(fov_v / 2);
    corners[1][1] = camera_pos[1] + range * std::sin(-fov_h / 2) * std::cos(fov_v / 2);
    corners[1][2] = camera_pos[2] + range * std::sin(fov_v / 2);
    // Bottom-right
    corners[2][0] = camera_pos[0] + range * std::cos(-fov_h / 2) * std::cos(-fov_v / 2);
    corners[2][1] = camera_pos[1] + range * std::sin(-fov_h / 2) * std::cos(-fov_v / 2);
    corners[2][2] = camera_pos[2] + range * std::sin(-fov_v / 2);
    // Bottom-left
    corners[3][0] = camera_pos[0] + range * std::cos(fov_h / 2) * std::cos(-fov_v / 2);
    corners[3][1] = camera_pos[1] + range * std::sin(fov_h / 2) * std::cos(-fov_v / 2);
    corners[3][2] = camera_pos[2] + range * std::sin(-fov_v / 2);

    // Draw lines from camera to corners
    for (int i = 0; i < 4; i++) {
        Viewer::drawLine(camera_pos, corners[i], 0.9f, 0.9f, 0.2f);
    }

    // Draw camera view overlay
    drawCameraView();
}

int main(int argc, char** argv) {
    std::cout << "=== 18: Camera Sensor ===" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    createObstacles();
    camera = new DepthCamera(space, 64, 48, M_PI/2, M_PI/3, 10.0);
    
    Viewer viewer(argc, argv, "18: Camera Sensor - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    
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
