#include "viewer.h"
#include <cmath>
#include <iostream>
#include <sys/time.h>

namespace ode_tutorial {

// Static member initialization
std::function<void(double)> Viewer::simCallback;
std::function<void()> Viewer::drawCallback;
std::function<void(unsigned char, int, int)> Viewer::keyCallback;
Camera Viewer::camera;
int Viewer::mouseButton = -1;
int Viewer::mouseX = 0;
int Viewer::mouseY = 0;
bool Viewer::paused = false;
bool Viewer::resetFlag = false;
double Viewer::lastTime = 0.0;
int Viewer::windowWidth = 800;
int Viewer::windowHeight = 600;

Viewer::Viewer(int argc, char** argv, const char* title) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow(title);

    // OpenGL settings
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    GLfloat light_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat light_diffuse[] = {0.7f, 0.7f, 0.7f, 1.0f};
    GLfloat light_position[] = {2.0f, 4.0f, 5.0f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    // Register callbacks
    glutDisplayFunc(displayCallback);
    glutReshapeFunc(reshapeCallback);
    glutIdleFunc(idleCallback);
    glutMouseFunc(mouseCallback);
    glutMotionFunc(motionCallback);
    glutKeyboardFunc(keyboardCallbackInternal);
    glutSpecialFunc(specialKeyCallback);

    // Initialize time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    lastTime = tv.tv_sec + tv.tv_usec / 1000000.0;
}

Viewer::~Viewer() {
}

void Viewer::setSimulationCallback(std::function<void(double)> callback) {
    simCallback = callback;
}

void Viewer::setDrawCallback(std::function<void()> callback) {
    drawCallback = callback;
}

void Viewer::setKeyboardCallback(std::function<void(unsigned char, int, int)> callback) {
    keyCallback = callback;
}

void Viewer::start() {
    glutMainLoop();
}

Camera& Viewer::getCamera() {
    return camera;
}

void Viewer::displayCallback() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.9f, 0.9f, 0.95f, 1.0f);

    // Set up camera
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)windowWidth / windowHeight, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera position
    float eye_x = camera.target_x + camera.distance * cos(camera.pitch * M_PI / 180.0) * cos(camera.yaw * M_PI / 180.0);
    float eye_y = camera.target_y + camera.distance * cos(camera.pitch * M_PI / 180.0) * sin(camera.yaw * M_PI / 180.0);
    float eye_z = camera.target_z + camera.distance * sin(camera.pitch * M_PI / 180.0);

    gluLookAt(eye_x, eye_y, eye_z,
              camera.target_x, camera.target_y, camera.target_z,
              0, 0, 1);

    // Draw grid and axes
    drawGrid(10.0f, 20);
    drawAxes(1.0f);

    // User drawing callback
    if (drawCallback) {
        drawCallback();
    }

    glutSwapBuffers();
}

void Viewer::reshapeCallback(int width, int height) {
    windowWidth = width;
    windowHeight = height;
    glViewport(0, 0, width, height);
}

void Viewer::idleCallback() {
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double currentTime = tv.tv_sec + tv.tv_usec / 1000000.0;
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Limit dt to avoid instability
    if (dt > 0.05) dt = 0.05;

    // Simulation callback
    if (simCallback && !paused) {
        simCallback(dt);
    }

    glutPostRedisplay();
}

void Viewer::mouseCallback(int button, int state, int x, int y) {
    // Mouse wheel support (button 3 = scroll up, button 4 = scroll down)
    if (button == 3 && state == GLUT_DOWN) {
        // Scroll up = zoom in
        camera.distance -= 0.3f;
        if (camera.distance < 0.5f) camera.distance = 0.5f;
        return;
    }
    if (button == 4 && state == GLUT_DOWN) {
        // Scroll down = zoom out
        camera.distance += 0.3f;
        if (camera.distance > 50.0f) camera.distance = 50.0f;
        return;
    }

    if (state == GLUT_DOWN) {
        mouseButton = button;
        mouseX = x;
        mouseY = y;
    } else {
        mouseButton = -1;
    }
}

void Viewer::motionCallback(int x, int y) {
    int dx = x - mouseX;
    int dy = y - mouseY;
    mouseX = x;
    mouseY = y;

    if (mouseButton == GLUT_LEFT_BUTTON) {
        camera.yaw += dx * 0.5f;
        camera.pitch += dy * 0.5f;
        if (camera.pitch > 89.0f) camera.pitch = 89.0f;
        if (camera.pitch < -89.0f) camera.pitch = -89.0f;
    } else if (mouseButton == GLUT_RIGHT_BUTTON) {
        camera.distance += dy * 0.02f;
        if (camera.distance < 0.5f) camera.distance = 0.5f;
        if (camera.distance > 50.0f) camera.distance = 50.0f;
    }
}

void Viewer::keyboardCallbackInternal(unsigned char key, int x, int y) {
    switch (key) {
        case ' ':
            togglePause();
            break;
        case 'r':
        case 'R':
            resetFlag = true;
            break;
        case 'q':
        case 'Q':
        case 27: // ESC
            exit(0);
            break;
        // Zoom controls (trackpad-friendly)
        case '+':
        case '=':
        case 'z':
        case 'Z':
            camera.distance -= 0.3f;
            if (camera.distance < 0.5f) camera.distance = 0.5f;
            break;
        case '-':
        case '_':
        case 'x':
        case 'X':
            camera.distance += 0.3f;
            if (camera.distance > 50.0f) camera.distance = 50.0f;
            break;
        // Pan controls (trackpad-friendly)
        case 'i':
        case 'I':
            camera.target_y += 0.2f;
            break;
        case 'k':
        case 'K':
            camera.target_y -= 0.2f;
            break;
        case 'j':
        case 'J':
            camera.target_x -= 0.2f;
            break;
        case 'l':
        case 'L':
            camera.target_x += 0.2f;
            break;
        case 'u':
        case 'U':
            camera.target_z += 0.2f;
            break;
        case 'o':
        case 'O':
            camera.target_z -= 0.2f;
            break;
        default:
            if (keyCallback) {
                keyCallback(key, x, y);
            }
            break;
    }
}

void Viewer::specialKeyCallback(int key, int x, int y) {
    // Arrow keys for camera pan (trackpad-friendly alternative)
    switch (key) {
        case GLUT_KEY_UP:
            camera.target_y += 0.2f;
            break;
        case GLUT_KEY_DOWN:
            camera.target_y -= 0.2f;
            break;
        case GLUT_KEY_LEFT:
            camera.target_x -= 0.2f;
            break;
        case GLUT_KEY_RIGHT:
            camera.target_x += 0.2f;
            break;
        case GLUT_KEY_PAGE_UP:
            camera.target_z += 0.2f;
            break;
        case GLUT_KEY_PAGE_DOWN:
            camera.target_z -= 0.2f;
            break;
    }
}

void Viewer::drawBox(const dReal* pos, const dReal* R, const dReal sides[3]) {
    glPushMatrix();
    glTranslated(pos[0], pos[1], pos[2]);

    GLfloat matrix[16];
    matrix[0] = R[0]; matrix[1] = R[4]; matrix[2] = R[8];  matrix[3] = 0;
    matrix[4] = R[1]; matrix[5] = R[5]; matrix[6] = R[9];  matrix[7] = 0;
    matrix[8] = R[2]; matrix[9] = R[6]; matrix[10] = R[10]; matrix[11] = 0;
    matrix[12] = 0;   matrix[13] = 0;   matrix[14] = 0;    matrix[15] = 1;
    glMultMatrixf(matrix);

    glScaled(sides[0], sides[1], sides[2]);
    glutSolidCube(1.0);
    glPopMatrix();
}

void Viewer::drawSphere(const dReal* pos, const dReal* R, dReal radius) {
    glPushMatrix();
    glTranslated(pos[0], pos[1], pos[2]);
    glutSolidSphere(radius, 20, 20);
    glPopMatrix();
}

void Viewer::drawCapsule(const dReal* pos, const dReal* R, dReal length, dReal radius) {
    glPushMatrix();
    glTranslated(pos[0], pos[1], pos[2]);

    GLfloat matrix[16];
    matrix[0] = R[0]; matrix[1] = R[4]; matrix[2] = R[8];  matrix[3] = 0;
    matrix[4] = R[1]; matrix[5] = R[5]; matrix[6] = R[9];  matrix[7] = 0;
    matrix[8] = R[2]; matrix[9] = R[6]; matrix[10] = R[10]; matrix[11] = 0;
    matrix[12] = 0;   matrix[13] = 0;   matrix[14] = 0;    matrix[15] = 1;
    glMultMatrixf(matrix);

    // Draw cylinder
    glPushMatrix();
    glRotated(90, 1, 0, 0);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, radius, radius, length, 20, 1);
    gluDeleteQuadric(quad);
    glPopMatrix();

    // Draw top sphere
    glPushMatrix();
    glTranslated(0, 0, length / 2);
    glutSolidSphere(radius, 20, 20);
    glPopMatrix();

    // Draw bottom sphere
    glPushMatrix();
    glTranslated(0, 0, -length / 2);
    glutSolidSphere(radius, 20, 20);
    glPopMatrix();

    glPopMatrix();
}

void Viewer::drawCylinder(const dReal* pos, const dReal* R, dReal length, dReal radius) {
    glPushMatrix();
    glTranslated(pos[0], pos[1], pos[2]);

    GLfloat matrix[16];
    matrix[0] = R[0]; matrix[1] = R[4]; matrix[2] = R[8];  matrix[3] = 0;
    matrix[4] = R[1]; matrix[5] = R[5]; matrix[6] = R[9];  matrix[7] = 0;
    matrix[8] = R[2]; matrix[9] = R[6]; matrix[10] = R[10]; matrix[11] = 0;
    matrix[12] = 0;   matrix[13] = 0;   matrix[14] = 0;    matrix[15] = 1;
    glMultMatrixf(matrix);

    glRotated(90, 1, 0, 0);
    GLUquadric* quad = gluNewQuadric();
    gluCylinder(quad, radius, radius, length, 20, 1);
    gluDeleteQuadric(quad);

    glPopMatrix();
}

void Viewer::drawPlane(const dReal* normal, dReal distance, dReal size) {
    glDisable(GL_LIGHTING);
    glColor3f(0.6f, 0.6f, 0.6f);

    // Calculate a point on the plane
    dReal point[3];
    point[0] = normal[0] * distance;
    point[1] = normal[1] * distance;
    point[2] = normal[2] * distance;

    // Create two perpendicular vectors to the normal for drawing the plane
    dReal v1[3], v2[3];

    // Find a vector not parallel to normal
    if (std::abs(normal[0]) < 0.9) {
        v1[0] = 1; v1[1] = 0; v1[2] = 0;
    } else {
        v1[0] = 0; v1[1] = 1; v1[2] = 0;
    }

    // v2 = normal × v1
    v2[0] = normal[1] * v1[2] - normal[2] * v1[1];
    v2[1] = normal[2] * v1[0] - normal[0] * v1[2];
    v2[2] = normal[0] * v1[1] - normal[1] * v1[0];

    // Normalize v2
    dReal len = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]);
    v2[0] /= len; v2[1] /= len; v2[2] /= len;

    // v1 = v2 × normal
    v1[0] = v2[1] * normal[2] - v2[2] * normal[1];
    v1[1] = v2[2] * normal[0] - v2[0] * normal[2];
    v1[2] = v2[0] * normal[1] - v2[1] * normal[0];

    // Draw grid pattern for better visibility
    glBegin(GL_LINES);
    int divisions = 20;
    for (int i = -divisions; i <= divisions; i++) {
        dReal t = i * size / divisions;

        // Lines in v1 direction
        dReal start1[3] = {
            point[0] + t * v2[0] - size * v1[0],
            point[1] + t * v2[1] - size * v1[1],
            point[2] + t * v2[2] - size * v1[2]
        };
        dReal end1[3] = {
            point[0] + t * v2[0] + size * v1[0],
            point[1] + t * v2[1] + size * v1[1],
            point[2] + t * v2[2] + size * v1[2]
        };
        glVertex3d(start1[0], start1[1], start1[2]);
        glVertex3d(end1[0], end1[1], end1[2]);

        // Lines in v2 direction
        dReal start2[3] = {
            point[0] + t * v1[0] - size * v2[0],
            point[1] + t * v1[1] - size * v2[1],
            point[2] + t * v1[2] - size * v2[2]
        };
        dReal end2[3] = {
            point[0] + t * v1[0] + size * v2[0],
            point[1] + t * v1[1] + size * v2[1],
            point[2] + t * v1[2] + size * v2[2]
        };
        glVertex3d(start2[0], start2[1], start2[2]);
        glVertex3d(end2[0], end2[1], end2[2]);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

void Viewer::drawLine(const dReal* start, const dReal* end, float r, float g, float b) {
    glDisable(GL_LIGHTING);
    glColor3f(r, g, b);
    glLineWidth(2.0f);

    glBegin(GL_LINES);
    glVertex3d(start[0], start[1], start[2]);
    glVertex3d(end[0], end[1], end[2]);
    glEnd();

    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Viewer::drawGrid(float size, int divisions) {
    glDisable(GL_LIGHTING);
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(1.0f);

    float step = size / divisions;
    glBegin(GL_LINES);
    for (int i = -divisions; i <= divisions; i++) {
        glVertex3f(-size, i * step, 0);
        glVertex3f(size, i * step, 0);
        glVertex3f(i * step, -size, 0);
        glVertex3f(i * step, size, 0);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

void Viewer::drawAxes(float size) {
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);

    // X axis (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(size, 0, 0);
    glEnd();

    // Y axis (green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, size, 0);
    glEnd();

    // Z axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, size);
    glEnd();

    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Viewer::setColor(float r, float g, float b, float alpha) {
    GLfloat color[4] = {r, g, b, alpha};
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
}

void Viewer::drawGeometry(dGeomID geom) {
    if (!geom) return;

    const dReal* pos = dGeomGetPosition(geom);
    const dReal* R = dGeomGetRotation(geom);

    int type = dGeomGetClass(geom);

    switch (type) {
        case dSphereClass: {
            dReal radius = dGeomSphereGetRadius(geom);
            drawSphere(pos, R, radius);
            break;
        }
        case dBoxClass: {
            dVector3 sides;
            dGeomBoxGetLengths(geom, sides);
            drawBox(pos, R, sides);
            break;
        }
        case dCapsuleClass: {
            dReal radius, length;
            dGeomCapsuleGetParams(geom, &radius, &length);
            drawCapsule(pos, R, length, radius);
            break;
        }
        case dCylinderClass: {
            dReal radius, length;
            dGeomCylinderGetParams(geom, &radius, &length);
            drawCylinder(pos, R, length, radius);
            break;
        }
        case dPlaneClass: {
            dVector4 plane;
            dGeomPlaneGetParams(geom, plane);
            drawPlane(plane, plane[3], 10.0);
            break;
        }
    }
}

void Viewer::drawBody(dBodyID body) {
    // This function is deprecated - use drawGeometry directly with geom IDs
    // ODE doesn't provide dBodyGetFirstGeom/dBodyGetNextGeom APIs
    // Instead, keep track of geom IDs in your application
    if (!body) return;
}

void Viewer::pause() {
    paused = true;
    std::cout << "Simulation paused" << std::endl;
}

void Viewer::resume() {
    paused = false;
    std::cout << "Simulation resumed" << std::endl;
}

void Viewer::togglePause() {
    paused = !paused;
    if (paused) {
        std::cout << "Simulation paused" << std::endl;
    } else {
        std::cout << "Simulation resumed" << std::endl;
    }
}

bool Viewer::isPaused() {
    return paused;
}

void Viewer::reset() {
    resetFlag = true;
}

void Viewer::setShouldReset(bool value) {
    resetFlag = value;
}

bool Viewer::shouldReset() {
    return resetFlag;
}

} // namespace ode_tutorial
