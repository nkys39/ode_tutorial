#include "drawstuff_viewer.h"
#include <sys/time.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#endif

namespace ode_tutorial {

// Static member initialization
std::function<void(double)> DrawStuffViewer::simCallback;
std::function<void()> DrawStuffViewer::drawCallback;
std::function<void(int)> DrawStuffViewer::commandCallback;
double DrawStuffViewer::lastTime = 0.0;

static double getTime() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void DrawStuffViewer::startCallback() {
    // Set viewpoint
    float xyz[3] = {2.0f, -2.0f, 1.5f};
    float hpr[3] = {45.0f, -20.0f, 0.0f};
    dsSetViewpoint(xyz, hpr);

    lastTime = getTime();
}

void DrawStuffViewer::stepCallback(int pause) {
    if (pause) return;

    double currentTime = getTime();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Call simulation callback
    if (simCallback) {
        simCallback(dt);
    }

    // Call draw callback
    if (drawCallback) {
        drawCallback();
    }
}

void DrawStuffViewer::commandCallback_static(int cmd) {
    if (commandCallback) {
        commandCallback(cmd);
    }
}

void DrawStuffViewer::start(int argc, char** argv) {
    // Setup drawstuff functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &DrawStuffViewer::startCallback;
    fn.step = &DrawStuffViewer::stepCallback;
    fn.command = &DrawStuffViewer::commandCallback_static;
    fn.stop = nullptr;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    // Start simulation loop
    dsSimulationLoop(argc, argv, 800, 600, &fn);
}

void DrawStuffViewer::setColor(float r, float g, float b, float alpha) {
    dsSetColor(r, g, b);
}

void DrawStuffViewer::drawBox(const dReal* pos, const dReal* R, const dReal sides[3]) {
    dsDrawBox(pos, R, sides);
}

void DrawStuffViewer::drawSphere(const dReal* pos, const dReal* R, dReal radius) {
    dsDrawSphere(pos, R, radius);
}

void DrawStuffViewer::drawCapsule(const dReal* pos, const dReal* R, dReal length, dReal radius) {
    dsDrawCapsule(pos, R, length, radius);
}

void DrawStuffViewer::drawCylinder(const dReal* pos, const dReal* R, dReal length, dReal radius) {
    dsDrawCylinder(pos, R, length, radius);
}

} // namespace ode_tutorial
