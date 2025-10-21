#include "drawstuff_viewer.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world
dWorldID world;

void simulationStep(double dt) {
    // Advance physics simulation
    dWorldStep(world, dt);
}

void drawScene() {
    // Draw a simple box at the origin
    dReal pos[3] = {0, 0, 0.5};
    dReal R[12];
    dRSetIdentity(R);
    dReal sides[3] = {1.0, 1.0, 1.0};

    DrawStuffViewer::setColor(0.8f, 0.3f, 0.3f);
    DrawStuffViewer::drawBox(pos, R, sides);
}

int main(int argc, char** argv) {
    std::cout << "=== 01: Hello World (DrawStuff version) ===" << std::endl;
    std::cout << "A simple demonstration using DrawStuff for visualization." << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    // Create world
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create viewer
    DrawStuffViewer viewer;
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    // Start visualization
    viewer.start(argc, argv);

    // Cleanup
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
