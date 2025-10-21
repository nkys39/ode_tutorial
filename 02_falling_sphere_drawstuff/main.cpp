#include "drawstuff_viewer.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dBodyID sphere;
dReal sphere_radius = 0.3;

void reset() {
    dBodySetPosition(sphere, 0, 0, 2.0);
    dBodySetLinearVel(sphere, 0, 0, 0);
    dBodySetAngularVel(sphere, 0, 0, 0);
}

void simulationStep(double dt) {
    // Check if sphere fell too low
    const dReal* pos = dBodyGetPosition(sphere);
    if (pos[2] < -5.0) {
        reset();
    }

    // Advance physics simulation
    dWorldStep(world, dt);
}

void drawScene() {
    // Draw falling sphere
    const dReal* pos = dBodyGetPosition(sphere);
    const dReal* R = dBodyGetRotation(sphere);

    DrawStuffViewer::setColor(0.3f, 0.7f, 0.9f);
    DrawStuffViewer::drawSphere(pos, R, sphere_radius);

    // Draw ground reference plane at z=0
    dReal ground_pos[3] = {0, 0, 0};
    dReal ground_R[12];
    dRSetIdentity(ground_R);
    dReal ground_size[3] = {5.0, 5.0, 0.01};

    DrawStuffViewer::setColor(0.5f, 0.5f, 0.5f);
    DrawStuffViewer::drawBox(ground_pos, ground_R, ground_size);
}

void commandCallback(int cmd) {
    if (cmd == 'r' || cmd == 'R') {
        reset();
        std::cout << "Reset sphere position" << std::endl;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 02: Falling Sphere (DrawStuff version) ===" << std::endl;
    std::cout << "A sphere falls due to gravity." << std::endl;
    std::cout << "Press 'R' to reset the sphere position." << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    // Create world
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create sphere
    sphere = dBodyCreate(world);
    dMass mass;
    dMassSetSphere(&mass, 1.0, sphere_radius);  // density = 1.0
    dBodySetMass(sphere, &mass);
    reset();

    // Create viewer
    DrawStuffViewer viewer;
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setCommandCallback(commandCallback);

    // Start visualization
    viewer.start(argc, argv);

    // Cleanup
    dBodyDestroy(sphere);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
