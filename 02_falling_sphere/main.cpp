#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dBodyID sphere;
dGeomID sphere_geom;

void reset() {
    // Reset sphere position
    dBodySetPosition(sphere, 0, 0, 3.0);
    dBodySetLinearVel(sphere, 0, 0, 0);
    dBodySetAngularVel(sphere, 0, 0, 0);
}

// Simulation callback
void simulationStep(double dt) {
    if (Viewer::shouldReset()) {
        reset();
        Viewer::setShouldReset(false);
    }

    // Step the world
    dWorldStep(world, dt);

    // Print position
    const dReal* pos = dBodyGetPosition(sphere);
    if ((int)(pos[2] * 100) % 50 == 0) {  // Print every 0.5m
        std::cout << "Sphere height: " << pos[2] << " m" << std::endl;
    }
}

// Drawing callback
void drawScene() {
    // Draw sphere
    const dReal* pos = dBodyGetPosition(sphere);
    const dReal* R = dBodyGetRotation(sphere);
    dReal radius = dGeomSphereGetRadius(sphere_geom);

    Viewer::setColor(0.2f, 0.6f, 0.9f);
    Viewer::drawSphere(pos, R, radius);
}

int main(int argc, char** argv) {
    std::cout << "=== 02: Falling Sphere ===" << std::endl;
    std::cout << "A sphere falls under gravity." << std::endl;
    std::cout << "Watch the sphere fall!" << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    // Create world
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create space
    space = dHashSpaceCreate(0);

    // Create sphere
    sphere = createSphere(world, space, 0, 0, 3.0, 0.2, 1.0, &sphere_geom);

    // Create viewer
    Viewer viewer(argc, argv, "02: Falling Sphere - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    // Start visualization
    viewer.start();

    // Cleanup
    dBodyDestroy(sphere);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
