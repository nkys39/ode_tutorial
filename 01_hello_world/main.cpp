#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;

// Simulation callback
void simulationStep(double dt) {
    // Step the world
    dWorldStep(world, dt);
}

// Drawing callback
void drawScene() {
    // Nothing to draw yet - just the grid and axes
    std::cout << "Hello, ODE World!" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "=== 01: Hello World ===" << std::endl;
    std::cout << "This tutorial demonstrates basic ODE initialization." << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Mouse drag: Rotate camera" << std::endl;
    std::cout << "  Mouse wheel: Zoom" << std::endl;
    std::cout << "  Space: Pause/Resume" << std::endl;
    std::cout << "  R: Reset" << std::endl;
    std::cout << "  Q/ESC: Quit" << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    // Create world
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create space for collision detection
    space = dHashSpaceCreate(0);

    // Create viewer
    Viewer viewer(argc, argv, "01: Hello World - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    // Start visualization
    viewer.start();

    // Cleanup
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
