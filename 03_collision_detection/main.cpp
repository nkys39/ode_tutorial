#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dBodyID sphere;
dGeomID ground_geom;
dJointGroupID contact_group;

// Collision callback
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnected(b1, b2)) return;

    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];

    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.5;  // Friction
        contact[i].surface.bounce = 0.8;  // Bounciness
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void reset() {
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

    // Detect collisions
    dSpaceCollide(space, 0, &nearCallback);

    // Step the world
    dWorldStep(world, dt);

    // Remove contact joints
    dJointGroupEmpty(contact_group);
}

// Drawing callback
void drawScene() {
    // Draw ground
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // Draw sphere
    const dReal* pos = dBodyGetPosition(sphere);
    const dReal* R = dBodyGetRotation(sphere);
    dGeomID geom = dBodyGetFirstGeom(sphere);
    dReal radius = dGeomSphereGetRadius(geom);

    Viewer::setColor(0.9f, 0.3f, 0.3f);
    Viewer::drawSphere(pos, R, radius);
}

int main(int argc, char** argv) {
    std::cout << "=== 03: Collision Detection ===" << std::endl;
    std::cout << "A sphere bounces on the ground." << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    // Create world
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create space
    space = dHashSpaceCreate(0);

    // Create contact joint group
    contact_group = dJointGroupCreate(0);

    // Create ground plane
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // Create sphere
    sphere = createSphere(world, space, 0, 0, 3.0, 0.2, 1.0);

    // Create viewer
    Viewer viewer(argc, argv, "03: Collision Detection - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    // Start visualization
    viewer.start();

    // Cleanup
    dBodyDestroy(sphere);
    dGeomDestroy(ground_geom);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
