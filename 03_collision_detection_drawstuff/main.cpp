#include "drawstuff_viewer.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;

dBodyID sphere;
dGeomID sphere_geom;
dGeomID ground_geom;

dReal sphere_radius = 0.3;

// Collision callback
void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    // Get bodies associated with geometries
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    // Skip if bodies are connected
    if (b1 && b2 && dAreConnected(b1, b2)) return;

    // Collision detection
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        // Set contact properties
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.5;            // Friction
        contact[i].surface.bounce = 0.5;        // Bounciness
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        // Create contact joint
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void reset() {
    dBodySetPosition(sphere, 0, 0, 2.0);
    dBodySetLinearVel(sphere, 0, 0, 0);
    dBodySetAngularVel(sphere, 0, 0, 0);
}

void simulationStep(double dt) {
    // Collision detection
    dSpaceCollide(space, 0, &nearCallback);

    // Advance physics simulation
    dWorldStep(world, dt);

    // Clear contact joints
    dJointGroupEmpty(contact_group);
}

void drawScene() {
    // Draw sphere
    const dReal* pos = dBodyGetPosition(sphere);
    const dReal* R = dBodyGetRotation(sphere);

    DrawStuffViewer::setColor(0.3f, 0.7f, 0.9f);
    DrawStuffViewer::drawSphere(pos, R, sphere_radius);

    // Draw ground plane
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
    std::cout << "=== 03: Collision Detection (DrawStuff version) ===" << std::endl;
    std::cout << "A sphere bounces on the ground." << std::endl;
    std::cout << "Press 'R' to reset the sphere position." << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    // Create world
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create collision space
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    // Create ground plane (z = 0)
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // Create sphere
    sphere = dBodyCreate(world);
    dMass mass;
    dMassSetSphere(&mass, 1.0, sphere_radius);
    dBodySetMass(sphere, &mass);

    sphere_geom = dCreateSphere(space, sphere_radius);
    dGeomSetBody(sphere_geom, sphere);

    reset();

    // Create viewer
    DrawStuffViewer viewer;
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setCommandCallback(commandCallback);

    // Start visualization
    viewer.start(argc, argv);

    // Cleanup
    dGeomDestroy(sphere_geom);
    dGeomDestroy(ground_geom);
    dBodyDestroy(sphere);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
