#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

std::vector<dBodyID> bodies;
std::vector<dGeomID> geoms;

void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.5;
        contact[i].surface.bounce = 0.3;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void create() {
    // Simple demonstration object
    dGeomID geom;
    dBodyID body = createBox(world, space, 0, 0, 2.0, 0.5, 0.5, 0.5, 1.0, &geom);
    bodies.push_back(body);
    geoms.push_back(geom);
}

void simulationStep(double dt) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

void drawScene() {
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);
    
    for (size_t i = 0; i < bodies.size(); i++) {
        const dReal* pos = dBodyGetPosition(bodies[i]);
        const dReal* R = dBodyGetRotation(bodies[i]);
        dReal sides[3] = {0.5, 0.5, 0.5};
        Viewer::setColor(0.7f, 0.4f, 0.3f);
        Viewer::drawBox(pos, R, sides);
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 12_vehicle: Vehicle simulation ===" << std::endl;
    std::cout << "This is a simplified implementation." << std::endl;
    std::cout << "Full implementation can be added later." << std::endl;
    
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    create();
    
    Viewer viewer(argc, argv, "12_vehicle - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    
    for (auto body : bodies) dBodyDestroy(body);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
