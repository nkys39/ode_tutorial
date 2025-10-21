#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

dBodyID fixed_body, hinge_body;
dGeomID fixed_geom, hinge_geom;
dJointID hinge_joint;

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
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createHinge() {
    // Fixed base
    fixed_body = createBox(world, space, 0, 0, 1.0, 0.3, 0.3, 0.3, 10.0, &fixed_geom);
    dBodySetKinematic(fixed_body);  // Make it immovable
    
    // Swinging body
    hinge_body = createBox(world, space, 0, 0.5, 1.0, 0.2, 1.0, 0.1, 1.0, &hinge_geom);
    
    // Create hinge joint
    hinge_joint = dJointCreateHinge(world, 0);
    dJointAttach(hinge_joint, fixed_body, hinge_body);
    dJointSetHingeAnchor(hinge_joint, 0, 0, 1.0);
    dJointSetHingeAxis(hinge_joint, 1, 0, 0);  // Rotate around X-axis
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
    
    const dReal* pos1 = dBodyGetPosition(fixed_body);
    const dReal* R1 = dBodyGetRotation(fixed_body);
    dReal sides1[3] = {0.3, 0.3, 0.3};
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawBox(pos1, R1, sides1);
    
    const dReal* pos2 = dBodyGetPosition(hinge_body);
    const dReal* R2 = dBodyGetRotation(hinge_body);
    dReal sides2[3] = {0.2, 1.0, 0.1};
    Viewer::setColor(0.7f, 0.3f, 0.9f);
    Viewer::drawBox(pos2, R2, sides2);
}

int main(int argc, char** argv) {
    std::cout << "=== 07: Hinge Joint ===" << std::endl;
    std::cout << "A swinging door/pendulum using hinge joint" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    createHinge();
    
    Viewer viewer(argc, argv, "07: Hinge Joint - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
