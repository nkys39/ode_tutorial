#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;
dBodyID fixed_body, ball_body;
dGeomID fixed_geom, ball_geom;
dJointID ball_joint;

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

void createBallJoint() {
    fixed_body = createSphere(world, space, 0, 0, 2.0, 0.2, 10.0, &fixed_geom);
    dBodySetKinematic(fixed_body);
    ball_body = createBox(world, space, 0, 0, 1.5, 0.3, 0.3, 0.8, 1.0, &ball_geom);
    ball_joint = dJointCreateBall(world, 0);
    dJointAttach(ball_joint, fixed_body, ball_body);
    dJointSetBallAnchor(ball_joint, 0, 0, 2.0);
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
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawSphere(pos1, R1, 0.2);
    
    const dReal* pos2 = dBodyGetPosition(ball_body);
    const dReal* R2 = dBodyGetRotation(ball_body);
    dReal sides[3] = {0.3, 0.3, 0.8};
    Viewer::setColor(0.9f, 0.4f, 0.2f);
    Viewer::drawBox(pos2, R2, sides);
}

int main(int argc, char** argv) {
    std::cout << "=== 09: Ball Joint ===" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    createBallJoint();
    Viewer viewer(argc, argv, "09: Ball Joint - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
