#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;
dBodyID base_body, wheel_body;
dGeomID base_geom, wheel_geom;
dJointID motor_joint;

void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.8;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createMotor() {
    base_body = createBox(world, space, 0, 0, 0.3, 0.5, 0.5, 0.2, 5.0, &base_geom);
    wheel_body = createCylinder(world, space, 0.3, 0, 0.3, 0.2, 0.1, 0.5, &wheel_geom);
    
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, M_PI / 2);
    dBodySetRotation(wheel_body, R);
    
    motor_joint = dJointCreateHinge(world, 0);
    dJointAttach(motor_joint, base_body, wheel_body);
    dJointSetHingeAnchor(motor_joint, 0.3, 0, 0.3);
    dJointSetHingeAxis(motor_joint, 0, 1, 0);
    
    // Set motor parameters
    dJointSetHingeParam(motor_joint, dParamVel, 10.0);  // Angular velocity
    dJointSetHingeParam(motor_joint, dParamFMax, 5.0);  // Max torque
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
    
    const dReal* pos1 = dBodyGetPosition(base_body);
    const dReal* R1 = dBodyGetRotation(base_body);
    dReal sides[3] = {0.5, 0.5, 0.2};
    Viewer::setColor(0.3f, 0.3f, 0.7f);
    Viewer::drawBox(pos1, R1, sides);
    
    const dReal* pos2 = dBodyGetPosition(wheel_body);
    const dReal* R2 = dBodyGetRotation(wheel_body);
    Viewer::setColor(0.7f, 0.3f, 0.3f);
    Viewer::drawCylinder(pos2, R2, 0.1, 0.2);
}

int main(int argc, char** argv) {
    std::cout << "=== 10: Motor ===" << std::endl;
    std::cout << "Motor-controlled rotating wheel" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    createMotor();
    Viewer viewer(argc, argv, "10: Motor - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
