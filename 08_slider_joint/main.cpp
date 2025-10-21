#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;
dBodyID fixed_body, slider_body;
dGeomID fixed_geom, slider_geom;
dJointID slider_joint;

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

void createSlider() {
    fixed_body = createBox(world, space, 0, 0, 1.5, 0.3, 0.3, 0.3, 10.0, &fixed_geom);
    dBodySetKinematic(fixed_body);
    slider_body = createBox(world, space, 0, 0, 1.0, 0.2, 0.2, 0.4, 1.0, &slider_geom);
    slider_joint = dJointCreateSlider(world, 0);
    dJointAttach(slider_joint, fixed_body, slider_body);
    dJointSetSliderAxis(slider_joint, 0, 0, 1);  // Slide along Z-axis
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
    
    const dReal* pos2 = dBodyGetPosition(slider_body);
    const dReal* R2 = dBodyGetRotation(slider_body);
    dReal sides2[3] = {0.2, 0.2, 0.4};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos2, R2, sides2);
}

int main(int argc, char** argv) {
    std::cout << "=== 08: Slider Joint ===" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    createSlider();
    Viewer viewer(argc, argv, "08: Slider Joint - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
