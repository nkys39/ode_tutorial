#include "viewer.h"
#include "utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// Compound object: box with two spheres attached
dBodyID compound_body;
dGeomID box_geom, sphere1_geom, sphere2_geom;

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

void createCompoundObject() {
    compound_body = dBodyCreate(world);
    dBodySetPosition(compound_body, 0, 0, 2.0);
    
    dMass m, m_total;
    dMassSetZero(&m_total);
    
    // Central box
    dMassSetBox(&m, 1, 0.5, 0.3, 0.2);
    dMassAdd(&m_total, &m);
    box_geom = dCreateBox(space, 0.5, 0.3, 0.2);
    dGeomSetBody(box_geom, compound_body);
    
    // Left sphere
    dMassSetSphere(&m, 1, 0.15);
    dMassTranslate(&m, -0.4, 0, 0);
    dMassAdd(&m_total, &m);
    sphere1_geom = dCreateSphere(space, 0.15);
    dGeomSetBody(sphere1_geom, compound_body);
    dGeomSetOffsetPosition(sphere1_geom, -0.4, 0, 0);
    
    // Right sphere
    dMassSetSphere(&m, 1, 0.15);
    dMassTranslate(&m, 0.4, 0, 0);
    dMassAdd(&m_total, &m);
    sphere2_geom = dCreateSphere(space, 0.15);
    dGeomSetBody(sphere2_geom, compound_body);
    dGeomSetOffsetPosition(sphere2_geom, 0.4, 0, 0);
    
    dBodySetMass(compound_body, &m_total);
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
    
    const dReal* pos = dBodyGetPosition(compound_body);
    const dReal* R = dBodyGetRotation(compound_body);
    dReal box_sides[3] = {0.5, 0.3, 0.2};
    
    Viewer::setColor(0.7f, 0.4f, 0.2f);
    Viewer::drawBox(pos, R, box_sides);
    
    // Draw spheres at offset positions
    dVector3 sphere_pos;
    dReal offset1[3] = {-0.4, 0, 0};
    dReal offset2[3] = {0.4, 0, 0};
    
    // Transform offsets by rotation
    for (int i = 0; i < 3; i++) {
        sphere_pos[i] = pos[i] + R[i*4] * offset1[0] + R[i*4+1] * offset1[1] + R[i*4+2] * offset1[2];
    }
    Viewer::setColor(0.3f, 0.7f, 0.9f);
    Viewer::drawSphere(sphere_pos, R, 0.15);
    
    for (int i = 0; i < 3; i++) {
        sphere_pos[i] = pos[i] + R[i*4] * offset2[0] + R[i*4+1] * offset2[1] + R[i*4+2] * offset2[2];
    }
    Viewer::drawSphere(sphere_pos, R, 0.15);
}

int main(int argc, char** argv) {
    std::cout << "=== 06: Compound Objects ===" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    createCompoundObject();
    
    Viewer viewer(argc, argv, "06: Compound Objects - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    
    dBodyDestroy(compound_body);
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
