#include "viewer.h"
#include "utils.h"
#include "pedestrian.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

CrowdManager* crowd_manager;

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

void createCrowd() {
    // Create a larger crowd
    for (int i = 0; i < 8; i++) {
        dReal angle = i * 2 * M_PI / 8;
        dReal radius = 3.0;
        dReal x = radius * std::cos(angle);
        dReal y = radius * std::sin(angle);
        
        Pedestrian* ped = crowd_manager->addPedestrian(x, y, 0.8);
        ped->setGoal(-x * 0.8, -y * 0.8);
        ped->setDesiredSpeed(0.8 + (i % 3) * 0.2);
    }
}

void simulationStep(double dt) {
    crowd_manager->update(dt);
    
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            ped->setGoal(-pos[0], -pos[1]);
        }
    }
    
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

void drawScene() {
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);
    
    int ped_num = 0;
    for (auto ped : crowd_manager->getPedestrians()) {
        const dReal* pos = ped->getPosition();
        dGeomID geom = ped->getGeom();
        const dReal* R = dGeomGetRotation(geom);
        dReal radius, length;
        dGeomCapsuleGetParams(geom, &radius, &length);
        
        float hue = (ped_num++ * 0.13f);
        Viewer::setColor(0.8f - hue * 0.3f, 0.3f + hue * 0.5f, 0.9f - hue * 0.4f);
        Viewer::drawCapsule(pos, R, length, radius);
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 21: Crowd Simulation ===" << std::endl;
    std::cout << "Multiple pedestrians interacting using Social Force Model" << std::endl;
    
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    crowd_manager = new CrowdManager(world, space);
    createCrowd();
    
    Viewer viewer(argc, argv, "21: Crowd Simulation - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    
    delete crowd_manager;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
