#include "viewer.h"
#include "utils.h"
#include "pedestrian.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// Pedestrians
CrowdManager* crowd_manager;

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
        contact[i].surface.mu = 0.5;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createPedestrians() {
    // Create several pedestrians with different goals
    Pedestrian* p1 = crowd_manager->addPedestrian(-3, -2, 0.8);
    p1->setGoal(3, 2);
    p1->setDesiredSpeed(1.2);

    Pedestrian* p2 = crowd_manager->addPedestrian(3, 2, 0.8);
    p2->setGoal(-3, -2);
    p2->setDesiredSpeed(1.0);

    Pedestrian* p3 = crowd_manager->addPedestrian(-2, 2, 0.8);
    p3->setGoal(2, -2);
    p3->setDesiredSpeed(1.1);

    Pedestrian* p4 = crowd_manager->addPedestrian(2, -2, 0.8);
    p4->setGoal(-2, 2);
    p4->setDesiredSpeed(0.9);

    std::cout << "Created " << crowd_manager->getCount() << " pedestrians" << std::endl;
}

void simulationStep(double dt) {
    // Update pedestrian behaviors
    crowd_manager->update(dt);

    // Reset pedestrians when they reach their goals
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            // Reverse goal
            dReal new_goal_x = -pos[0];
            dReal new_goal_y = -pos[1];
            ped->setGoal(new_goal_x, new_goal_y);
            std::cout << "Pedestrian reached goal, heading back!" << std::endl;
        }
    }

    // Collision detection
    dSpaceCollide(space, 0, &nearCallback);

    // Step simulation
    dWorldStep(world, dt);

    // Clear contacts
    dJointGroupEmpty(contact_group);
}

void drawScene() {
    // Draw ground
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // Draw pedestrians
    int ped_num = 0;
    for (auto ped : crowd_manager->getPedestrians()) {
        const dReal* pos = ped->getPosition();
        const dReal* vel = ped->getVelocity();

        dGeomID geom = ped->getGeom();
        const dReal* R = dGeomGetRotation(geom);
        dReal radius, length;
        dGeomCapsuleGetParams(geom, &radius, &length);

        // Color based on pedestrian number
        float hue = (ped_num++ * 0.25f);
        Viewer::setColor(0.8f - hue * 0.3f, 0.3f + hue * 0.5f, 0.9f - hue * 0.4f);
        Viewer::drawCapsule(pos, R, length, radius);

        // Draw velocity vector
        dReal vel_end[3] = {
            pos[0] + vel[0] * 0.5,
            pos[1] + vel[1] * 0.5,
            pos[2]
        };
        Viewer::drawLine(pos, vel_end, 1.0f, 1.0f, 0.0f);
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 20: Walking Pedestrian Simulation ===" << std::endl;
    std::cout << "Simulates pedestrians as moving obstacles." << std::endl;
    std::cout << "Pedestrians use Social Force Model for natural movement." << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    // Create ground
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // Create crowd manager
    crowd_manager = new CrowdManager(world, space);

    // Create pedestrians
    createPedestrians();

    // Create viewer
    Viewer viewer(argc, argv, "20: Walking Pedestrian - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    Camera& cam = Viewer::getCamera();
    cam.distance = 10.0;
    cam.pitch = 60.0;

    viewer.start();

    // Cleanup
    delete crowd_manager;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
