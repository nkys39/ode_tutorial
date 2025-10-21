#include "viewer.h"
#include "utils.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

struct Object {
    dBodyID body;
    dGeomID geom;
    int type; // 0=box, 1=sphere
    dReal size[3]; // for box: lx,ly,lz; for sphere: radius,0,0
};

std::vector<Object> objects;

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
        contact[i].surface.bounce = 0.3;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createObjects() {
    Object obj;

    // Create boxes
    for (int i = 0; i < 3; i++) {
        obj.type = 0;
        obj.size[0] = 0.3;
        obj.size[1] = 0.3;
        obj.size[2] = 0.3;
        obj.body = createBox(world, space, -1.5 + i * 0.8, 0, 2.0 + i * 0.5,
                            obj.size[0], obj.size[1], obj.size[2], 1.0, &obj.geom);
        objects.push_back(obj);
    }

    // Create spheres
    for (int i = 0; i < 3; i++) {
        obj.type = 1;
        obj.size[0] = 0.2; // radius
        obj.body = createSphere(world, space, -1.0 + i * 0.8, 1.5, 3.0 + i * 0.5,
                               obj.size[0], 1.0, &obj.geom);
        objects.push_back(obj);
    }
}

void reset() {
    int box_idx = 0;
    int sphere_idx = 0;

    for (auto& obj : objects) {
        if (obj.type == 0) { // box
            dBodySetPosition(obj.body, -1.5 + box_idx * 0.8, 0, 2.0 + box_idx * 0.5);
            box_idx++;
        } else { // sphere
            dBodySetPosition(obj.body, -1.0 + sphere_idx * 0.8, 1.5, 3.0 + sphere_idx * 0.5);
            sphere_idx++;
        }
        dBodySetLinearVel(obj.body, 0, 0, 0);
        dBodySetAngularVel(obj.body, 0, 0, 0);

        dMatrix3 R;
        dRFromAxisAndAngle(R, 0, 0, 1, 0);
        dBodySetRotation(obj.body, R);
    }
}

void simulationStep(double dt) {
    if (Viewer::shouldReset()) {
        reset();
        Viewer::setShouldReset(false);
    }

    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, dt);
    dJointGroupEmpty(contact_group);
}

void drawScene() {
    // Draw ground
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // Draw objects
    for (const auto& obj : objects) {
        const dReal* pos = dBodyGetPosition(obj.body);
        const dReal* R = dBodyGetRotation(obj.body);

        if (obj.type == 0) { // box
            Viewer::setColor(0.8f, 0.4f, 0.2f);
            Viewer::drawBox(pos, R, obj.size);
        } else { // sphere
            Viewer::setColor(0.2f, 0.7f, 0.9f);
            Viewer::drawSphere(pos, R, obj.size[0]);
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 04: Multiple Objects ===" << std::endl;
    std::cout << "Multiple boxes and spheres fall and collide." << std::endl;
    std::cout << std::endl;

    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    createObjects();

    Viewer viewer(argc, argv, "04: Multiple Objects - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    viewer.start();

    // Cleanup
    for (auto& obj : objects) {
        dBodyDestroy(obj.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
