#include "viewer.h"
#include "utils.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

struct Shape {
    dBodyID body;
    dGeomID geom;
    int type; // 0=box, 1=sphere, 2=capsule, 3=cylinder
    dReal params[3];
};

std::vector<Shape> shapes;

void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;

    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 0.7;
        contact[i].surface.bounce = 0.2;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createShapes() {
    Shape shape;

    // Box
    shape.type = 0;
    shape.params[0] = 0.4; shape.params[1] = 0.3; shape.params[2] = 0.2;
    shape.body = createBox(world, space, -1.5, 0, 2.0,
                          shape.params[0], shape.params[1], shape.params[2], 1.0, &shape.geom);
    shapes.push_back(shape);

    // Sphere
    shape.type = 1;
    shape.params[0] = 0.25;
    shape.body = createSphere(world, space, -0.5, 0, 2.5, shape.params[0], 1.0, &shape.geom);
    shapes.push_back(shape);

    // Capsule
    shape.type = 2;
    shape.params[0] = 0.15; // radius
    shape.params[1] = 0.5;  // length
    shape.body = createCapsule(world, space, 0.5, 0, 3.0,
                              shape.params[0], shape.params[1], 1.0, &shape.geom);
    shapes.push_back(shape);

    // Cylinder
    shape.type = 3;
    shape.params[0] = 0.2; // radius
    shape.params[1] = 0.4; // length
    shape.body = createCylinder(world, space, 1.5, 0, 2.0,
                               shape.params[0], shape.params[1], 1.0, &shape.geom);
    shapes.push_back(shape);
}

void reset() {
    if (!shapes.empty()) dBodySetPosition(shapes[0].body, -1.5, 0, 2.0);
    if (shapes.size() > 1) dBodySetPosition(shapes[1].body, -0.5, 0, 2.5);
    if (shapes.size() > 2) dBodySetPosition(shapes[2].body, 0.5, 0, 3.0);
    if (shapes.size() > 3) dBodySetPosition(shapes[3].body, 1.5, 0, 2.0);

    for (auto& shape : shapes) {
        dBodySetLinearVel(shape.body, 0, 0, 0);
        dBodySetAngularVel(shape.body, 0, 0, 0);
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
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    for (const auto& shape : shapes) {
        const dReal* pos = dBodyGetPosition(shape.body);
        const dReal* R = dBodyGetRotation(shape.body);

        switch (shape.type) {
            case 0: // Box
                Viewer::setColor(0.9f, 0.3f, 0.3f);
                Viewer::drawBox(pos, R, shape.params);
                break;
            case 1: // Sphere
                Viewer::setColor(0.3f, 0.9f, 0.3f);
                Viewer::drawSphere(pos, R, shape.params[0]);
                break;
            case 2: // Capsule
                Viewer::setColor(0.3f, 0.3f, 0.9f);
                Viewer::drawCapsule(pos, R, shape.params[1], shape.params[0]);
                break;
            case 3: // Cylinder
                Viewer::setColor(0.9f, 0.9f, 0.3f);
                Viewer::drawCylinder(pos, R, shape.params[1], shape.params[0]);
                break;
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 05: Different Shapes ===" << std::endl;
    std::cout << "Various geometric shapes: box, sphere, capsule, cylinder" << std::endl;
    std::cout << std::endl;

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    createShapes();

    Viewer viewer(argc, argv, "05: Different Shapes - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();

    for (auto& shape : shapes) {
        dBodyDestroy(shape.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
