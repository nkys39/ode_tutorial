#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dGeomID ground_geom;

DepthCamera* camera;
dReal camera_pos[3] = {0, -3, 1.5};

std::vector<dBodyID> obstacles;

void createObstacles() {
    dGeomID geom;
    obstacles.push_back(createBox(world, space, 1, 0, 0.5, 1.0, 0.5, 1.0, 1.0, &geom));
    obstacles.push_back(createSphere(world, space, -1, 1, 0.5, 0.5, 1.0, &geom));
}

void simulationStep(double dt) {
    dMatrix3 R;
    dRSetIdentity(R);
    DepthImage img = camera->capture(camera_pos, R);
    
    static int counter = 0;
    if (counter++ % 50 == 0) {
        std::cout << "Depth image captured: " << img.width << "x" << img.height << std::endl;
        std::cout << "Center pixel depth: " << img.depths[img.height/2][img.width/2] << "m" << std::endl;
    }
    
    dWorldStep(world, dt);
}

void drawScene() {
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);
    
    for (size_t i = 0; i < obstacles.size(); i++) {
        const dReal* pos = dBodyGetPosition(obstacles[i]);
        const dReal* R = dBodyGetRotation(obstacles[i]);
        
        if (i == 0) {
            dReal sides[3] = {1.0, 0.5, 1.0};
            Viewer::setColor(0.7f, 0.3f, 0.3f);
            Viewer::drawBox(pos, R, sides);
        } else {
            Viewer::setColor(0.3f, 0.7f, 0.3f);
            Viewer::drawSphere(pos, R, 0.5);
        }
    }
    
    // Draw camera position
    dMatrix3 R;
    dRSetIdentity(R);
    Viewer::setColor(0.2f, 0.2f, 0.9f);
    Viewer::drawSphere(camera_pos, R, 0.1);
}

int main(int argc, char** argv) {
    std::cout << "=== 18: Camera Sensor ===" << std::endl;
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    createObstacles();
    camera = new DepthCamera(space, 64, 48, M_PI/2, M_PI/3, 10.0);
    
    Viewer viewer(argc, argv, "18: Camera Sensor - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.start();
    
    delete camera;
    for (auto obs : obstacles) dBodyDestroy(obs);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
