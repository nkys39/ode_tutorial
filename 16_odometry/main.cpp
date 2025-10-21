#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"
#include <iostream>

using namespace ode_tutorial;

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

dBodyID robot_body, left_wheel, right_wheel;
dJointID left_hinge, right_hinge;

const dReal WHEEL_RADIUS = 0.05;
const dReal WHEEL_WIDTH = 0.03;
const dReal WHEEL_BASE = 0.3;

DifferentialDrive* diff_drive;
Odometry* odometry;

dReal target_linear_vel = 0.5;
dReal target_angular_vel = 0.0;

void nearCallback(void* data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 10.0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;
        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createRobot() {
    dGeomID geom;
    robot_body = createBox(world, space, 0, 0, 0.2, 0.4, 0.3, 0.15, 5.0, &geom);
    left_wheel = createCylinder(world, space, -0.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &geom);
    right_wheel = createCylinder(world, space, 0.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &geom);
    
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);
    dBodySetRotation(right_wheel, R);
    
    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, -0.15, 0, 0.125);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);
    
    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, 0.15, 0, 0.125);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);
    
    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
    odometry = new Odometry(WHEEL_BASE, WHEEL_RADIUS);
}

void simulationStep(double dt) {
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel, left_vel, right_vel);
    
    dJointSetHingeParam(left_hinge, dParamVel, left_vel);
    dJointSetHingeParam(left_hinge, dParamFMax, 10.0);
    dJointSetHingeParam(right_hinge, dParamVel, right_vel);
    dJointSetHingeParam(right_hinge, dParamFMax, 10.0);
    
    // Update odometry
    dReal left_angle = dJointGetHingeAngle(left_hinge);
    dReal right_angle = dJointGetHingeAngle(right_hinge);
    odometry->update(left_angle, right_angle);
    
    static int counter = 0;
    if (counter++ % 100 == 0) {
        dReal odo_x, odo_y, odo_theta;
        odometry->getPose(odo_x, odo_y, odo_theta);
        
        const dReal* actual_pos = dBodyGetPosition(robot_body);
        const dReal* actual_R = dBodyGetRotation(robot_body);
        dReal actual_yaw = getYawFromRotation(actual_R);
        
        std::cout << "Odometry: (" << odo_x << ", " << odo_y << "), θ=" << radToDeg(odo_theta) << "°" << std::endl;
        std::cout << "Actual:   (" << actual_pos[0] << ", " << actual_pos[1] << "), θ=" << radToDeg(actual_yaw) << "°" << std::endl;
        std::cout << "Error: " << (actual_pos[0] - odo_x) << ", " << (actual_pos[1] - odo_y) << std::endl;
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
    
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {0.4, 0.3, 0.15};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);
}

void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w': target_linear_vel = 0.5; target_angular_vel = 0.0; break;
        case 's': target_linear_vel = -0.5; target_angular_vel = 0.0; break;
        case 'a': target_linear_vel = 0.3; target_angular_vel = 1.0; break;
        case 'd': target_linear_vel = 0.3; target_angular_vel = -1.0; break;
        case 'x': target_linear_vel = 0.0; target_angular_vel = 0.0; break;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 16: Odometry ===" << std::endl;
    std::cout << "Wheel encoder-based localization" << std::endl;
    std::cout << "Controls: W/A/S/D/X" << std::endl;
    
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);
    
    createRobot();
    
    Viewer viewer(argc, argv, "16: Odometry - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);
    viewer.start();
    
    delete diff_drive;
    delete odometry;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
