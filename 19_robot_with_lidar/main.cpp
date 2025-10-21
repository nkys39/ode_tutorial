#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"
#include "sensors.h"
#include <iostream>
#include <vector>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// Robot components
dBodyID robot_body;
dBodyID left_wheel, right_wheel;
dJointID left_hinge, right_hinge;

// Robot parameters
const dReal WHEEL_RADIUS = 0.05;
const dReal WHEEL_WIDTH = 0.03;
const dReal WHEEL_BASE = 0.3;
const dReal BODY_LENGTH = 0.4;
const dReal BODY_WIDTH = 0.3;
const dReal BODY_HEIGHT = 0.15;

// LiDAR sensor
LidarSensor* lidar;

// Obstacles
struct Obstacle {
    dBodyID body;
    dGeomID geom;
    dReal lx, ly, lz;
};
std::vector<Obstacle> obstacles;

// Robot controller
DifferentialDrive* diff_drive;
dReal target_linear_vel = 0.0;
dReal target_angular_vel = 0.0;

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
        contact[i].surface.mu = 10.0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createObstacles() {
    Obstacle obs;

    obs.lx = 0.5; obs.ly = 0.5; obs.lz = 0.5;
    obs.body = createBox(world, space, 2, 0, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.3; obs.ly = 0.3; obs.lz = 0.5;
    obs.body = createBox(world, space, -1, 1.5, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.4; obs.ly = 0.4; obs.lz = 0.5;
    obs.body = createBox(world, space, -1, -1.5, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.6; obs.ly = 0.2; obs.lz = 0.6;
    obs.body = createBox(world, space, 1, 2, 0.3, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.4; obs.ly = 0.4; obs.lz = 0.4;
    obs.body = createBox(world, space, -2, -1, 0.2, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);
}

void createRobot(dReal x, dReal y, dReal z) {
    // Create robot body (z is body center height)
    robot_body = createBox(world, space, x, y, z,
                          BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, 5.0);

    // Create wheels at body bottom level
    dReal wheel_center_z = z - BODY_HEIGHT / 2;
    left_wheel = createCylinder(world, space,
                               x - WHEEL_BASE / 2, y, wheel_center_z,
                               WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);

    right_wheel = createCylinder(world, space,
                                x + WHEEL_BASE / 2, y, wheel_center_z,
                                WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dBodySetRotation(right_wheel, R);

    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, x - WHEEL_BASE / 2, y, wheel_center_z);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, x + WHEEL_BASE / 2, y, wheel_center_z);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
}

void reset() {
    // Correct height: wheel radius + half body height
    dReal correct_height = WHEEL_RADIUS + BODY_HEIGHT / 2;
    dBodySetPosition(robot_body, 0, 0, correct_height);
    dBodySetLinearVel(robot_body, 0, 0, 0);
    dBodySetAngularVel(robot_body, 0, 0, 0);

    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 0, 1, 0);
    dBodySetRotation(robot_body, R);
}

void simulationStep(double dt) {
    if (Viewer::shouldReset()) {
        reset();
        Viewer::setShouldReset(false);
    }

    // Get robot pose
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal yaw = getYawFromRotation(R);

    // Perform LiDAR scan from robot position
    dReal lidar_pos[3] = {pos[0], pos[1], pos[2]};
    LidarScan scan = lidar->scan(lidar_pos, yaw);

    // Simple obstacle avoidance using LiDAR
    dReal min_front_distance = scan.max_range;
    for (int i = 150; i < 210; i++) {  // Front 60 degrees
        if (scan.ranges[i] < min_front_distance) {
            min_front_distance = scan.ranges[i];
        }
    }

    // If obstacle ahead, turn
    if (min_front_distance < 0.8 && target_linear_vel > 0) {
        std::cout << "Obstacle ahead at " << min_front_distance << "m - turning!" << std::endl;
        target_angular_vel = 1.0;  // Turn left
    }

    // Apply motor control
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel,
                                      left_vel, right_vel);

    dJointSetHingeParam(left_hinge, dParamVel, left_vel);
    dJointSetHingeParam(left_hinge, dParamFMax, 10.0);

    dJointSetHingeParam(right_hinge, dParamVel, right_vel);
    dJointSetHingeParam(right_hinge, dParamFMax, 10.0);

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

    // Draw obstacles
    for (const auto& obs : obstacles) {
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);
        dReal sides[3] = {obs.lx, obs.ly, obs.lz};

        Viewer::setColor(0.7f, 0.3f, 0.3f);
        Viewer::drawBox(pos, R, sides);
    }

    // Draw robot body
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);

    // Draw wheels
    const dReal* lw_pos = dBodyGetPosition(left_wheel);
    const dReal* lw_R = dBodyGetRotation(left_wheel);
    Viewer::setColor(0.2f, 0.2f, 0.2f);
    Viewer::drawCylinder(lw_pos, lw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    const dReal* rw_pos = dBodyGetPosition(right_wheel);
    const dReal* rw_R = dBodyGetRotation(right_wheel);
    Viewer::drawCylinder(rw_pos, rw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    // Draw LiDAR rays
    dReal yaw = getYawFromRotation(R);
    const LidarScan& scan = lidar->getLastScan();
    for (size_t i = 0; i < scan.ranges.size(); i += 10) {
        dReal angle = yaw + scan.angles[i];
        dReal range = scan.ranges[i];

        dReal end_x = pos[0] + range * std::cos(angle);
        dReal end_y = pos[1] + range * std::sin(angle);
        dReal end_z = pos[2];

        dReal start[3] = {pos[0], pos[1], pos[2]};
        dReal end[3] = {end_x, end_y, end_z};

        float color = 1.0f - (range / scan.max_range);
        Viewer::drawLine(start, end, color, 1.0f, color);
    }
}

void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w':
        case 'W':
            target_linear_vel = 0.5;
            target_angular_vel = 0.0;
            std::cout << "Move forward" << std::endl;
            break;
        case 's':
        case 'S':
            target_linear_vel = -0.3;
            target_angular_vel = 0.0;
            std::cout << "Move backward" << std::endl;
            break;
        case 'a':
        case 'A':
            target_linear_vel = 0.3;
            target_angular_vel = 1.0;
            std::cout << "Turn left" << std::endl;
            break;
        case 'd':
        case 'D':
            target_linear_vel = 0.3;
            target_angular_vel = -1.0;
            std::cout << "Turn right" << std::endl;
            break;
        case 'x':
        case 'X':
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
            std::cout << "Stop" << std::endl;
            break;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 19: Robot with LiDAR ===" << std::endl;
    std::cout << "Differential drive robot equipped with LiDAR sensor." << std::endl;
    std::cout << "Simple obstacle avoidance is implemented!" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  W: Move forward (with auto obstacle avoidance)" << std::endl;
    std::cout << "  S: Move backward" << std::endl;
    std::cout << "  A: Turn left" << std::endl;
    std::cout << "  D: Turn right" << std::endl;
    std::cout << "  X: Stop" << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    // Create ground
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // Create obstacles
    createObstacles();

    // Create robot at correct height (wheel radius + half body height)
    dReal robot_height = WHEEL_RADIUS + BODY_HEIGHT / 2;
    createRobot(0, 0, robot_height);

    // Create LiDAR sensor
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 5.0, 0.1);

    // Create viewer
    Viewer viewer(argc, argv, "19: Robot with LiDAR - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    Camera& cam = Viewer::getCamera();
    cam.distance = 6.0;
    cam.pitch = 50.0;

    viewer.start();

    // Cleanup
    delete lidar;
    delete diff_drive;
    for (auto& obs : obstacles) {
        dBodyDestroy(obs.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
