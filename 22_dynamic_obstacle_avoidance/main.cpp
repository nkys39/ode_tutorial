#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"
#include "sensors.h"
#include "pedestrian.h"
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

// Pedestrians
CrowdManager* crowd_manager;

// Static obstacles
struct Obstacle {
    dBodyID body;
    dGeomID geom;
    dReal lx, ly, lz;
};
std::vector<Obstacle> obstacles;

// Robot controller
DifferentialDrive* diff_drive;
dReal target_linear_vel = 0.5;  // Auto-drive mode
dReal target_angular_vel = 0.0;
bool auto_drive = true;

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
        contact[i].surface.mu = 5.0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createObstacles() {
    // Static obstacles
    Obstacle obs;

    obs.lx = 0.6; obs.ly = 0.4; obs.lz = 0.6;
    obs.body = createBox(world, space, 3, 1, 0.3, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.5; obs.ly = 0.5; obs.lz = 0.5;
    obs.body = createBox(world, space, -2, -2, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.6; obs.ly = 0.2; obs.lz = 0.6;
    obs.body = createBox(world, space, 2, -3, 0.3, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);
}

void createPedestrians() {
    // Create pedestrians crossing the robot's path
    Pedestrian* p1 = crowd_manager->addPedestrian(-3, 2, 0.8);
    p1->setGoal(3, -2);
    p1->setDesiredSpeed(1.0);

    Pedestrian* p2 = crowd_manager->addPedestrian(3, -2, 0.8);
    p2->setGoal(-3, 2);
    p2->setDesiredSpeed(1.2);

    Pedestrian* p3 = crowd_manager->addPedestrian(-2, -3, 0.8);
    p3->setGoal(2, 3);
    p3->setDesiredSpeed(0.9);

    Pedestrian* p4 = crowd_manager->addPedestrian(0, 3, 0.8);
    p4->setGoal(0, -3);
    p4->setDesiredSpeed(1.1);
}

void createRobot(dReal x, dReal y, dReal z) {
    robot_body = createBox(world, space, x, y, z,
                          BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, 5.0);

    left_wheel = createCylinder(world, space,
                               x - WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2,
                               WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);

    right_wheel = createCylinder(world, space,
                                x + WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2,
                                WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dBodySetRotation(right_wheel, R);

    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, x - WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, x + WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
}

void simulationStep(double dt) {
    // Update pedestrians
    crowd_manager->update(dt);

    // Reset pedestrians when they reach goals
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            ped->setGoal(-pos[0] * 0.9, -pos[1] * 0.9);
        }
    }

    // Get robot pose
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal yaw = getYawFromRotation(R);

    // Perform LiDAR scan
    dReal lidar_pos[3] = {pos[0], pos[1], pos[2]};
    LidarScan scan = lidar->scan(lidar_pos, yaw);

    if (auto_drive) {
        // Dynamic obstacle avoidance algorithm
        dReal min_left_distance = scan.max_range;
        dReal min_front_distance = scan.max_range;
        dReal min_right_distance = scan.max_range;

        // Check left sector (210-270 degrees)
        for (int i = 210; i < 270; i++) {
            if (scan.ranges[i] < min_left_distance) {
                min_left_distance = scan.ranges[i];
            }
        }

        // Check front sector (150-210 degrees)
        for (int i = 150; i < 210; i++) {
            if (scan.ranges[i] < min_front_distance) {
                min_front_distance = scan.ranges[i];
            }
        }

        // Check right sector (90-150 degrees)
        for (int i = 90; i < 150; i++) {
            if (scan.ranges[i] < min_right_distance) {
                min_right_distance = scan.ranges[i];
            }
        }

        // Decision making
        if (min_front_distance < 1.5) {
            // Obstacle ahead - choose direction with more space
            if (min_left_distance > min_right_distance) {
                target_angular_vel = 1.2;  // Turn left
                target_linear_vel = 0.3;
                std::cout << "Avoiding obstacle - turning LEFT (front: "
                          << min_front_distance << "m)" << std::endl;
            } else {
                target_angular_vel = -1.2;  // Turn right
                target_linear_vel = 0.3;
                std::cout << "Avoiding obstacle - turning RIGHT (front: "
                          << min_front_distance << "m)" << std::endl;
            }
        } else if (min_front_distance < 2.5) {
            // Slow down approach
            target_angular_vel = 0.0;
            target_linear_vel = 0.3;
        } else {
            // Clear path - move forward
            target_angular_vel = 0.0;
            target_linear_vel = 0.5;
        }
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

    // Draw static obstacles
    for (const auto& obs : obstacles) {
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);
        dReal sides[3] = {obs.lx, obs.ly, obs.lz};

        Viewer::setColor(0.6f, 0.2f, 0.2f);
        Viewer::drawBox(pos, R, sides);
    }

    // Draw pedestrians
    int ped_num = 0;
    for (auto ped : crowd_manager->getPedestrians()) {
        const dReal* pos = ped->getPosition();
        dGeomID geom = ped->getGeom();
        const dReal* R = dGeomGetRotation(geom);
        dReal radius, length;
        dGeomCapsuleGetParams(geom, &radius, &length);

        float hue = (ped_num++ * 0.25f);
        Viewer::setColor(0.9f - hue * 0.3f, 0.5f + hue * 0.3f, 0.8f);
        Viewer::drawCapsule(pos, R, length, radius);
    }

    // Draw robot
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT};
    Viewer::setColor(0.2f, 0.8f, 0.2f);
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
    for (size_t i = 0; i < scan.ranges.size(); i += 15) {
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
        case 't':
        case 'T':
            auto_drive = !auto_drive;
            if (auto_drive) {
                std::cout << "Auto-drive ENABLED" << std::endl;
            } else {
                std::cout << "Auto-drive DISABLED (manual control)" << std::endl;
            }
            break;
        case 'w':
        case 'W':
            if (!auto_drive) {
                target_linear_vel = 0.5;
                target_angular_vel = 0.0;
            }
            break;
        case 's':
        case 'S':
            if (!auto_drive) {
                target_linear_vel = -0.3;
                target_angular_vel = 0.0;
            }
            break;
        case 'a':
        case 'A':
            if (!auto_drive) {
                target_linear_vel = 0.3;
                target_angular_vel = 1.0;
            }
            break;
        case 'd':
        case 'D':
            if (!auto_drive) {
                target_linear_vel = 0.3;
                target_angular_vel = -1.0;
            }
            break;
        case 'x':
        case 'X':
            target_linear_vel = 0.0;
            target_angular_vel = 0.0;
            break;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 22: Dynamic Obstacle Avoidance ===" << std::endl;
    std::cout << "Robot with LiDAR avoids both static and dynamic obstacles!" << std::endl;
    std::cout << "Pedestrians move around as dynamic obstacles." << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  T: Toggle auto-drive mode" << std::endl;
    std::cout << "  W/A/S/D: Manual control (when auto-drive is off)" << std::endl;
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

    // Create static obstacles
    createObstacles();

    // Create crowd manager and pedestrians
    crowd_manager = new CrowdManager(world, space);
    createPedestrians();

    // Create robot
    createRobot(-4, -4, 0.2);

    // Create LiDAR sensor
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 6.0, 0.1);

    // Create viewer
    Viewer viewer(argc, argv, "22: Dynamic Obstacle Avoidance - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    Camera& cam = Viewer::getCamera();
    cam.distance = 12.0;
    cam.pitch = 65.0;

    std::cout << "Auto-drive mode is ENABLED by default." << std::endl;
    std::cout << "Watch the robot avoid both static boxes and moving pedestrians!" << std::endl;

    viewer.start();

    // Cleanup
    delete lidar;
    delete diff_drive;
    delete crowd_manager;
    for (auto& obs : obstacles) {
        dBodyDestroy(obs.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
