#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include "robot_utils.h"
#include <iostream>
#include <map>
#include <cmath>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// Robot components
dBodyID robot_body, left_wheel, right_wheel;
dJointID left_hinge, right_hinge;
dGeomID robot_geom, left_wheel_geom, right_wheel_geom;

const dReal WHEEL_RADIUS = 0.05;
const dReal WHEEL_WIDTH = 0.03;
const dReal WHEEL_BASE = 0.3;

DifferentialDrive* diff_drive;
Odometry* odometry;
LidarSensor* lidar;

// Obstacles
struct Obstacle {
    dBodyID body;
    dGeomID geom;
    dReal lx, ly, lz;
};
std::vector<Obstacle> obstacles;

// Simple occupancy grid map
const int MAP_SIZE = 100;
const dReal MAP_RESOLUTION = 0.1; // meters per cell
int occupancy_grid[MAP_SIZE][MAP_SIZE];

// Robot control
dReal target_linear_vel = 0.3;
dReal target_angular_vel = 0.0;

void initMap() {
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            occupancy_grid[i][j] = -1; // Unknown
        }
    }
}

void updateMap(dReal robot_x, dReal robot_y, dReal robot_yaw, const LidarScan& scan) {
    int center_i = MAP_SIZE / 2;
    int center_j = MAP_SIZE / 2;

    for (size_t i = 0; i < scan.ranges.size(); i++) {
        if (scan.ranges[i] >= scan.max_range - 0.1) continue;

        dReal angle = scan.min_angle + i * scan.angle_increment + robot_yaw;
        dReal hit_x = robot_x + scan.ranges[i] * std::cos(angle);
        dReal hit_y = robot_y + scan.ranges[i] * std::sin(angle);

        int grid_i = center_i + static_cast<int>(hit_x / MAP_RESOLUTION);
        int grid_j = center_j + static_cast<int>(hit_y / MAP_RESOLUTION);

        if (grid_i >= 0 && grid_i < MAP_SIZE && grid_j >= 0 && grid_j < MAP_SIZE) {
            occupancy_grid[grid_i][grid_j] = 100; // Occupied
        }

        // Mark free space along the ray
        int robot_i = center_i + static_cast<int>(robot_x / MAP_RESOLUTION);
        int robot_j = center_j + static_cast<int>(robot_y / MAP_RESOLUTION);

        int steps = static_cast<int>(scan.ranges[i] / MAP_RESOLUTION);
        for (int s = 0; s < steps; s++) {
            dReal t = static_cast<dReal>(s) / steps;
            int free_i = robot_i + static_cast<int>(t * (grid_i - robot_i));
            int free_j = robot_j + static_cast<int>(t * (grid_j - robot_j));

            if (free_i >= 0 && free_i < MAP_SIZE && free_j >= 0 && free_j < MAP_SIZE) {
                if (occupancy_grid[free_i][free_j] == -1) {
                    occupancy_grid[free_i][free_j] = 0; // Free
                }
            }
        }
    }
}

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
    robot_body = createBox(world, space, 0, 0, 0.2, 0.4, 0.3, 0.15, 5.0, &robot_geom);
    left_wheel = createCylinder(world, space, -0.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &left_wheel_geom);
    right_wheel = createCylinder(world, space, 0.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &right_wheel_geom);

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
    lidar = new LidarSensor(space, 360, -M_PI, M_PI, 5.0);
}

void createObstacles() {
    Obstacle obs;

    obs.lx = 1.0; obs.ly = 0.2; obs.lz = 0.5;
    obs.body = createBox(world, space, 2, 0, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.2; obs.ly = 1.5; obs.lz = 0.5;
    obs.body = createBox(world, space, 0, 2.5, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 1.2; obs.ly = 0.2; obs.lz = 0.5;
    obs.body = createBox(world, space, -1.5, -1.5, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);
}

void simulationStep(double dt) {
    // Update robot control
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

    // Get robot pose
    const dReal* robot_pos = dBodyGetPosition(robot_body);
    const dReal* robot_R = dBodyGetRotation(robot_body);
    dReal robot_yaw = getYawFromRotation(robot_R);

    // Perform LIDAR scan and update map
    static int scan_counter = 0;
    if (scan_counter++ % 10 == 0) {
        LidarScan scan = lidar->scan(robot_pos, robot_yaw);
        updateMap(robot_pos[0], robot_pos[1], robot_yaw, scan);
    }

    // Simple autonomous behavior - circle around
    static int behavior_counter = 0;
    if (behavior_counter++ > 200) {
        target_angular_vel = (target_angular_vel > 0.5) ? 0.0 : 0.8;
        behavior_counter = 0;
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

    // Draw obstacles
    for (const auto& obs : obstacles) {
        const dReal* pos = dBodyGetPosition(obs.body);
        const dReal* R = dBodyGetRotation(obs.body);
        dReal sides[3] = {obs.lx, obs.ly, obs.lz};
        Viewer::setColor(0.7f, 0.3f, 0.3f);
        Viewer::drawBox(pos, R, sides);
    }

    // Draw robot
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {0.4, 0.3, 0.15};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);

    // Draw occupancy grid map (simplified visualization)
    int center_i = MAP_SIZE / 2;
    int center_j = MAP_SIZE / 2;
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            if (occupancy_grid[i][j] == 100) {
                dReal x = (i - center_i) * MAP_RESOLUTION;
                dReal y = (j - center_j) * MAP_RESOLUTION;
                dReal map_pos[3] = {x, y, 0.02};
                dMatrix3 map_R;
                dRSetIdentity(map_R);
                Viewer::setColor(0.9f, 0.1f, 0.1f);
                Viewer::drawSphere(map_pos, map_R, 0.03);
            }
        }
    }
}

void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'w': target_linear_vel = 0.5; target_angular_vel = 0.0; break;
        case 's': target_linear_vel = -0.5; target_angular_vel = 0.0; break;
        case 'a': target_linear_vel = 0.3; target_angular_vel = 1.0; break;
        case 'd': target_linear_vel = 0.3; target_angular_vel = -1.0; break;
        case 'x': target_linear_vel = 0.0; target_angular_vel = 0.0; break;
        case 'c': initMap(); std::cout << "Map cleared" << std::endl; break;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 23: SLAM Basics ===" << std::endl;
    std::cout << "Simultaneous Localization and Mapping" << std::endl;
    std::cout << "Controls: W/A/S/D/X - Move robot, C - Clear map" << std::endl;
    std::cout << "Red dots show detected occupied cells in the map" << std::endl;

    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    initMap();
    createRobot();
    createObstacles();

    Viewer viewer(argc, argv, "23: SLAM Basics - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    viewer.start();

    // Cleanup
    delete diff_drive;
    delete odometry;
    delete lidar;
    for (auto& obs : obstacles) {
        dBodyDestroy(obs.body);
    }
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
