#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include "robot_utils.h"
#include "pedestrian.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

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
CrowdManager* crowd_manager;

// Obstacles
struct Obstacle {
    dBodyID body;
    dGeomID geom;
    dReal lx, ly, lz;
};
std::vector<Obstacle> obstacles;

// Navigation
struct Waypoint {
    dReal x, y;
};
std::queue<Waypoint> waypoints;
Waypoint current_goal = {4.0, 0.0};

dReal target_linear_vel = 0.0;
dReal target_angular_vel = 0.0;

// Dynamic window approach parameters
const dReal MAX_LINEAR_VEL = 0.8;
const dReal MAX_ANGULAR_VEL = 1.5;
const dReal LINEAR_ACC = 0.5;
const dReal ANGULAR_ACC = 2.0;
const dReal GOAL_THRESHOLD = 0.4;

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
    robot_body = createBox(world, space, -4, -3, 0.2, 0.4, 0.3, 0.15, 5.0, &robot_geom);
    left_wheel = createCylinder(world, space, -4.15, -3, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &left_wheel_geom);
    right_wheel = createCylinder(world, space, -3.85, -3, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &right_wheel_geom);

    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);
    dBodySetRotation(right_wheel, R);

    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, -4.15, -3, 0.125);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, -3.85, -3, 0.125);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
    odometry = new Odometry(WHEEL_BASE, WHEEL_RADIUS);
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 5.0);
}

void createObstacles() {
    Obstacle obs;

    obs.lx = 0.3; obs.ly = 2.0; obs.lz = 0.5;
    obs.body = createBox(world, space, 0, 0, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 1.5; obs.ly = 0.3; obs.lz = 0.5;
    obs.body = createBox(world, space, 2, -2, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);

    obs.lx = 0.3; obs.ly = 1.5; obs.lz = 0.5;
    obs.body = createBox(world, space, -2, 2, 0.25, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);
}

void createCrowd() {
    // Create some pedestrians as dynamic obstacles
    for (int i = 0; i < 4; i++) {
        dReal x = -1.0 + i * 1.5;
        dReal y = (i % 2 == 0) ? 1.0 : -1.0;

        Pedestrian* ped = crowd_manager->addPedestrian(x, y, 0.8);
        ped->setGoal(x + 2.0, -y);
        ped->setDesiredSpeed(0.5);
    }
}

void initWaypoints() {
    waypoints.push(Waypoint{-2.0, -3.0});
    waypoints.push(Waypoint{-2.0, 0.0});
    waypoints.push(Waypoint{2.0, 2.0});
    waypoints.push(Waypoint{4.0, 0.0});
    current_goal = waypoints.front();
}

dReal evaluateTrajectory(dReal v, dReal w, dReal robot_x, dReal robot_y, dReal robot_yaw,
                         const LidarScan& scan, dReal dt) {
    // Predict position after dt
    dReal pred_x = robot_x + v * std::cos(robot_yaw) * dt;
    dReal pred_y = robot_y + v * std::sin(robot_yaw) * dt;
    dReal pred_yaw = robot_yaw + w * dt;

    // Goal heading
    dReal dx = current_goal.x - pred_x;
    dReal dy = current_goal.y - pred_y;
    dReal goal_angle = std::atan2(dy, dx);
    dReal heading_error = std::abs(goal_angle - pred_yaw);
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    heading_error = std::abs(heading_error);

    // Distance to goal
    dReal dist_to_goal = std::sqrt(dx * dx + dy * dy);

    // Clearance (minimum distance to obstacles)
    dReal min_clearance = 5.0;
    for (size_t i = 0; i < scan.ranges.size(); i++) {
        if (scan.ranges[i] < min_clearance) {
            min_clearance = scan.ranges[i];
        }
    }

    // Cost function (lower is better)
    dReal heading_cost = heading_error / M_PI;
    dReal clearance_cost = (min_clearance < 0.5) ? (1.0 - min_clearance / 0.5) : 0.0;
    dReal velocity_cost = 1.0 - (v / MAX_LINEAR_VEL);

    dReal total_cost = 0.5 * heading_cost + 0.3 * clearance_cost + 0.2 * velocity_cost;

    // Return negative cost as score (higher is better)
    return -total_cost;
}

void computeAutonomousNavigation(dReal robot_x, dReal robot_y, dReal robot_yaw,
                                 const LidarScan& scan, dReal dt,
                                 dReal current_v, dReal current_w,
                                 dReal& linear_vel, dReal& angular_vel) {
    // Check if goal reached
    dReal dx = current_goal.x - robot_x;
    dReal dy = current_goal.y - robot_y;
    dReal dist = std::sqrt(dx * dx + dy * dy);

    if (dist < GOAL_THRESHOLD) {
        if (!waypoints.empty()) {
            waypoints.pop();
            if (!waypoints.empty()) {
                current_goal = waypoints.front();
                std::cout << "Waypoint reached! Next goal: (" << current_goal.x << ", " << current_goal.y << ")" << std::endl;
            } else {
                std::cout << "All waypoints reached!" << std::endl;
                linear_vel = 0.0;
                angular_vel = 0.0;
                return;
            }
        } else {
            linear_vel = 0.0;
            angular_vel = 0.0;
            return;
        }
    }

    // Dynamic Window Approach - sample velocities
    dReal best_score = -1e10;
    dReal best_v = 0.0;
    dReal best_w = 0.0;

    for (dReal v = 0.0; v <= MAX_LINEAR_VEL; v += 0.1) {
        for (dReal w = -MAX_ANGULAR_VEL; w <= MAX_ANGULAR_VEL; w += 0.3) {
            // Check if velocities are reachable
            if (std::abs(v - current_v) > LINEAR_ACC * dt) continue;
            if (std::abs(w - current_w) > ANGULAR_ACC * dt) continue;

            dReal score = evaluateTrajectory(v, w, robot_x, robot_y, robot_yaw, scan, dt);

            if (score > best_score) {
                best_score = score;
                best_v = v;
                best_w = w;
            }
        }
    }

    linear_vel = best_v;
    angular_vel = best_w;
}

void simulationStep(double dt) {
    // Update crowd
    crowd_manager->update(dt);

    // Reset pedestrians when they reach goals
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            ped->setGoal(pos[0] + 2.0, -pos[1]);
        }
    }

    // Update odometry
    dReal left_angle = dJointGetHingeAngle(left_hinge);
    dReal right_angle = dJointGetHingeAngle(right_hinge);
    odometry->update(left_angle, right_angle);

    // Get robot pose
    const dReal* robot_pos = dBodyGetPosition(robot_body);
    const dReal* robot_R = dBodyGetRotation(robot_body);
    dReal robot_yaw = getYawFromRotation(robot_R);

    // Scan environment
    LidarScan scan = lidar->scan(robot_pos, robot_yaw);

    // Compute autonomous navigation
    computeAutonomousNavigation(robot_pos[0], robot_pos[1], robot_yaw, scan, dt,
                               target_linear_vel, target_angular_vel,
                               target_linear_vel, target_angular_vel);

    // Apply robot control
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel, left_vel, right_vel);

    dJointSetHingeParam(left_hinge, dParamVel, left_vel);
    dJointSetHingeParam(left_hinge, dParamFMax, 10.0);
    dJointSetHingeParam(right_hinge, dParamVel, right_vel);
    dJointSetHingeParam(right_hinge, dParamFMax, 10.0);

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

    // Draw current goal
    dReal goal_pos[3] = {current_goal.x, current_goal.y, 0.15};
    dMatrix3 goal_R;
    dRSetIdentity(goal_R);
    Viewer::setColor(1.0f, 0.8f, 0.0f);
    Viewer::drawSphere(goal_pos, goal_R, 0.25);

    // Draw waypoints
    std::queue<Waypoint> wp_copy = waypoints;
    while (!wp_copy.empty()) {
        Waypoint wp = wp_copy.front();
        wp_copy.pop();
        dReal wp_pos[3] = {wp.x, wp.y, 0.1};
        Viewer::setColor(0.9f, 0.9f, 0.2f);
        Viewer::drawSphere(wp_pos, goal_R, 0.15);
    }

    // Draw pedestrians
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

    // Draw robot
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {0.4, 0.3, 0.15};
    Viewer::setColor(0.2f, 0.8f, 0.2f);
    Viewer::drawBox(pos, R, sides);
}

void keyboardCallback(unsigned char key, int x, int y) {
    switch (key) {
        case 'r':
            // Reset waypoints
            while (!waypoints.empty()) waypoints.pop();
            initWaypoints();
            std::cout << "Waypoints reset" << std::endl;
            break;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 25: Autonomous Navigation ===" << std::endl;
    std::cout << "Robot navigates autonomously through waypoints" << std::endl;
    std::cout << "Uses Dynamic Window Approach for local planning" << std::endl;
    std::cout << "Press 'r' to reset waypoints" << std::endl;

    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    crowd_manager = new CrowdManager(world, space);
    createRobot();
    createObstacles();
    createCrowd();
    initWaypoints();

    Viewer viewer(argc, argv, "25: Autonomous Navigation - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    viewer.start();

    // Cleanup
    delete diff_drive;
    delete odometry;
    delete lidar;
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
