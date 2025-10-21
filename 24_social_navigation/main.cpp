#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include "robot_utils.h"
#include "pedestrian.h"
#include <iostream>
#include <vector>
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
LidarSensor* lidar;
CrowdManager* crowd_manager;

// Robot control and navigation
dReal robot_goal_x = 4.0;
dReal robot_goal_y = 0.0;
dReal target_linear_vel = 0.0;
dReal target_angular_vel = 0.0;

// Social navigation parameters
const dReal PERSONAL_SPACE = 1.0; // meters
const dReal COMFORT_DISTANCE = 0.8; // meters

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
    robot_body = createBox(world, space, -3, 0, 0.2, 0.4, 0.3, 0.15, 5.0, &robot_geom);
    left_wheel = createCylinder(world, space, -3.15, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &left_wheel_geom);
    right_wheel = createCylinder(world, space, -2.85, 0, 0.125, WHEEL_RADIUS, WHEEL_WIDTH, 0.5, &right_wheel_geom);

    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);
    dBodySetRotation(right_wheel, R);

    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, -3.15, 0, 0.125);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, -2.85, 0, 0.125);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 5.0);
}

void createCrowd() {
    // Create pedestrians moving in various directions
    for (int i = 0; i < 6; i++) {
        dReal x = -2.0 + i * 1.2;
        dReal y = (i % 2 == 0) ? 1.5 : -1.5;

        Pedestrian* ped = crowd_manager->addPedestrian(x, y, 0.8);
        ped->setGoal(x, -y * 1.5);
        ped->setDesiredSpeed(0.6 + (i % 3) * 0.2);
    }
}

void computeSocialNavigation(dReal robot_x, dReal robot_y, dReal robot_yaw,
                             const LidarScan& scan,
                             dReal& linear_vel, dReal& angular_vel) {
    // Goal direction
    dReal dx_goal = robot_goal_x - robot_x;
    dReal dy_goal = robot_goal_y - robot_y;
    dReal dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (dist_to_goal < 0.3) {
        linear_vel = 0.0;
        angular_vel = 0.0;
        return;
    }

    dReal goal_angle = std::atan2(dy_goal, dx_goal);
    dReal angle_diff = goal_angle - robot_yaw;

    // Normalize angle to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Find closest obstacles in front and sides
    dReal min_front = 5.0;
    dReal min_left = 5.0;
    dReal min_right = 5.0;

    for (size_t i = 0; i < scan.ranges.size(); i++) {
        dReal angle = scan.min_angle + i * scan.angle_increment;
        dReal abs_angle = std::abs(angle);

        if (abs_angle < M_PI / 6) { // front sector
            if (scan.ranges[i] < min_front) min_front = scan.ranges[i];
        } else if (angle > 0 && angle < M_PI / 2) { // left sector
            if (scan.ranges[i] < min_left) min_left = scan.ranges[i];
        } else if (angle < 0 && angle > -M_PI / 2) { // right sector
            if (scan.ranges[i] < min_right) min_right = scan.ranges[i];
        }
    }

    // Social navigation: maintain comfort distance
    dReal desired_linear_vel = 0.5;

    if (min_front < COMFORT_DISTANCE) {
        // Slow down when approaching people
        desired_linear_vel = 0.2 * (min_front / COMFORT_DISTANCE);
    }

    // Choose direction based on social forces
    dReal desired_angular_vel = angle_diff * 0.8;

    if (min_front < PERSONAL_SPACE) {
        // Navigate around - choose less crowded side
        if (min_left > min_right) {
            desired_angular_vel = 0.8; // Turn left
        } else {
            desired_angular_vel = -0.8; // Turn right
        }
        desired_linear_vel = std::min(desired_linear_vel, 0.3);
    }

    linear_vel = desired_linear_vel;
    angular_vel = desired_angular_vel;
}

void simulationStep(double dt) {
    // Update crowd
    crowd_manager->update(dt);

    // Reset pedestrians when they reach goals
    for (auto ped : crowd_manager->getPedestrians()) {
        if (ped->isGoalReached()) {
            const dReal* pos = ped->getPosition();
            ped->setGoal(pos[0], -pos[1] * 1.5);
        }
    }

    // Get robot pose
    const dReal* robot_pos = dBodyGetPosition(robot_body);
    const dReal* robot_R = dBodyGetRotation(robot_body);
    dReal robot_yaw = getYawFromRotation(robot_R);

    // Scan environment
    LidarScan scan = lidar->scan(robot_pos, robot_yaw);

    // Compute social navigation
    computeSocialNavigation(robot_pos[0], robot_pos[1], robot_yaw, scan,
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

    // Draw goal
    dReal goal_pos[3] = {robot_goal_x, robot_goal_y, 0.1};
    dMatrix3 goal_R;
    dRSetIdentity(goal_R);
    Viewer::setColor(1.0f, 0.8f, 0.0f);
    Viewer::drawSphere(goal_pos, goal_R, 0.2);

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
    const dReal* robot_pos = dBodyGetPosition(robot_body);

    switch (key) {
        case 'g':
            std::cout << "Click to set new goal (for now, cycling through preset goals)" << std::endl;
            if (robot_goal_x > 3.0) {
                robot_goal_x = -3.0;
                robot_goal_y = 2.0;
            } else if (robot_goal_x < -2.0) {
                robot_goal_x = 0.0;
                robot_goal_y = -2.0;
            } else {
                robot_goal_x = 4.0;
                robot_goal_y = 0.0;
            }
            std::cout << "New goal: (" << robot_goal_x << ", " << robot_goal_y << ")" << std::endl;
            break;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 24: Social Navigation ===" << std::endl;
    std::cout << "Robot navigates among pedestrians with social awareness" << std::endl;
    std::cout << "Press 'g' to cycle through goal positions" << std::endl;
    std::cout << "Robot maintains comfortable distance from people" << std::endl;

    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);
    contact_group = dJointGroupCreate(0);

    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    crowd_manager = new CrowdManager(world, space);
    createRobot();
    createCrowd();

    Viewer viewer(argc, argv, "24: Social Navigation - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    viewer.start();

    // Cleanup
    delete diff_drive;
    delete lidar;
    delete crowd_manager;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
