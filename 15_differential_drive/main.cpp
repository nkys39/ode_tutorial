#include "viewer.h"
#include "utils.h"
#include "robot_utils.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dJointGroupID contact_group;
dGeomID ground_geom;

// Robot components
dBodyID robot_body;
dBodyID left_wheel, right_wheel;
dBodyID front_caster, rear_caster;
dJointID left_hinge, right_hinge;
dJointID front_caster_joint, rear_caster_joint;

// Robot parameters
const dReal WHEEL_RADIUS = 0.05;
const dReal WHEEL_WIDTH = 0.03;
const dReal WHEEL_BASE = 0.3;
const dReal BODY_LENGTH = 0.4;
const dReal BODY_WIDTH = 0.3;
const dReal BODY_HEIGHT = 0.15;
const dReal CASTER_RADIUS = 0.03;  // Small caster wheels

// Robot controller
DifferentialDrive* diff_drive;
dReal target_linear_vel = 0.0;  // Start stationary (was 0.5)
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

        // Low friction for casters, high friction for drive wheels
        bool is_caster = (b1 == front_caster || b1 == rear_caster ||
                         b2 == front_caster || b2 == rear_caster);
        contact[i].surface.mu = is_caster ? 0.1 : 10.0;

        contact[i].surface.bounce = 0.1;
        contact[i].surface.soft_cfm = 0.01;

        dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

void createRobot(dReal x, dReal y, dReal z) {
    // Create robot body
    robot_body = createBox(world, space, x, y, z,
                          BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, 5.0);

    // Create left wheel (cylinder, rotated 90 degrees around X axis)
    left_wheel = createCylinder(world, space,
                               x - WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2,
                               WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dMatrix3 R;
    dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
    dBodySetRotation(left_wheel, R);

    // Create right wheel (cylinder, rotated 90 degrees around X axis)
    right_wheel = createCylinder(world, space,
                                x + WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2,
                                WHEEL_RADIUS, WHEEL_WIDTH, 0.5);
    dBodySetRotation(right_wheel, R);

    // Create hinge joints for drive wheels
    left_hinge = dJointCreateHinge(world, 0);
    dJointAttach(left_hinge, robot_body, left_wheel);
    dJointSetHingeAnchor(left_hinge, x - WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2);
    dJointSetHingeAxis(left_hinge, 1, 0, 0);

    right_hinge = dJointCreateHinge(world, 0);
    dJointAttach(right_hinge, robot_body, right_wheel);
    dJointSetHingeAnchor(right_hinge, x + WHEEL_BASE / 2, y, z - BODY_HEIGHT / 2);
    dJointSetHingeAxis(right_hinge, 1, 0, 0);

    // Create front caster (sphere, low friction, free to rotate)
    dReal caster_z = z - BODY_HEIGHT / 2 - WHEEL_RADIUS + CASTER_RADIUS;
    front_caster = createSphere(world, space,
                               x, y + BODY_WIDTH / 2 + CASTER_RADIUS / 2, caster_z,
                               CASTER_RADIUS, 0.1);

    // Create rear caster (sphere, low friction, free to rotate)
    rear_caster = createSphere(world, space,
                              x, y - BODY_WIDTH / 2 - CASTER_RADIUS / 2, caster_z,
                              CASTER_RADIUS, 0.1);

    // Attach casters with ball joints (allows free rotation)
    front_caster_joint = dJointCreateBall(world, 0);
    dJointAttach(front_caster_joint, robot_body, front_caster);
    dJointSetBallAnchor(front_caster_joint, x, y + BODY_WIDTH / 2 + CASTER_RADIUS / 2,
                       z - BODY_HEIGHT / 2);

    rear_caster_joint = dJointCreateBall(world, 0);
    dJointAttach(rear_caster_joint, robot_body, rear_caster);
    dJointSetBallAnchor(rear_caster_joint, x, y - BODY_WIDTH / 2 - CASTER_RADIUS / 2,
                       z - BODY_HEIGHT / 2);

    // Initialize differential drive controller
    diff_drive = new DifferentialDrive(WHEEL_BASE, WHEEL_RADIUS);
}

void reset() {
    dBodySetPosition(robot_body, 0, 0, 0.2);
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

    // Compute wheel velocities from desired linear and angular velocities
    dReal left_vel, right_vel;
    diff_drive->computeWheelVelocities(target_linear_vel, target_angular_vel,
                                      left_vel, right_vel);

    // Apply motor velocities to wheels
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

    // Print robot position
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal yaw = getYawFromRotation(R);

    static int counter = 0;
    if (counter++ % 100 == 0) {
        std::cout << "Robot pos: (" << pos[0] << ", " << pos[1]
                  << "), yaw: " << radToDeg(yaw) << "°" << std::endl;
    }
}

void drawScene() {
    // Draw ground
    dVector4 plane;
    dGeomPlaneGetParams(ground_geom, plane);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawPlane(plane, plane[3], 10.0);

    // Draw robot body
    const dReal* pos = dBodyGetPosition(robot_body);
    const dReal* R = dBodyGetRotation(robot_body);
    dReal sides[3] = {BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT};
    Viewer::setColor(0.3f, 0.7f, 0.3f);
    Viewer::drawBox(pos, R, sides);

    // Draw left drive wheel (black)
    const dReal* lw_pos = dBodyGetPosition(left_wheel);
    const dReal* lw_R = dBodyGetRotation(left_wheel);
    Viewer::setColor(0.2f, 0.2f, 0.2f);
    Viewer::drawCylinder(lw_pos, lw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    // Draw right drive wheel (black)
    const dReal* rw_pos = dBodyGetPosition(right_wheel);
    const dReal* rw_R = dBodyGetRotation(right_wheel);
    Viewer::drawCylinder(rw_pos, rw_R, WHEEL_WIDTH, WHEEL_RADIUS);

    // Draw front caster (gray)
    const dReal* fc_pos = dBodyGetPosition(front_caster);
    const dReal* fc_R = dBodyGetRotation(front_caster);
    Viewer::setColor(0.5f, 0.5f, 0.5f);
    Viewer::drawSphere(fc_pos, fc_R, CASTER_RADIUS);

    // Draw rear caster (gray)
    const dReal* rc_pos = dBodyGetPosition(rear_caster);
    const dReal* rc_R = dBodyGetRotation(rear_caster);
    Viewer::drawSphere(rc_pos, rc_R, CASTER_RADIUS);
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
            target_linear_vel = -0.5;
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
    std::cout << "=== 15: Differential Drive Robot ===" << std::endl;
    std::cout << "Control a differential drive robot!" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  W: Move forward" << std::endl;
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

    // Create robot
    createRobot(0, 0, 0.2);

    // Create viewer
    Viewer viewer(argc, argv, "15: Differential Drive Robot - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);
    viewer.setKeyboardCallback(keyboardCallback);

    viewer.start();

    // Cleanup
    delete diff_drive;
    dJointGroupDestroy(contact_group);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
