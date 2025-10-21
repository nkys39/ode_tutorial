#include "viewer.h"
#include "utils.h"
#include "sensors.h"
#include <iostream>

using namespace ode_tutorial;

// ODE world and objects
dWorldID world;
dSpaceID space;
dGeomID ground_geom;

// Obstacles
struct Obstacle {
    dBodyID body;
    dGeomID geom;
    dReal lx, ly, lz;
};
std::vector<Obstacle> obstacles;

// LiDAR sensor
LidarSensor* lidar;
dReal sensor_pos[3] = {0, 0, 0.5};
dReal sensor_yaw = 0.0;

void createObstacles() {
    // Create some boxes as obstacles
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
    obs.body = createBox(world, space, 0, 2.5, 0.3, obs.lx, obs.ly, obs.lz, 1.0, &obs.geom);
    obstacles.push_back(obs);
}

void simulationStep(double dt) {
    // Rotate sensor slowly
    sensor_yaw += dt * 0.5;  // 0.5 rad/s rotation

    // Perform LiDAR scan
    LidarScan scan = lidar->scan(sensor_pos, sensor_yaw);

    // Print some scan data
    static int counter = 0;
    if (counter++ % 50 == 0) {
        std::cout << "LiDAR scan at yaw=" << radToDeg(sensor_yaw) << "°" << std::endl;
        std::cout << "  Front (0°): " << scan.ranges[180] << " m" << std::endl;
        std::cout << "  Right (90°): " << scan.ranges[90] << " m" << std::endl;
        std::cout << "  Back (180°): " << scan.ranges[0] << " m" << std::endl;
        std::cout << "  Left (270°): " << scan.ranges[270] << " m" << std::endl;
    }

    dWorldStep(world, dt);
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

    // Draw sensor position
    dMatrix3 R;
    dRSetIdentity(R);
    Viewer::setColor(0.3f, 0.7f, 0.9f);
    Viewer::drawSphere(sensor_pos, R, 0.1);

    // Draw LiDAR rays
    const LidarScan& scan = lidar->getLastScan();
    for (size_t i = 0; i < scan.ranges.size(); i += 10) {  // Draw every 10th ray
        dReal angle = sensor_yaw + scan.angles[i];
        dReal range = scan.ranges[i];

        dReal end_x = sensor_pos[0] + range * std::cos(angle);
        dReal end_y = sensor_pos[1] + range * std::sin(angle);
        dReal end_z = sensor_pos[2];

        dReal end[3] = {end_x, end_y, end_z};

        // Color based on distance
        float color = 1.0f - (range / scan.max_range);
        Viewer::drawLine(sensor_pos, end, color, color, 1.0f);
    }
}

int main(int argc, char** argv) {
    std::cout << "=== 17: LiDAR Sensor Simulation ===" << std::endl;
    std::cout << "Simulates a 2D LiDAR sensor using raycasting." << std::endl;
    std::cout << "The sensor rotates and detects obstacles." << std::endl;
    std::cout << std::endl;

    // Initialize ODE
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);

    space = dHashSpaceCreate(0);

    // Create ground
    ground_geom = dCreatePlane(space, 0, 0, 1, 0);

    // Create obstacles
    createObstacles();

    // Create LiDAR sensor (360 degrees, 360 rays, 10m max range)
    lidar = new LidarSensor(space, -M_PI, M_PI, 360, 10.0, 0.1);

    // Create viewer
    Viewer viewer(argc, argv, "17: LiDAR Sensor - ODE Tutorial");
    viewer.setSimulationCallback(simulationStep);
    viewer.setDrawCallback(drawScene);

    // Adjust camera
    Camera& cam = Viewer::getCamera();
    cam.distance = 8.0;
    cam.pitch = 60.0;

    viewer.start();

    // Cleanup
    delete lidar;
    for (auto& obs : obstacles) {
        dBodyDestroy(obs.body);
    }
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
