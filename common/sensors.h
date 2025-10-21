#ifndef SENSORS_H
#define SENSORS_H

#include <ode/ode.h>
#include <vector>
#include <cmath>
#include "utils.h"

namespace ode_tutorial {

// LiDAR scan data
struct LidarScan {
    std::vector<dReal> ranges;     // Distance measurements
    std::vector<dReal> angles;     // Corresponding angles
    dReal min_angle;
    dReal max_angle;
    dReal angle_increment;
    dReal max_range;
    dReal min_range;

    LidarScan(dReal min_ang = -M_PI, dReal max_ang = M_PI,
              int num_rays = 360, dReal max_r = 10.0, dReal min_r = 0.1)
        : min_angle(min_ang), max_angle(max_ang), max_range(max_r), min_range(min_r) {
        angle_increment = (max_angle - min_angle) / (num_rays - 1);
        ranges.resize(num_rays, max_range);
        angles.resize(num_rays);
        for (int i = 0; i < num_rays; i++) {
            angles[i] = min_angle + i * angle_increment;
        }
    }
};

// LiDAR sensor
class LidarSensor {
public:
    LidarSensor(dSpaceID space, dReal min_angle = -M_PI, dReal max_angle = M_PI,
                int num_rays = 360, dReal max_range = 10.0, dReal min_range = 0.1)
        : space_(space), scan_(min_angle, max_angle, num_rays, max_range, min_range) {}

    // Perform a scan from the given position and orientation
    LidarScan scan(const dReal* position, dReal yaw) {
        for (size_t i = 0; i < scan_.angles.size(); i++) {
            dReal angle = yaw + scan_.angles[i];
            dReal dx = std::cos(angle);
            dReal dy = std::sin(angle);

            // Ray cast
            dGeomID ray = dCreateRay(space_, scan_.max_range);
            dGeomRaySet(ray, position[0], position[1], position[2], dx, dy, 0);

            dReal closest_distance = scan_.max_range;

            // Check all geometries in space
            int num_geoms = dSpaceGetNumGeoms(space_);
            for (int j = 0; j < num_geoms; j++) {
                dGeomID geom = dSpaceGetGeom(space_, j);
                if (dGeomGetClass(geom) == dPlaneClass) {
                    // Skip plane for now (ground)
                    continue;
                }

                dContactGeom contact[1];
                int n = dCollide(ray, geom, 1, contact, sizeof(dContactGeom));
                if (n > 0) {
                    dReal distance = contact[0].depth;
                    if (distance < closest_distance && distance >= scan_.min_range) {
                        closest_distance = distance;
                    }
                }
            }

            scan_.ranges[i] = closest_distance;
            dGeomDestroy(ray);
        }

        return scan_;
    }

    const LidarScan& getLastScan() const { return scan_; }

private:
    dSpaceID space_;
    LidarScan scan_;
};

// RGB color structure
struct Color {
    float r, g, b;
    Color(float r_ = 0.0f, float g_ = 0.0f, float b_ = 0.0f) : r(r_), g(g_), b(b_) {}
};

// RGB-D camera image
struct RGBDImage {
    std::vector<std::vector<dReal>> depths;
    std::vector<std::vector<Color>> colors;
    int width;
    int height;
    dReal h_fov;  // Horizontal field of view
    dReal v_fov;  // Vertical field of view
    dReal max_range;

    RGBDImage(int w = 64, int h = 48, dReal hfov = M_PI / 2, dReal vfov = M_PI / 3, dReal max_r = 10.0)
        : width(w), height(h), h_fov(hfov), v_fov(vfov), max_range(max_r) {
        depths.resize(height, std::vector<dReal>(width, max_range));
        colors.resize(height, std::vector<Color>(width, Color(0.5f, 0.5f, 0.5f)));
    }
};

// For backward compatibility
typedef RGBDImage DepthImage;

// Simple depth camera
class DepthCamera {
public:
    DepthCamera(dSpaceID space, int width = 64, int height = 48,
                dReal h_fov = M_PI / 2, dReal v_fov = M_PI / 3, dReal max_range = 10.0)
        : space_(space), image_(width, height, h_fov, v_fov, max_range) {}

    DepthImage capture(const dReal* position, const dReal* rotation) {
        dReal h_angle_step = image_.h_fov / image_.width;
        dReal v_angle_step = image_.v_fov / image_.height;

        for (int v = 0; v < image_.height; v++) {
            dReal v_angle = -image_.v_fov / 2 + v * v_angle_step;
            for (int h = 0; h < image_.width; h++) {
                dReal h_angle = -image_.h_fov / 2 + h * h_angle_step;

                // Ray direction in camera frame
                dReal dx = std::cos(v_angle) * std::cos(h_angle);
                dReal dy = std::cos(v_angle) * std::sin(h_angle);
                dReal dz = std::sin(v_angle);

                // Transform to world frame (simplified - assume rotation is identity)
                dGeomID ray = dCreateRay(space_, image_.max_range);
                dGeomRaySet(ray, position[0], position[1], position[2], dx, dy, dz);

                dReal closest_distance = image_.max_range;
                dGeomID closest_geom = nullptr;
                int num_geoms = dSpaceGetNumGeoms(space_);
                for (int j = 0; j < num_geoms; j++) {
                    dGeomID geom = dSpaceGetGeom(space_, j);

                    dContactGeom contact[1];
                    int n = dCollide(ray, geom, 1, contact, sizeof(dContactGeom));
                    if (n > 0) {
                        dReal distance = contact[0].depth;
                        if (distance < closest_distance) {
                            closest_distance = distance;
                            closest_geom = geom;
                        }
                    }
                }

                image_.depths[v][h] = closest_distance;

                // Get color from geometry data
                if (closest_geom != nullptr && closest_distance < image_.max_range) {
                    Color* color = static_cast<Color*>(dGeomGetData(closest_geom));
                    if (color != nullptr) {
                        image_.colors[v][h] = *color;
                    } else {
                        // Default gray color if no color data
                        image_.colors[v][h] = Color(0.7f, 0.7f, 0.7f);
                    }
                } else {
                    // Sky/background color
                    image_.colors[v][h] = Color(0.5f, 0.7f, 0.9f);
                }

                dGeomDestroy(ray);
            }
        }

        return image_;
    }

    const DepthImage& getLastImage() const { return image_; }

    dReal getHorizontalFOV() const { return image_.h_fov; }
    dReal getVerticalFOV() const { return image_.v_fov; }

private:
    dSpaceID space_;
    DepthImage image_;
};

} // namespace ode_tutorial

#endif // SENSORS_H
