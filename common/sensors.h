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

// RGB-D camera image (supports different resolutions for RGB and depth)
struct RGBDImage {
    std::vector<std::vector<dReal>> depths;
    std::vector<std::vector<Color>> colors;
    int rgb_width, rgb_height;    // RGB camera resolution (high-res)
    int depth_width, depth_height; // Depth sensor resolution (low-res)
    dReal h_fov;  // Horizontal field of view
    dReal v_fov;  // Vertical field of view
    dReal max_range;

    RGBDImage(int rgb_w = 640, int rgb_h = 480, int depth_w = 64, int depth_h = 48,
              dReal hfov = M_PI / 2, dReal vfov = M_PI / 3, dReal max_r = 10.0)
        : rgb_width(rgb_w), rgb_height(rgb_h), depth_width(depth_w), depth_height(depth_h),
          h_fov(hfov), v_fov(vfov), max_range(max_r) {
        depths.resize(depth_height, std::vector<dReal>(depth_width, max_range));
        colors.resize(rgb_height, std::vector<Color>(rgb_width, Color(0.5f, 0.5f, 0.5f)));
    }
};

// For backward compatibility
typedef RGBDImage DepthImage;

// RGB-D camera (RGB: OpenGL rendering, Depth: raycast)
class DepthCamera {
public:
    DepthCamera(dSpaceID space, int rgb_width = 640, int rgb_height = 480,
                int depth_width = 64, int depth_height = 48,
                dReal h_fov = M_PI / 2, dReal v_fov = M_PI / 3, dReal max_range = 10.0)
        : space_(space), image_(rgb_width, rgb_height, depth_width, depth_height, h_fov, v_fov, max_range) {}

    // Capture depth only via raycast (RGB will be captured by OpenGL rendering)
    DepthImage captureDepth(const dReal* position, const dReal* rotation) {
        dReal h_angle_step = image_.h_fov / image_.depth_width;
        dReal v_angle_step = image_.v_fov / image_.depth_height;

        for (int v = 0; v < image_.depth_height; v++) {
            dReal v_angle = -image_.v_fov / 2 + v * v_angle_step;
            for (int h = 0; h < image_.depth_width; h++) {
                dReal h_angle = -image_.h_fov / 2 + h * h_angle_step;

                // Ray direction in camera frame
                dReal dx = std::cos(v_angle) * std::cos(h_angle);
                dReal dy = std::cos(v_angle) * std::sin(h_angle);
                dReal dz = std::sin(v_angle);

                // Transform to world frame (simplified - assume rotation is identity)
                dGeomID ray = dCreateRay(space_, image_.max_range);
                dGeomRaySet(ray, position[0], position[1], position[2], dx, dy, dz);

                dReal closest_distance = image_.max_range;
                int num_geoms = dSpaceGetNumGeoms(space_);
                for (int j = 0; j < num_geoms; j++) {
                    dGeomID geom = dSpaceGetGeom(space_, j);

                    dContactGeom contact[1];
                    int n = dCollide(ray, geom, 1, contact, sizeof(dContactGeom));
                    if (n > 0) {
                        dReal distance = contact[0].depth;
                        if (distance < closest_distance) {
                            closest_distance = distance;
                        }
                    }
                }

                image_.depths[v][h] = closest_distance;
                dGeomDestroy(ray);
            }
        }

        return image_;
    }

    // Backward compatibility
    DepthImage capture(const dReal* position, const dReal* rotation) {
        return captureDepth(position, rotation);
    }

    // Get reference to image for external RGB data injection
    RGBDImage& getImage() { return image_; }

    const DepthImage& getLastImage() const { return image_; }

    dReal getHorizontalFOV() const { return image_.h_fov; }
    dReal getVerticalFOV() const { return image_.v_fov; }

private:
    dSpaceID space_;
    DepthImage image_;
};

} // namespace ode_tutorial

#endif // SENSORS_H
