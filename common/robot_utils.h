#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <ode/ode.h>
#include <cmath>
#include "utils.h"

namespace ode_tutorial {

// Differential drive robot kinematics
class DifferentialDrive {
public:
    DifferentialDrive(dReal wheel_base, dReal wheel_radius)
        : wheel_base_(wheel_base), wheel_radius_(wheel_radius) {}

    // Compute linear and angular velocity from wheel velocities
    void computeVelocities(dReal left_wheel_vel, dReal right_wheel_vel,
                          dReal& linear_vel, dReal& angular_vel) const {
        linear_vel = wheel_radius_ * (right_wheel_vel + left_wheel_vel) / 2.0;
        angular_vel = wheel_radius_ * (right_wheel_vel - left_wheel_vel) / wheel_base_;
    }

    // Compute wheel velocities from desired linear and angular velocity
    void computeWheelVelocities(dReal linear_vel, dReal angular_vel,
                               dReal& left_wheel_vel, dReal& right_wheel_vel) const {
        left_wheel_vel = (linear_vel - angular_vel * wheel_base_ / 2.0) / wheel_radius_;
        right_wheel_vel = (linear_vel + angular_vel * wheel_base_ / 2.0) / wheel_radius_;
    }

    dReal getWheelBase() const { return wheel_base_; }
    dReal getWheelRadius() const { return wheel_radius_; }

private:
    dReal wheel_base_;
    dReal wheel_radius_;
};

// Odometry estimator
class Odometry {
public:
    Odometry(dReal wheel_base, dReal wheel_radius)
        : wheel_base_(wheel_base), wheel_radius_(wheel_radius),
          x_(0), y_(0), theta_(0),
          left_wheel_pos_(0), right_wheel_pos_(0) {}

    // Update odometry based on wheel positions
    void update(dReal left_wheel_pos, dReal right_wheel_pos) {
        dReal delta_left = left_wheel_pos - left_wheel_pos_;
        dReal delta_right = right_wheel_pos - right_wheel_pos_;

        left_wheel_pos_ = left_wheel_pos;
        right_wheel_pos_ = right_wheel_pos;

        // Arc lengths
        dReal left_arc = delta_left * wheel_radius_;
        dReal right_arc = delta_right * wheel_radius_;

        // Linear and angular displacement
        dReal linear_disp = (left_arc + right_arc) / 2.0;
        dReal angular_disp = (right_arc - left_arc) / wheel_base_;

        // Update pose
        theta_ += angular_disp;
        x_ += linear_disp * std::cos(theta_);
        y_ += linear_disp * std::sin(theta_);

        // Normalize theta to [-pi, pi]
        while (theta_ > M_PI) theta_ -= 2 * M_PI;
        while (theta_ < -M_PI) theta_ += 2 * M_PI;
    }

    // Reset odometry to given pose
    void reset(dReal x = 0, dReal y = 0, dReal theta = 0) {
        x_ = x;
        y_ = y;
        theta_ = theta;
        left_wheel_pos_ = 0;
        right_wheel_pos_ = 0;
    }

    // Getters
    dReal getX() const { return x_; }
    dReal getY() const { return y_; }
    dReal getTheta() const { return theta_; }

    void getPose(dReal& x, dReal& y, dReal& theta) const {
        x = x_;
        y = y_;
        theta = theta_;
    }

private:
    dReal wheel_base_;
    dReal wheel_radius_;
    dReal x_, y_, theta_;
    dReal left_wheel_pos_, right_wheel_pos_;
};

// Simple PID controller
class PIDController {
public:
    PIDController(dReal kp, dReal ki, dReal kd, dReal max_output = 100.0)
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output),
          integral_(0), prev_error_(0), first_update_(true) {}

    dReal compute(dReal error, dReal dt) {
        if (first_update_) {
            prev_error_ = error;
            first_update_ = false;
        }

        integral_ += error * dt;
        dReal derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        dReal output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        return clamp(output, -max_output_, max_output_);
    }

    void reset() {
        integral_ = 0;
        prev_error_ = 0;
        first_update_ = true;
    }

private:
    dReal kp_, ki_, kd_;
    dReal max_output_;
    dReal integral_;
    dReal prev_error_;
    bool first_update_;
};

// Path follower
class PathFollower {
public:
    PathFollower(dReal lookahead_distance = 0.5)
        : lookahead_distance_(lookahead_distance), current_waypoint_index_(0) {}

    void setPath(const std::vector<std::pair<dReal, dReal>>& waypoints) {
        waypoints_ = waypoints;
        current_waypoint_index_ = 0;
    }

    // Compute steering angle to follow path
    dReal computeSteeringAngle(dReal x, dReal y, dReal theta) {
        if (waypoints_.empty()) return 0.0;

        // Find target waypoint
        while (current_waypoint_index_ < waypoints_.size()) {
            dReal dx = waypoints_[current_waypoint_index_].first - x;
            dReal dy = waypoints_[current_waypoint_index_].second - y;
            dReal distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 0.3) {  // Reached waypoint
                current_waypoint_index_++;
            } else {
                break;
            }
        }

        if (current_waypoint_index_ >= waypoints_.size()) {
            return 0.0;  // Path completed
        }

        // Compute angle to target
        dReal dx = waypoints_[current_waypoint_index_].first - x;
        dReal dy = waypoints_[current_waypoint_index_].second - y;
        dReal target_angle = std::atan2(dy, dx);

        // Angle error
        dReal angle_error = target_angle - theta;
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        return angle_error;
    }

    bool isPathCompleted() const {
        return current_waypoint_index_ >= waypoints_.size();
    }

    void reset() {
        current_waypoint_index_ = 0;
    }

private:
    dReal lookahead_distance_;
    std::vector<std::pair<dReal, dReal>> waypoints_;
    size_t current_waypoint_index_;
};

} // namespace ode_tutorial

#endif // ROBOT_UTILS_H
