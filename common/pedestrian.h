#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

#include <ode/ode.h>
#include <vector>
#include <cmath>
#include "utils.h"

namespace ode_tutorial {

// Pedestrian model
class Pedestrian {
public:
    Pedestrian(dWorldID world, dSpaceID space, dReal x, dReal y, dReal z)
        : world_(world), space_(space), desired_speed_(1.0), radius_(0.3) {
        // Create capsule body for pedestrian
        body_ = dBodyCreate(world);
        dBodySetPosition(body_, x, y, z);

        dMass m;
        dMassSetCapsule(&m, 70.0, 3, radius_, 1.4);  // 70 kg human
        dBodySetMass(body_, &m);

        geom_ = dCreateCapsule(space, radius_, 1.4);
        dGeomSetBody(geom_, body_);

        // Initialize goal
        goal_x_ = x;
        goal_y_ = y;
    }

    ~Pedestrian() {
        if (body_) dBodyDestroy(body_);
        if (geom_) dGeomDestroy(geom_);
    }

    // Set desired goal position
    void setGoal(dReal x, dReal y) {
        goal_x_ = x;
        goal_y_ = y;
    }

    // Set desired walking speed
    void setDesiredSpeed(dReal speed) {
        desired_speed_ = speed;
    }

    // Update pedestrian using Social Force Model
    void update(dReal dt, const std::vector<Pedestrian*>& other_pedestrians) {
        const dReal* pos = dBodyGetPosition(body_);
        const dReal* vel = dBodyGetLinearVel(body_);

        // Goal attractive force
        dReal dx_goal = goal_x_ - pos[0];
        dReal dy_goal = goal_y_ - pos[1];
        dReal dist_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

        dReal fx = 0, fy = 0;

        if (dist_goal > 0.1) {
            dReal desired_vx = (dx_goal / dist_goal) * desired_speed_;
            dReal desired_vy = (dy_goal / dist_goal) * desired_speed_;

            // Relaxation term (trying to reach desired velocity)
            dReal tau = 0.5;  // Relaxation time
            fx = (desired_vx - vel[0]) / tau;
            fy = (desired_vy - vel[1]) / tau;
        }

        // Repulsive force from other pedestrians
        for (const auto& other : other_pedestrians) {
            if (other == this) continue;

            const dReal* other_pos = other->getPosition();
            dReal dx = pos[0] - other_pos[0];
            dReal dy = pos[1] - other_pos[1];
            dReal dist = std::sqrt(dx * dx + dy * dy);

            if (dist < 2.0 && dist > 0.0) {  // Interaction range
                dReal force_magnitude = 2.0 * std::exp((2 * radius_ - dist) / 0.3);
                fx += force_magnitude * (dx / dist);
                fy += force_magnitude * (dy / dist);
            }
        }

        // Apply force (scaled by mass)
        const dMass* mass = dBodyGetMass(body_);
        dBodyAddForce(body_, fx * mass->mass, fy * mass->mass, 0);

        // Limit vertical movement
        dReal vz = vel[2];
        if (std::abs(vz) > 0.1) {
            dBodySetLinearVel(body_, vel[0], vel[1], 0);
        }
    }

    // Simple waypoint follower
    void followWaypoint(dReal target_x, dReal target_y) {
        const dReal* pos = dBodyGetPosition(body_);
        const dReal* vel = dBodyGetLinearVel(body_);

        dReal dx = target_x - pos[0];
        dReal dy = target_y - pos[1];
        dReal dist = std::sqrt(dx * dx + dy * dy);

        if (dist > 0.1) {
            dReal desired_vx = (dx / dist) * desired_speed_;
            dReal desired_vy = (dy / dist) * desired_speed_;

            dReal tau = 0.5;
            dReal fx = (desired_vx - vel[0]) / tau;
            dReal fy = (desired_vy - vel[1]) / tau;

            const dMass* mass = dBodyGetMass(body_);
            dBodyAddForce(body_, fx * mass->mass, fy * mass->mass, 0);
        }
    }

    // Getters
    dBodyID getBody() const { return body_; }
    dGeomID getGeom() const { return geom_; }
    const dReal* getPosition() const { return dBodyGetPosition(body_); }
    const dReal* getVelocity() const { return dBodyGetLinearVel(body_); }
    dReal getRadius() const { return radius_; }

    // Check if goal is reached
    bool isGoalReached() const {
        const dReal* pos = dBodyGetPosition(body_);
        dReal dx = goal_x_ - pos[0];
        dReal dy = goal_y_ - pos[1];
        dReal dist = std::sqrt(dx * dx + dy * dy);
        return dist < 0.3;
    }

private:
    dWorldID world_;
    dSpaceID space_;
    dBodyID body_;
    dGeomID geom_;
    dReal goal_x_, goal_y_;
    dReal desired_speed_;
    dReal radius_;
};

// Crowd manager
class CrowdManager {
public:
    CrowdManager(dWorldID world, dSpaceID space)
        : world_(world), space_(space) {}

    ~CrowdManager() {
        for (auto ped : pedestrians_) {
            delete ped;
        }
    }

    Pedestrian* addPedestrian(dReal x, dReal y, dReal z = 0.8) {
        Pedestrian* ped = new Pedestrian(world_, space_, x, y, z);
        pedestrians_.push_back(ped);
        return ped;
    }

    void update(dReal dt) {
        for (auto ped : pedestrians_) {
            ped->update(dt, pedestrians_);
        }
    }

    const std::vector<Pedestrian*>& getPedestrians() const {
        return pedestrians_;
    }

    size_t getCount() const {
        return pedestrians_.size();
    }

private:
    dWorldID world_;
    dSpaceID space_;
    std::vector<Pedestrian*> pedestrians_;
};

} // namespace ode_tutorial

#endif // PEDESTRIAN_H
