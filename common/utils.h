#ifndef UTILS_H
#define UTILS_H

#include <ode/ode.h>
#include <cmath>
#include <iostream>

namespace ode_tutorial {

// Math utilities
inline double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

inline double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

inline double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Vector utilities
inline void vector3Set(dReal* vec, dReal x, dReal y, dReal z) {
    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
}

inline dReal vector3Length(const dReal* vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

inline void vector3Normalize(dReal* vec) {
    dReal len = vector3Length(vec);
    if (len > 0.0) {
        vec[0] /= len;
        vec[1] /= len;
        vec[2] /= len;
    }
}

inline dReal vector3Dot(const dReal* a, const dReal* b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void vector3Cross(dReal* result, const dReal* a, const dReal* b) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

inline dReal vector3Distance(const dReal* a, const dReal* b) {
    dReal dx = a[0] - b[0];
    dReal dy = a[1] - b[1];
    dReal dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Rotation utilities
inline void setRotationZ(dMatrix3& R, dReal angle) {
    dReal c = std::cos(angle);
    dReal s = std::sin(angle);
    R[0] = c;  R[1] = -s; R[2] = 0;  R[3] = 0;
    R[4] = s;  R[5] = c;  R[6] = 0;  R[7] = 0;
    R[8] = 0;  R[9] = 0;  R[10] = 1; R[11] = 0;
}

inline dReal getYawFromRotation(const dReal* R) {
    return std::atan2(R[4], R[0]);
}

// Body creation utilities
inline dBodyID createBox(dWorldID world, dSpaceID space,
                         dReal x, dReal y, dReal z,
                         dReal lx, dReal ly, dReal lz,
                         dReal mass, dGeomID* out_geom = nullptr) {
    dBodyID body = dBodyCreate(world);
    dBodySetPosition(body, x, y, z);

    dMass m;
    dMassSetBox(&m, 1, lx, ly, lz);
    dMassAdjust(&m, mass);
    dBodySetMass(body, &m);

    dGeomID geom = dCreateBox(space, lx, ly, lz);
    dGeomSetBody(geom, body);

    if (out_geom) *out_geom = geom;
    return body;
}

inline dBodyID createSphere(dWorldID world, dSpaceID space,
                            dReal x, dReal y, dReal z,
                            dReal radius,
                            dReal mass, dGeomID* out_geom = nullptr) {
    dBodyID body = dBodyCreate(world);
    dBodySetPosition(body, x, y, z);

    dMass m;
    dMassSetSphere(&m, 1, radius);
    dMassAdjust(&m, mass);
    dBodySetMass(body, &m);

    dGeomID geom = dCreateSphere(space, radius);
    dGeomSetBody(geom, body);

    if (out_geom) *out_geom = geom;
    return body;
}

inline dBodyID createCapsule(dWorldID world, dSpaceID space,
                             dReal x, dReal y, dReal z,
                             dReal radius, dReal length,
                             dReal mass, dGeomID* out_geom = nullptr) {
    dBodyID body = dBodyCreate(world);
    dBodySetPosition(body, x, y, z);

    dMass m;
    dMassSetCapsule(&m, 1, 3, radius, length);
    dMassAdjust(&m, mass);
    dBodySetMass(body, &m);

    dGeomID geom = dCreateCapsule(space, radius, length);
    dGeomSetBody(geom, body);

    if (out_geom) *out_geom = geom;
    return body;
}

inline dBodyID createCylinder(dWorldID world, dSpaceID space,
                              dReal x, dReal y, dReal z,
                              dReal radius, dReal length,
                              dReal mass, dGeomID* out_geom = nullptr) {
    dBodyID body = dBodyCreate(world);
    dBodySetPosition(body, x, y, z);

    dMass m;
    dMassSetCylinder(&m, 1, 3, radius, length);
    dMassAdjust(&m, mass);
    dBodySetMass(body, &m);

    dGeomID geom = dCreateCylinder(space, radius, length);
    dGeomSetBody(geom, body);

    if (out_geom) *out_geom = geom;
    return body;
}

// Print utilities
inline void printVector3(const char* name, const dReal* vec) {
    std::cout << name << ": [" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]" << std::endl;
}

inline void printPosition(dBodyID body, const char* name = "Position") {
    const dReal* pos = dBodyGetPosition(body);
    printVector3(name, pos);
}

inline void printQuaternion(const dReal* q, const char* name = "Quaternion") {
    std::cout << name << ": [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << std::endl;
}

} // namespace ode_tutorial

#endif // UTILS_H
