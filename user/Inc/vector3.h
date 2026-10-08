
#ifndef MINI_WHEEL_LEGGED_ROBOT_VECTOR3_H
#define MINI_WHEEL_LEGGED_ROBOT_VECTOR3_H

#include <math.h>

struct Vector3 {
    float x, y, z;

    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& rhs) const { return Vector3(x + rhs.x, y + rhs.y, z + rhs.z); }
    Vector3 operator-(const Vector3& rhs) const { return Vector3(x - rhs.x, y - rhs.y, z - rhs.z); }
    Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }

    float dot(const Vector3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }

    Vector3 cross(const Vector3& rhs) const {
        return Vector3(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }

    float norm() const { return sqrtf(x * x + y * y + z * z); }

    Vector3 normalized() const {
        float n = norm();
        if (n < 1e-9f) {
            return Vector3(0.0f, 0.0f, 0.0f);
        }
        float inv = 1.0f / n;
        return Vector3(x * inv, y * inv, z * inv);
    }
};

#endif //MINI_WHEEL_LEGGED_ROBOT_VECTOR3_H
