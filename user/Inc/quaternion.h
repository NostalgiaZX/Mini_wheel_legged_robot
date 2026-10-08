
#ifndef MINI_WHEEL_LEGGED_ROBOT_QUATERNION_H
#define MINI_WHEEL_LEGGED_ROBOT_QUATERNION_H

#include <math.h>
#include "vector3.h"

struct EulerAngles {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    void normalize() {
        float n = sqrtf(w * w + x * x + y * y + z * z);
        if (n < 1e-9f) {
            return;
        }
        float inv = 1.0f / n;
        w *= inv; x *= inv; y *= inv; z *= inv;
    }

    void integrate(const Vector3& omega, float dt) {
        float halfDt = 0.5f * dt;
        float qw = w, qx = x, qy = y, qz = z;

        w += (-qx * omega.x - qy * omega.y - qz * omega.z) * halfDt;
        x += ( qw * omega.x + qy * omega.z - qz * omega.y) * halfDt;
        y += ( qw * omega.y - qx * omega.z + qz * omega.x) * halfDt;
        z += ( qw * omega.z + qx * omega.y - qy * omega.x) * halfDt;

        normalize();
    }

    EulerAngles toEuler() const {
        EulerAngles e;

        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        e.roll = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (w * y - z * x);
        if (sinp > 1.0f) sinp = 1.0f;
        if (sinp < -1.0f) sinp = -1.0f;
        e.pitch = asinf(sinp);

        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        e.yaw = atan2f(siny_cosp, cosy_cosp);

        return e;
    }
};

#endif //MINI_WHEEL_LEGGED_ROBOT_QUATERNION_H
