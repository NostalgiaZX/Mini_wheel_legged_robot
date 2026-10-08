
#ifndef MINI_WHEEL_LEGGED_ROBOT_IMU_H
#define MINI_WHEEL_LEGGED_ROBOT_IMU_H

#include <stdint.h>

#include "quaternion.h"
#include "vector3.h"

enum class GyroBiasMode {
    kCalibrateOnBoot,
    kOffline
};

class IMU {
public:
    explicit IMU(float kp = 2.0f);
    bool data_ready=false;
    void init(GyroBiasMode mode = GyroBiasMode::kCalibrateOnBoot, uint32_t calibSamples = 1000);

    void update(float dt);

    const Quaternion& quaternion() const { return q_; }

    const EulerAngles& eulerRad() const { return eulerRad_; }
    const EulerAngles& eulerDeg() const { return eulerDeg_; }

    const Vector3& gyroBias() const { return gyroBias_; }

    void setKp(float kp) { kp_ = kp; }
    float kp() const { return kp_; }
    void read_start();

private:
    Quaternion q_;
    EulerAngles eulerRad_;
    EulerAngles eulerDeg_;
    Vector3 gyroBias_;
    float kp_;

    static Vector3 readGyroRadPerSec();
    static Vector3 readAccelG();
    void calibrateGyro(uint32_t samples);
    static const Vector3 kOfflineGyroBiasRad;
};

#endif //MINI_WHEEL_LEGGED_ROBOT_IMU_H
