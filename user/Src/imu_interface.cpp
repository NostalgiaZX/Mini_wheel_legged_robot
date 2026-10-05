
#include "../Inc/imu_interface.h"
#include "imu.h"

constexpr float kRad2Deg = 180.0f / 3.14159265358979323846f;

IMU g_imu;

void imu_init(imu_gyro_bias_mode_t mode) {

    GyroBiasMode m = (mode == IMU_GYRO_BIAS_OFFLINE) ? GyroBiasMode::kOffline : GyroBiasMode::kCalibrateOnBoot;
    g_imu.init(m);
}

void imu_update(float dt) {
    g_imu.update(dt);
}

void imu_get_quaternion(float *w, float *x, float *y, float *z) {
    const Quaternion &q = g_imu.quaternion();
    *w = q.w; *x = q.x; *y = q.y; *z = q.z;
}

void imu_get_euler_rad(float *roll, float *pitch, float *yaw) {
    const EulerAngles &e = g_imu.eulerRad();
    *roll = e.roll; *pitch = e.pitch; *yaw = e.yaw;
}

void imu_get_euler_deg(float *roll, float *pitch, float *yaw) {
    const EulerAngles &e = g_imu.eulerDeg();
    *roll = e.roll; *pitch = e.pitch; *yaw = e.yaw;
}

void imu_get_gyro_bias_dps(float *gx, float *gy, float *gz) {
    const Vector3 &b = g_imu.gyroBias();
    *gx = b.x * kRad2Deg; *gy = b.y * kRad2Deg; *gz = b.z * kRad2Deg;
}
