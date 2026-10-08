
#ifndef MINI_WHEEL_LEGGED_ROBOT_IMU_INTERFACE_H
#define MINI_WHEEL_LEGGED_ROBOT_IMU_INTERFACE_H

typedef enum {
    IMU_GYRO_BIAS_CALIBRATE_ON_BOOT = 0,
    IMU_GYRO_BIAS_OFFLINE = 1
} imu_gyro_bias_mode_t;

#ifdef __cplusplus
#include "imu.h"
extern IMU body_imu;
extern "C" {
#endif

void imu_init(imu_gyro_bias_mode_t mode);
void imu_update(float dt);
void imu_get_quaternion(float *w, float *x, float *y, float *z);
void imu_get_euler_rad(float *roll, float *pitch, float *yaw);
void imu_get_euler_deg(float *roll, float *pitch, float *yaw);
void imu_get_gyro_bias_dps(float *gx, float *gy, float *gz);

#ifdef __cplusplus
}
#endif

#endif //MINI_WHEEL_LEGGED_ROBOT_IMU_INTERFACE_H
