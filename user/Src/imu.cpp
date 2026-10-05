
#include "../Inc/imu.h"
#include "mpu6050.h"

constexpr float GYRO_LSB_PER_DPS = 16.4f;
constexpr float ACCEL_LSB_PER_G = 2048.0f;
constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
constexpr float RAD2DEG = 180.0f / 3.14159265358979323846f;

const Vector3 IMU::kOfflineGyroBiasRad(-0.0454367623f, -0.0448727049f, 0.0323835537f);

IMU::IMU(float kp) : kp_(kp) {}

void IMU::init(GyroBiasMode mode, uint32_t calibSamples) {
    mpu6050_init();

    if (mode == GyroBiasMode::kCalibrateOnBoot) {
        calibrateGyro(calibSamples);
    } else {
        gyroBias_ = kOfflineGyroBiasRad;
    }
}

void IMU::calibrateGyro(uint32_t samples) {
    if (samples == 0) {
        return;
    }

    Vector3 sum;
    for (uint32_t i = 0; i < samples; ++i) {
        mpu6050_read();
        sum = sum + readGyroRadPerSec();
    }
    gyroBias_ = sum * (1.0f / static_cast<float>(samples));
}

Vector3 IMU::readGyroRadPerSec() {
    return Vector3(
        static_cast<float>(gyro[0]) / GYRO_LSB_PER_DPS * DEG2RAD,
        static_cast<float>(gyro[1]) / GYRO_LSB_PER_DPS * DEG2RAD,
        static_cast<float>(gyro[2]) / GYRO_LSB_PER_DPS * DEG2RAD
    );
}

Vector3 IMU::readAccelG() {
    return Vector3(
        static_cast<float>(accel[0]) / ACCEL_LSB_PER_G,
        static_cast<float>(accel[1]) / ACCEL_LSB_PER_G,
        static_cast<float>(accel[2]) / ACCEL_LSB_PER_G
    );
}

void IMU::update(float dt) {
    //mpu6050_read();
    if (!data_ready) {
        return;
    }
    data_ready = false;
    read_start();
    Vector3 omega = readGyroRadPerSec() - gyroBias_;
    Vector3 a = readAccelG();

    if (a.dot(a) > 1e-6f) {
        a = a.normalized();

        Vector3 v(
            2.0f * (q_.x * q_.z - q_.w * q_.y),
            2.0f * (q_.w * q_.x + q_.y * q_.z),
            q_.w * q_.w - q_.x * q_.x - q_.y * q_.y + q_.z * q_.z
        );

        Vector3 e = a.cross(v);
        omega = omega + e * kp_;
    }

    q_.integrate(omega, dt);

    eulerRad_ = q_.toEuler();
    eulerDeg_.roll  = eulerRad_.roll  * RAD2DEG;
    eulerDeg_.pitch = eulerRad_.pitch * RAD2DEG;
    eulerDeg_.yaw   = eulerRad_.yaw   * RAD2DEG;
}
void IMU::read_start() {
    mpu6050_read_startdma();
}
