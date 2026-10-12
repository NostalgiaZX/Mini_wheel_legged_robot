#include "../Inc/imu.h"
#include  "mpu6050.h"
#include "callbacks.h"
#include "i2c.h"
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == hi2c2.Instance) {
        if (mpu6050_reading_state == Accel) {
            mpu6050_reading_state = Gyro;
            if (HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*)gyro, 6) != HAL_OK) {
                mpu6050_reading_state = Idle;
            }
        } else if (mpu6050_reading_state == Gyro) {
            mpu6050_reading_state = Idle;
            reverseint16array(accel, 3);
            reverseint16array(gyro, 3);
            body_imu.data_ready = true;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == hi2c2.Instance) {
        mpu6050_reading_state = Idle;
        I2C2_BusRecovery();
        mpu6050_read_startdma();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}
