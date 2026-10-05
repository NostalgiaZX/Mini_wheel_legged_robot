
#include "../Inc/mpu6050.h"
#include "i2c.h"

int16_t accel[3];
int16_t gyro[3];
ReadingState mpu6050_reading_state=Idle;
uint8_t mpu6050_check() {
    uint8_t id = 0x00;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&id,1,100);
    return id == MPU6050_ID;
}

void mpu6050_init() {
    uint8_t data = 0x01;
    HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,MPU6050_PWR_MGMT_1,I2C_MEMADD_SIZE_8BIT,&data,1,100);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,MPU6050_PWR_MGMT_2,I2C_MEMADD_SIZE_8BIT,&data,1,100);
    data = 0x09;
    HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,MPU6050_SMPLRT_DIV,I2C_MEMADD_SIZE_8BIT,&data,1,100);
    data = 0x06;
    HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,MPU6050_CONFIG,I2C_MEMADD_SIZE_8BIT,&data,1,100);
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_CONFIG,I2C_MEMADD_SIZE_8BIT,&data,1,100);
    HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG,I2C_MEMADD_SIZE_8BIT,&data,1,100);

    if (!mpu6050_check()) {
        Error_Handler();
    }
}

void mpu6050_read() {
    uint8_t datah = 0x00;
    uint8_t datal = 0x00;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H,I2C_MEMADD_SIZE_8BIT,&datah,1,100);
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_L,I2C_MEMADD_SIZE_8BIT,&datal,1,100);
    accel[0]=(datah<<8)|datal;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_H,I2C_MEMADD_SIZE_8BIT,&datah,1,100);
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_L,I2C_MEMADD_SIZE_8BIT,&datal,1,100);
    accel[1]=(datah<<8)|datal;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_ZOUT_H,I2C_MEMADD_SIZE_8BIT,&datah,1,100);
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_ZOUT_L,I2C_MEMADD_SIZE_8BIT,&datal,1,100);
    accel[2]=(datah<<8)|datal;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_XOUT_H,I2C_MEMADD_SIZE_8BIT,&datah,1,100);
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_XOUT_L,I2C_MEMADD_SIZE_8BIT,&datal,1,100);
    gyro[0]=(datah<<8)|datal;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_YOUT_H,I2C_MEMADD_SIZE_8BIT,&datah,1,100);
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_YOUT_L,I2C_MEMADD_SIZE_8BIT,&datal,1,100);
    gyro[1]=(datah<<8)|datal;
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_ZOUT_H,I2C_MEMADD_SIZE_8BIT,&datah,1,100);
    HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,MPU6050_GYRO_ZOUT_L,I2C_MEMADD_SIZE_8BIT,&datal,1,100);
    gyro[2]=(datah<<8)|datal;
}
void mpu6050_read_startdma() {
    if (mpu6050_reading_state == Idle) {
        mpu6050_reading_state = Accel;
        HAL_I2C_Mem_Read_DMA(&hi2c2,MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H,I2C_MEMADD_SIZE_8BIT,(uint8_t*)accel,6);
    }
}
void reverseint16array(int16_t* array,uint8_t num) {
    uint8_t* datah;
    uint8_t* datal;
    for (uint8_t i=0;i<num;i++) {
        datah=(uint8_t*)&array[i];
        datal=datah+1;
        uint8_t temp=*datah;
        *datah=*datal;
        *datal=temp;
    }
}
