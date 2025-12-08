/*
 * mpu6050.h
 *
 * MPU6050 IMU Driver for STM32
 * Place this file in: Core/Inc/
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

// MPU6050 I2C Address (AD0 pin low = 0x68, AD0 pin high = 0x69)
#define MPU6050_ADDR 0xD0  // 0x68 << 1 for HAL I2C

// MPU6050 Register Addresses
#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43

// Accelerometer sensitivity (LSB/g)
#define MPU6050_ACCEL_SENS_2G     16384.0f
#define MPU6050_ACCEL_SENS_4G     8192.0f
#define MPU6050_ACCEL_SENS_8G     4096.0f
#define MPU6050_ACCEL_SENS_16G    2048.0f

// Gyroscope sensitivity (LSB/°/s)
#define MPU6050_GYRO_SENS_250     131.0f
#define MPU6050_GYRO_SENS_500     65.5f
#define MPU6050_GYRO_SENS_1000    32.8f
#define MPU6050_GYRO_SENS_2000    16.4f

// MPU6050 Data Structure
typedef struct {
    // Raw data
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    // Processed data (real units)
    float accel_x;  // g
    float accel_y;  // g
    float accel_z;  // g
    float temp;     // °C
    float gyro_x;   // °/s
    float gyro_y;   // °/s
    float gyro_z;   // °/s

    // Calculated angles (optional)
    float roll;     // degrees
    float pitch;    // degrees

} MPU6050_t;

// Function Prototypes
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);
uint8_t MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);
uint8_t MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);
uint8_t MPU6050_Read_Temp(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);
void MPU6050_Calculate_Angles(MPU6050_t *mpu);

#endif /* MPU6050_H_ */
