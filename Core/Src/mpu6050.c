/*
 * mpu6050.c
 *
 * MPU6050 IMU Driver Implementation
 * Place this file in: Core/Src/
 */

#include "mpu6050.h"
#include <math.h>

// I2C timeout
#define I2C_TIMEOUT 100

// Initialize MPU6050
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    // Check device ID (WHO_AM_I register should return 0x68)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1, I2C_TIMEOUT);

    if (check == 0x68)  // 0x68 is the correct device ID
    {
        // Wake up MPU6050 (clear sleep mode bit)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT);

        // Set sample rate to 1kHz
        data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, I2C_TIMEOUT);

        // Set accelerometer configuration (±2g)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, I2C_TIMEOUT);

        // Set gyroscope configuration (±250°/s)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, I2C_TIMEOUT);

        // Set digital low pass filter
        data = 0x03;  // ~44Hz bandwidth
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_CONFIG, 1, &data, 1, I2C_TIMEOUT);

        return 0;  // Success
    }
    return 1;  // Failed
}

// Read all sensor data (accel, gyro, temp)
uint8_t MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[14];

    // Read 14 bytes starting from ACCEL_XOUT_H register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, data, 14, I2C_TIMEOUT);

    if (status != HAL_OK)
        return 1;  // Read failed

    // Combine high and low bytes for each sensor
    mpu->accel_x_raw = (int16_t)((data[0] << 8) | data[1]);
    mpu->accel_y_raw = (int16_t)((data[2] << 8) | data[3]);
    mpu->accel_z_raw = (int16_t)((data[4] << 8) | data[5]);
    mpu->temp_raw    = (int16_t)((data[6] << 8) | data[7]);
    mpu->gyro_x_raw  = (int16_t)((data[8] << 8) | data[9]);
    mpu->gyro_y_raw  = (int16_t)((data[10] << 8) | data[11]);
    mpu->gyro_z_raw  = (int16_t)((data[12] << 8) | data[13]);

    // Convert to real units
    mpu->accel_x = mpu->accel_x_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_y = mpu->accel_y_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_z = mpu->accel_z_raw / MPU6050_ACCEL_SENS_2G;

    mpu->temp = (mpu->temp_raw / 340.0f) + 36.53f;  // Temperature in °C

    mpu->gyro_x = mpu->gyro_x_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_y = mpu->gyro_y_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_z = mpu->gyro_z_raw / MPU6050_GYRO_SENS_250;

    return 0;  // Success
}

// Read accelerometer only
uint8_t MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[6];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, data, 6, I2C_TIMEOUT);

    if (status != HAL_OK)
        return 1;

    mpu->accel_x_raw = (int16_t)((data[0] << 8) | data[1]);
    mpu->accel_y_raw = (int16_t)((data[2] << 8) | data[3]);
    mpu->accel_z_raw = (int16_t)((data[4] << 8) | data[5]);

    mpu->accel_x = mpu->accel_x_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_y = mpu->accel_y_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_z = mpu->accel_z_raw / MPU6050_ACCEL_SENS_2G;

    return 0;
}

// Read gyroscope only
uint8_t MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[6];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H, 1, data, 6, I2C_TIMEOUT);

    if (status != HAL_OK)
        return 1;

    mpu->gyro_x_raw = (int16_t)((data[0] << 8) | data[1]);
    mpu->gyro_y_raw = (int16_t)((data[2] << 8) | data[3]);
    mpu->gyro_z_raw = (int16_t)((data[4] << 8) | data[5]);

    mpu->gyro_x = mpu->gyro_x_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_y = mpu->gyro_y_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_z = mpu->gyro_z_raw / MPU6050_GYRO_SENS_250;

    return 0;
}

// Read temperature only
uint8_t MPU6050_Read_Temp(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[2];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H, 1, data, 2, I2C_TIMEOUT);

    if (status != HAL_OK)
        return 1;

    mpu->temp_raw = (int16_t)((data[0] << 8) | data[1]);
    mpu->temp = (mpu->temp_raw / 340.0f) + 36.53f;

    return 0;
}

// Calculate roll and pitch angles from accelerometer
void MPU6050_Calculate_Angles(MPU6050_t *mpu)
{
    // Roll (rotation around X-axis)
    mpu->roll = atan2(mpu->accel_y, mpu->accel_z) * 180.0f / M_PI;

    // Pitch (rotation around Y-axis)
    mpu->pitch = atan2(-mpu->accel_x, sqrt(mpu->accel_y * mpu->accel_y + mpu->accel_z * mpu->accel_z)) * 180.0f / M_PI;
}
