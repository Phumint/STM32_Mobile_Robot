/*
 * uart_comm.c
 *
 * UART Communication Protocol Implementation
 * Place this file in: Core/Src/
 */

#include "uart_comm.h"
#include <string.h>

// UART receive buffers
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_rx_data;
volatile uint16_t uart_rx_index = 0;
volatile uint8_t uart_data_ready = 0;

static UART_HandleTypeDef *uart_handle;

// Initialize UART communication
void UART_Init(UART_HandleTypeDef *huart)
{
    uart_handle = huart;
    uart_rx_index = 0;
    uart_data_ready = 0;

    // Start receiving data in interrupt mode
    HAL_UART_Receive_IT(huart, &uart_rx_data, 1);
}

// Calculate simple checksum
uint8_t UART_CalculateChecksum(uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0;
    for(uint16_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

// Parse received command
UART_Command_t UART_ParseCommand(uint8_t *buffer, uint16_t length)
{
    UART_Command_t cmd = {0};
    cmd.valid = 0;

    // Minimum packet: START + CMD + DATA1 + DATA2 + CHECKSUM + END = 6 bytes
    if(length < 6)
        return cmd;

    // Check start and end bytes
    if(buffer[0] != UART_START_BYTE || buffer[length-1] != UART_END_BYTE)
        return cmd;

    // Verify checksum (all bytes except start, end, and checksum itself)
    uint8_t received_checksum = buffer[length-2];
    uint8_t calculated_checksum = UART_CalculateChecksum(&buffer[1], length-3);

    if(received_checksum != calculated_checksum)
        return cmd;

    // Extract command and data
    cmd.command = buffer[1];
    cmd.data1 = buffer[2];
    cmd.data2 = buffer[3];
    cmd.valid = 1;

    return cmd;
}

// Process received data in main loop
void UART_ProcessReceivedData(void)
{
    if(uart_data_ready)
    {
        uart_data_ready = 0;

        // Parse the command
        UART_Command_t cmd = UART_ParseCommand(uart_rx_buffer, uart_rx_index);

        if(cmd.valid)
        {
            // Command will be processed in main.c
            // Set a flag or callback here
        }

        // Reset buffer for next command
        uart_rx_index = 0;
    }
}

// Send encoder data
void UART_SendEncoderData(UART_HandleTypeDef *huart, int32_t encoder_pos)
{
    uint8_t tx_buffer[12];
    uint8_t index = 0;

    tx_buffer[index++] = UART_START_BYTE;
    tx_buffer[index++] = DATA_ENCODER;

    // Split int32_t into 4 bytes
    tx_buffer[index++] = (encoder_pos >> 24) & 0xFF;
    tx_buffer[index++] = (encoder_pos >> 16) & 0xFF;
    tx_buffer[index++] = (encoder_pos >> 8) & 0xFF;
    tx_buffer[index++] = encoder_pos & 0xFF;

    // Calculate checksum
    tx_buffer[index] = UART_CalculateChecksum(&tx_buffer[1], index-1);
    index++;

    tx_buffer[index++] = UART_END_BYTE;

    HAL_UART_Transmit(huart, tx_buffer, index, 100);
}

// Send IMU data
void UART_SendIMUData(UART_HandleTypeDef *huart, MPU6050_t *mpu)
{
    uint8_t tx_buffer[64];
    uint8_t index = 0;

    tx_buffer[index++] = UART_START_BYTE;
    tx_buffer[index++] = DATA_IMU;

    // Pack float data as bytes (simple method - send as int16 scaled values)
    int16_t accel_x_int = (int16_t)(mpu->accel_x * 1000);
    int16_t accel_y_int = (int16_t)(mpu->accel_y * 1000);
    int16_t accel_z_int = (int16_t)(mpu->accel_z * 1000);
    int16_t gyro_x_int = (int16_t)(mpu->gyro_x * 10);
    int16_t gyro_y_int = (int16_t)(mpu->gyro_y * 10);
    int16_t gyro_z_int = (int16_t)(mpu->gyro_z * 10);
    int16_t roll_int = (int16_t)(mpu->roll * 10);
    int16_t pitch_int = (int16_t)(mpu->pitch * 10);
    int16_t temp_int = (int16_t)(mpu->temp * 10);

    // Accel X
    tx_buffer[index++] = (accel_x_int >> 8) & 0xFF;
    tx_buffer[index++] = accel_x_int & 0xFF;

    // Accel Y
    tx_buffer[index++] = (accel_y_int >> 8) & 0xFF;
    tx_buffer[index++] = accel_y_int & 0xFF;

    // Accel Z
    tx_buffer[index++] = (accel_z_int >> 8) & 0xFF;
    tx_buffer[index++] = accel_z_int & 0xFF;

    // Gyro X
    tx_buffer[index++] = (gyro_x_int >> 8) & 0xFF;
    tx_buffer[index++] = gyro_x_int & 0xFF;

    // Gyro Y
    tx_buffer[index++] = (gyro_y_int >> 8) & 0xFF;
    tx_buffer[index++] = gyro_y_int & 0xFF;

    // Gyro Z
    tx_buffer[index++] = (gyro_z_int >> 8) & 0xFF;
    tx_buffer[index++] = gyro_z_int & 0xFF;

    // Roll
    tx_buffer[index++] = (roll_int >> 8) & 0xFF;
    tx_buffer[index++] = roll_int & 0xFF;

    // Pitch
    tx_buffer[index++] = (pitch_int >> 8) & 0xFF;
    tx_buffer[index++] = pitch_int & 0xFF;

    // Temperature
    tx_buffer[index++] = (temp_int >> 8) & 0xFF;
    tx_buffer[index++] = temp_int & 0xFF;

    // Calculate checksum
    tx_buffer[index] = UART_CalculateChecksum(&tx_buffer[1], index-1);
    index++;

    tx_buffer[index++] = UART_END_BYTE;

    HAL_UART_Transmit(huart, tx_buffer, index, 100);
}

// Send all sensor data
void UART_SendAllSensorData(UART_HandleTypeDef *huart, int32_t encoder_pos, MPU6050_t *mpu)
{
    uint8_t tx_buffer[64];
    uint8_t index = 0;

    tx_buffer[index++] = UART_START_BYTE;
    tx_buffer[index++] = DATA_ALL_SENSORS;

    // Encoder (4 bytes)
    tx_buffer[index++] = (encoder_pos >> 24) & 0xFF;
    tx_buffer[index++] = (encoder_pos >> 16) & 0xFF;
    tx_buffer[index++] = (encoder_pos >> 8) & 0xFF;
    tx_buffer[index++] = encoder_pos & 0xFF;

    // IMU data (same as SendIMUData)
    int16_t accel_x_int = (int16_t)(mpu->accel_x * 1000);
    int16_t accel_y_int = (int16_t)(mpu->accel_y * 1000);
    int16_t accel_z_int = (int16_t)(mpu->accel_z * 1000);
    int16_t gyro_x_int = (int16_t)(mpu->gyro_x * 10);
    int16_t gyro_y_int = (int16_t)(mpu->gyro_y * 10);
    int16_t gyro_z_int = (int16_t)(mpu->gyro_z * 10);
    int16_t roll_int = (int16_t)(mpu->roll * 10);
    int16_t pitch_int = (int16_t)(mpu->pitch * 10);
    int16_t temp_int = (int16_t)(mpu->temp * 10);

    tx_buffer[index++] = (accel_x_int >> 8) & 0xFF;
    tx_buffer[index++] = accel_x_int & 0xFF;
    tx_buffer[index++] = (accel_y_int >> 8) & 0xFF;
    tx_buffer[index++] = accel_y_int & 0xFF;
    tx_buffer[index++] = (accel_z_int >> 8) & 0xFF;
    tx_buffer[index++] = accel_z_int & 0xFF;
    tx_buffer[index++] = (gyro_x_int >> 8) & 0xFF;
    tx_buffer[index++] = gyro_x_int & 0xFF;
    tx_buffer[index++] = (gyro_y_int >> 8) & 0xFF;
    tx_buffer[index++] = gyro_y_int & 0xFF;
    tx_buffer[index++] = (gyro_z_int >> 8) & 0xFF;
    tx_buffer[index++] = gyro_z_int & 0xFF;
    tx_buffer[index++] = (roll_int >> 8) & 0xFF;
    tx_buffer[index++] = roll_int & 0xFF;
    tx_buffer[index++] = (pitch_int >> 8) & 0xFF;
    tx_buffer[index++] = pitch_int & 0xFF;
    tx_buffer[index++] = (temp_int >> 8) & 0xFF;
    tx_buffer[index++] = temp_int & 0xFF;

    // Calculate checksum
    tx_buffer[index] = UART_CalculateChecksum(&tx_buffer[1], index-1);
    index++;

    tx_buffer[index++] = UART_END_BYTE;

    HAL_UART_Transmit(huart, tx_buffer, index, 100);
}
