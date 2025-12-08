/*
 * uart_comm.h
 *
 * UART Communication Protocol for RC Car
 * Place this file in: Core/Inc/
 */

#ifndef UART_COMM_H_
#define UART_COMM_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "mpu6050.h"

// Protocol Bytes
#define UART_START_BYTE   0xAA
#define UART_END_BYTE     0x55

// Command IDs (received from Jetson/Arduino)
#define CMD_SET_MOTOR     0x01
#define CMD_SET_SERVO     0x02
#define CMD_REQUEST_DATA  0x03
#define CMD_STOP_ALL      0x04

// Data Type IDs (sent to Jetson/Arduino)
#define DATA_ENCODER      0x10
#define DATA_IMU          0x20
#define DATA_ALL_SENSORS  0x30

// Buffer sizes
#define UART_RX_BUFFER_SIZE  64
#define UART_TX_BUFFER_SIZE  128

// Command structure
typedef struct {
    uint8_t command;
    uint8_t data1;
    uint8_t data2;
    uint8_t valid;
} UART_Command_t;

// Sensor data structure for transmission
typedef struct {
    int32_t encoder_position;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float temp;
} UART_SensorData_t;

// Function prototypes
void UART_Init(UART_HandleTypeDef *huart);
void UART_ProcessReceivedData(void);
UART_Command_t UART_ParseCommand(uint8_t *buffer, uint16_t length);
void UART_SendEncoderData(UART_HandleTypeDef *huart, int32_t encoder_pos);
void UART_SendIMUData(UART_HandleTypeDef *huart, MPU6050_t *mpu);
void UART_SendAllSensorData(UART_HandleTypeDef *huart, int32_t encoder_pos, MPU6050_t *mpu);
uint8_t UART_CalculateChecksum(uint8_t *data, uint16_t length);

// External variables
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern uint8_t uart_rx_data;
extern volatile uint16_t uart_rx_index;
extern volatile uint8_t uart_data_ready;

#endif /* UART_COMM_H_ */
