/*
 * ros_comms.c
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

#include "ros_comms.h"
#include "usart.h"
#include "robot_config.h"
#include <stdio.h>
#include <string.h>

static uint8_t rx_byte;
static uint8_t rx_buffer[UART_RX_BUF_SIZE];
static uint16_t rx_index = 0;
static volatile uint8_t msg_received_flag = 0;

void ROS_Comms_Init(void) {
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_UART_Transmit(&huart1, (uint8_t*)"$READY*\r\n", 9, 100);
}

// Called from Main Loop
void ROS_CheckForCommand(int8_t* out_speed, uint8_t* out_angle) {
    if (msg_received_flag) {
        int speed_temp = 0;
        int angle_temp = SERVO_CENTER_ANGLE;

        // Safety: Ensure string termination
        rx_buffer[UART_RX_BUF_SIZE - 1] = '\0';

        if (strncmp((char*)rx_buffer, "#CMD,", 5) == 0) {
            sscanf((char*)&rx_buffer[5], "%d,%d", &speed_temp, &angle_temp);
            *out_speed = (int8_t)speed_temp;
            *out_angle = (uint8_t)angle_temp;
        }
        else if (strncmp((char*)rx_buffer, "#STOP", 5) == 0) {
            *out_speed = 0;
        }

        // Reset buffer and flag
        msg_received_flag = 0;
        memset(rx_buffer, 0, UART_RX_BUF_SIZE);
        rx_index = 0;
    }
}

void ROS_SendTelemetry(int32_t enc, MPU6050_t* mpu) {
    char buffer[128];
    // Keep format concise
    sprintf(buffer, "$ENC,%ld;IMU,%d,%d,%.2f*\r\n",
            enc,
            (int)(mpu->accel_x * 100),
            (int)(mpu->gyro_z * 100),
            mpu->roll); // Example subset

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 50);
}

// Call this from HAL_UART_RxCpltCallback
void ROS_UART_ISR_Handler(void) {
    if (rx_index < UART_RX_BUF_SIZE - 1) {
        if (rx_byte == '*') {
            // End of message
            rx_buffer[rx_index] = '\0';
            msg_received_flag = 1;
        } else {
            rx_buffer[rx_index++] = rx_byte;
        }
    } else {
        // Buffer overflow protection
        rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

