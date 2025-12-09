/*
 * ros_comms.h
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

/* Core/Inc/ros_comms.h */
#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#include <stdint.h>
#include "mpu6050.h" // Required for MPU6050_t struct definition

/**
 * @brief Initialize UART interrupts and send ready signal.
 */
void ROS_Comms_Init(void);

/**
 * @brief Check if a valid command packet has been received.
 * Non-blocking.
 * @param out_speed: Pointer to store parsed speed (-100 to 100).
 * @param out_angle: Pointer to store parsed angle (deg).
 */
void ROS_CheckForCommand(int8_t* out_speed, uint8_t* out_angle);

/**
 * @brief Send sensor data to ROS.
 * @param enc: Current encoder count.
 * @param mpu: Pointer to the MPU6050 data structure.
 */
void ROS_SendTelemetry(int32_t enc, MPU6050_t* mpu);

/**
 * @brief UART Receive Interrupt Handler.
 * Call this function inside HAL_UART_RxCpltCallback.
 */
void ROS_UART_ISR_Handler(void);

#endif /* ROS_COMMS_H */
