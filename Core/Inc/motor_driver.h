/*
 * motor_driver.h
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

/* Core/Inc/motor_driver.h */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * @brief Initialize the motor PWM channels.
 */
void Motor_Init(void);

/**
 * @brief Set the motor speed.
 * @param speed: Speed value from -100 (full reverse) to 100 (full forward).
 * 0 stops the motor.
 */
void Motor_SetSpeed(int8_t speed);

#endif /* MOTOR_DRIVER_H */
