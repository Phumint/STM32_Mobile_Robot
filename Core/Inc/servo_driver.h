/*
 * servo_driver.h
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

/* Core/Inc/servo_driver.h */
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

/**
 * @brief Initialize the servo PWM channel and set to center.
 */
void Servo_Init(void);

/**
 * @brief Set the servo angle.
 * @param angle: Angle in degrees.
 * Clamped between SERVO_LEFT_MAX and SERVO_RIGHT_MAX
 * defined in robot_config.h.
 */
void Servo_SetAngle(uint8_t angle);

#endif /* SERVO_DRIVER_H */
