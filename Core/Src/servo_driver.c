/*
 * servo_driver.c
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

// Core/Src/servo_driver.c
#include "servo_driver.h"
#include "tim.h"
#include "robot_config.h"

void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    Servo_SetAngle(SERVO_CENTER_ANGLE);
}

void Servo_SetAngle(uint8_t angle) {
    if (angle < SERVO_LEFT_MAX) angle = SERVO_LEFT_MAX;
    if (angle > SERVO_RIGHT_MAX) angle = SERVO_RIGHT_MAX;

    // Map angle to pulse width
    uint32_t pulse_us = SERVO_PULSE_MIN_US +
        (((uint32_t)(angle - SERVO_LEFT_MAX) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US))
        / (SERVO_RIGHT_MAX - SERVO_LEFT_MAX));

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}
