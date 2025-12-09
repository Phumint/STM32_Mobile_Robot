/*
 * motor_driver.c
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */
#include "motor_driver.h"
#include "tim.h"
#include "gpio.h"
#include "robot_config.h"

// Helper to constrain values
static int8_t constrain_speed(int8_t speed) {
    if (speed > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (speed < MOTOR_MIN_SPEED) return MOTOR_MIN_SPEED;
    return speed;
}

void Motor_Init(void) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Motor_SetSpeed(int8_t speed) {
    speed = constrain_speed(speed);
    uint8_t abs_speed = (speed >= 0) ? speed : -speed;

    // Calculate PWM based on ARR (Period)
    uint32_t pwm_val = (abs_speed * __HAL_TIM_GET_AUTORELOAD(&htim3)) / 100;

    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_val);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

