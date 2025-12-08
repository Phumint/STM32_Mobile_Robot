///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : motor_control.c
//  * @brief          : Motor PWM control module
//  ******************************************************************************
//  */
///* USER CODE END Header */
//
//#include "motor_control.h"
//
///* USER CODE BEGIN 0 */
//
//static TIM_HandleTypeDef *motor_tim = NULL;
//static uint32_t motor_channel = 0;
//
//// Robot-specific maximum speed (mm/s)
//#define MAX_SPEED_MM_S 500   // adjust based on your robot
//#define PWM_MAX         999  // For TIM in PWM mode 1kHz (Period = 999)
//
///* USER CODE END 0 */
//
///**
//  * @brief  Initializes the motor PWM channel.
//  * @param  htim: PWM timer
//  * @param  channel: Timer PWM channel
//  * @retval None
//  */
//void Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel)
//{
//    /* USER CODE BEGIN Motor_Init */
//    motor_tim = htim;
//    motor_channel = channel;
//    HAL_TIM_PWM_Start(motor_tim, motor_channel);
//    /* USER CODE END Motor_Init */
//}
//
///**
//  * @brief  Set motor speed in mm/s.
//  * @param  speed_mm_s: signed integer speed
//  * @retval None
//  */
//void Motor_SetSpeed(int16_t speed_mm_s)
//{
//    /* USER CODE BEGIN Motor_SetSpeed */
//
//    if (speed_mm_s > MAX_SPEED_MM_S) speed_mm_s = MAX_SPEED_MM_S;
//    if (speed_mm_s < -MAX_SPEED_MM_S) speed_mm_s = -MAX_SPEED_MM_S;
//
//    int pwm;
//    pwm = (speed_mm_s * PWM_MAX) / MAX_SPEED_MM_S;
//
//    // Map negative speeds to reversed direction (optional depending on motor driver)
//    // If your motor driver uses separate DIR pin, set it here.
//
//    __HAL_TIM_SET_COMPARE(motor_tim, motor_channel, abs(pwm));
//
//    /* USER CODE END Motor_SetSpeed */
//}
//
///**
//  * @brief Immediately stop motor output.
//  * @retval None
//  */
//void Motor_Stop(void)
//{
//    /* USER CODE BEGIN Motor_Stop */
//    __HAL_TIM_SET_COMPARE(motor_tim, motor_channel, 0);
//    /* USER CODE END Motor_Stop */
//}
