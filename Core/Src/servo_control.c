///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : servo_control.c
//  * @brief          : Servo steering PWM module
//  ******************************************************************************
//  */
///* USER CODE END Header */
//
//#include "servo_control.h"
//
///* USER CODE BEGIN 0 */
//
//static TIM_HandleTypeDef *servo_tim = NULL;
//static uint32_t servo_channel = 0;
//
//// typical hobby servo parameters
//#define SERVO_MIN_US     1000
//#define SERVO_MAX_US     2000
//#define SERVO_CENTER_US  1500
//
//// steering limit: ±30 degrees = 30000 millidegrees
//#define SERVO_MAX_MDEG   30000
//
///* USER CODE END 0 */
//
///**
//  * @brief Initialize the servo PWM generator.
//  */
//void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel)
//{
//    /* USER CODE BEGIN Servo_Init */
//    servo_tim = htim;
//    servo_channel = channel;
//    HAL_TIM_PWM_Start(servo_tim, servo_channel);
//    /* USER CODE END Servo_Init */
//}
//
///**
//  * @brief Set servo angle in millidegrees.
//  */
//void Servo_SetAngle(int16_t steer_mdeg)
//{
//    /* USER CODE BEGIN Servo_SetAngle */
//
//    if (steer_mdeg > SERVO_MAX_MDEG) steer_mdeg = SERVO_MAX_MDEG;
//    if (steer_mdeg < -SERVO_MAX_MDEG) steer_mdeg = -SERVO_MAX_MDEG;
//
//    // Map ±30deg to 1000–2000 us
//    int pulse_us = SERVO_CENTER_US +
//                   (steer_mdeg * (SERVO_MAX_US - SERVO_MIN_US) / (SERVO_MAX_MDEG * 2));
//
//    __HAL_TIM_SET_COMPARE(servo_tim, servo_channel, pulse_us);
//
//    /* USER CODE END Servo_SetAngle */
//}
