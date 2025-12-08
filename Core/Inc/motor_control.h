/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_control.h
  * @brief          : Header for motor control module
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"

void Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Motor_SetSpeed(int16_t speed_mm_s);
void Motor_Stop(void);

#endif /* INC_MOTOR_CONTROL_H_ */
