/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : servo_control.h
  * @brief          : Header for servo steering control module
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

#include "main.h"

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(int16_t steer_mdeg);

#endif /* INC_SERVO_CONTROL_H_ */
