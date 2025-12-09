/*
 * robot_config.h
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

#ifndef INC_ROBOT_CONFIG_H_
#define INC_ROBOT_CONFIG_H_

// --- Servo Settings ---
#define SERVO_CENTER_ANGLE  90
#define SERVO_LEFT_MAX      50
#define SERVO_RIGHT_MAX     130
#define SERVO_PULSE_MIN_US  1000
#define SERVO_PULSE_MAX_US  2000

// --- Motor Settings ---
#define MOTOR_MAX_SPEED     100
#define MOTOR_MIN_SPEED    -100
#define MOTOR_PWM_MAX_VAL   1000 // Match your Timer ARR

// --- Comms ---
#define UART_RX_BUF_SIZE    64
#define SENSOR_PUB_RATE_HZ  10

#endif /* INC_ROBOT_CONFIG_H_ */
