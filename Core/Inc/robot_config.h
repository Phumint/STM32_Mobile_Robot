/* Core/Inc/robot_config.h */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// --- Physical Constants ---
#define WHEEL_DIAMETER          0.064f
#define COUNTS_PER_REV          979.0f
#define MAX_LINEAR_VELOCITY     0.5f

// --- PID Control ---
#define CONTROL_LOOP_FREQ       200.0f
#define DT                      (1.0f / CONTROL_LOOP_FREQ)
#define PID_KP                  80.0f
#define PID_KI                  150.0f
#define PID_KD                  2.0f
#define VEL_FILTER_ALPHA        0.4f

// --- Motor Driver Limits ---
#define MOTOR_MAX_SPEED         100      // Max PWM Duty Cycle (%)
#define MOTOR_MIN_SPEED         -100     // Min PWM Duty Cycle (%)
#define MAX_PWM_OUTPUT          100.0f   // For PID Calculation
#define MIN_PWM_OUTPUT          -100.0f

// --- Servo Configuration ---
// We set the logical range to 0-180 to map correctly to the servo's physical range.
// Your Python code handles the steering safety limits (e.g., 50-130).
#define SERVO_CENTER_ANGLE      90
#define SERVO_LEFT_MAX          0
#define SERVO_RIGHT_MAX         180
// Standard Servo Pulse Widths (in microseconds)
// 0 deg = 500us, 180 deg = 2500us (Standard for MG996R/SG90)
#define SERVO_PULSE_MIN_US      500
#define SERVO_PULSE_MAX_US      2500

// --- Communication Settings ---
#define UART_RX_BUF_SIZE        64       // Size of the Serial buffer
#define SENSOR_PUB_RATE_HZ      10       // How often to send telemetry (10 Hz)
#define SENSOR_PUBLISH_RATE     20       // Counter threshold (200Hz / 20 = 10Hz)

#endif // ROBOT_CONFIG_H
