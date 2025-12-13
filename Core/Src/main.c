/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 for ROS2 Ackermann RC Car
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_config.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "encoder_driver.h"
#include "ros_comms.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t mpu6050;

// MARK AS VOLATILE: Shared between Main Loop (Writer) and Interrupt (Reader)
volatile int8_t current_speed_cmd = 0;
uint8_t current_angle_cmd = SERVO_CENTER_ANGLE;
uint32_t last_pub_time = 0;

// --- Encoder & Velocity Globals ---
volatile int32_t encoder_curr = 0;
volatile int32_t encoder_prev = 0;
float v_actual = 0.0f;

// --- PID Controller State ---
typedef struct {
    float error_integral;
    float prev_error;
} PID_State_t;

PID_State_t motor_pid = {0.0f, 0.0f};

// DELETE THIS: extern int8_t motor_speed_cmd; <--- Caused the variable mismatch
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Converts raw encoder ticks to linear velocity (m/s)
 * Handles 16-bit timer wrapping automatically via casting.
 */
float Calculate_Velocity(void) {
    // 1. Read current counter from TIM2
    encoder_curr = __HAL_TIM_GET_COUNTER(&htim2);

    // 2. Calculate delta (cast to int16_t handles 0 -> 65535 wrap-around)
    int16_t delta_ticks = (int16_t)(encoder_curr - encoder_prev);

    // 3. Update previous count
    encoder_prev = encoder_curr;

    // 4. Convert to m/s
    // v = (delta / dt) * (circumference / counts_per_rev)
    float raw_vel = (delta_ticks / DT) * ((3.14159f * WHEEL_DIAMETER) / COUNTS_PER_REV);

    // 5. Low-Pass Filter to remove quantization noise
    static float v_smooth = 0.0f;
    v_smooth = (VEL_FILTER_ALPHA * raw_vel) + ((1.0f - VEL_FILTER_ALPHA) * v_smooth);

    return v_smooth;
}

/**
 * @brief Computes PID output
 * @param target_vel Target velocity in m/s
 * @param current_vel Actual velocity in m/s
 * @return Motor PWM command (-100 to 100)
 */
int8_t PID_Compute(float target_vel, float current_vel) {
    float error = target_vel - current_vel;

    // Proportional Term
    float P = PID_KP * error;

    // Integral Term (with Anti-Windup)
    motor_pid.error_integral += error * DT;

    // Clamp Integral to prevent runaway
    // (Simple clamping; can be improved based on output saturation)
    float I = PID_KI * motor_pid.error_integral;
    if (I > MAX_PWM_OUTPUT) motor_pid.error_integral = MAX_PWM_OUTPUT / PID_KI;
    if (I < MIN_PWM_OUTPUT) motor_pid.error_integral = MIN_PWM_OUTPUT / PID_KI;
    I = PID_KI * motor_pid.error_integral;

    // Derivative Term
    float D = PID_KD * (error - motor_pid.prev_error) / DT;
    motor_pid.prev_error = error;

    // Sum
    float output = P + I + D;

    // Final Clamping to Motor limits
    if (output > MAX_PWM_OUTPUT) output = MAX_PWM_OUTPUT;
    if (output < MIN_PWM_OUTPUT) output = MIN_PWM_OUTPUT;

    return (int8_t)output;
}

void Robot_Update_Loop(void) {
    // 1. Check for incoming ROS2 commands (Non-blocking)
    // This updates the global 'current_speed_cmd'
	ROS_CheckForCommand((int8_t*)&current_speed_cmd, &current_angle_cmd);

    // 2. Apply Actuation
    // REMOVED: Motor_SetSpeed(current_speed_cmd); <--- This was causing the fight!

    // Servo is still open-loop, so this stays here:
    Servo_SetAngle(current_angle_cmd);

    // 3. Telemetry Publishing (10Hz)
    // Keep sensor reading HERE, not in the Interrupt.
    if (HAL_GetTick() - last_pub_time >= (1000 / SENSOR_PUB_RATE_HZ)) {
        MPU6050_Read_All(&hi2c1, &mpu6050);
        MPU6050_Calculate_Angles(&mpu6050);
        ROS_SendTelemetry(Encoder_GetCount(), &mpu6050);
        last_pub_time = HAL_GetTick();
    }
}

/* Callbacks */
// Redirect Interrupts to specific modules
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) { // 200Hz Control Loop

    	Encoder_Update();

        // 1. Calculate Actual Velocity
        v_actual = Calculate_Velocity();

        // 2. Determine Setpoint
        // USE 'current_speed_cmd' HERE
        float v_target = ((float)current_speed_cmd / 100.0f) * MAX_LINEAR_VELOCITY;

        // 3. Compute PID Output
        int8_t pid_output = PID_Compute(v_target, v_actual);

        // 4. Handle Stop Condition (Reset PID when stopped to prevent windup)
        if (current_speed_cmd == 0) {
            pid_output = 0;
            motor_pid.error_integral = 0;
            motor_pid.prev_error = 0;
        }

        // 5. Apply to Motor (The Interrupt now OWNS the motor)
        Motor_SetSpeed(pid_output);

        // REMOVED: readIMU() and publishSensorData()
        // We do this in the main loop now to avoid blocking the interrupt.
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        ROS_UART_ISR_Handler();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // --- Module Initialization ---
    Motor_Init();
    Servo_Init();
    Encoder_Init();

    MPU6050_Init(&hi2c1);
    HAL_Delay(500); // Wait for sensor to settle

    ROS_Comms_Init();

    HAL_TIM_Base_Start_IT(&htim1); // Start Control Loop Timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // The Interrupts handle raw data (RX buffer, Encoder ticks).
	  // The Main Loop handles Logic (Parsing, PID, I2C Reads). tasks
	  Robot_Update_Loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
