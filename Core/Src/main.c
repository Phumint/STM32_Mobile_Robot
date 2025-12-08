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
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
//#include "motor_control.h"
//#include "servo_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_CENTER 90
#define SERVO_LEFT_MAX 50    // Maximum left turn
#define SERVO_RIGHT_MAX 130  // Maximum right turn

#define MOTOR_MAX_SPEED 100
#define MOTOR_MIN_SPEED -100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Encoder variables
volatile int32_t encoder_count = 0;
volatile uint16_t encoder_raw = 0, encoder_raw_prev = 0;

// MPU6050 instance
MPU6050_t mpu6050;
uint8_t mpu_initialized = 0;

// Control commands from ROS2
volatile int8_t motor_speed_cmd = 0;      // -100 to +100
volatile uint8_t servo_angle_cmd = 90;    // 50 to 130 degrees

// UART receive buffer for commands
#define RX_BUFFER_SIZE 64
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
uint8_t rx_byte = 0;   // remove volatile to match HAL_UART_Receive_IT prototype

// Helper to get TIM3 PWM top (use actual autoreload)
static inline uint32_t MOTOR_PWM_MAX(void) {
    return (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim3);
}


// Sensor publishing rate control
uint16_t sensor_publish_counter = 0;
#define SENSOR_PUBLISH_RATE 20  // Publish every 20 * 5ms = 100ms (10Hz)

void updateEncoderCount(void)
{
    encoder_raw = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t delta = (int16_t)(encoder_raw - encoder_raw_prev);

    // Handle timer overflow/underflow
    if(delta > 30000) encoder_count -= 65536;
    else if(delta < -30000) encoder_count += 65536;

    encoder_count += delta;
    encoder_raw_prev = encoder_raw;
}

void readIMU(void)
{
    if (mpu_initialized)
    {
        MPU6050_Read_All(&hi2c1, &mpu6050);
        MPU6050_Calculate_Angles(&mpu6050);
    }
}

void Servo_SetAngle(uint8_t angle)
{
    // Constrain to safe limits
    if(angle < SERVO_LEFT_MAX) angle = SERVO_LEFT_MAX;
    if(angle > SERVO_RIGHT_MAX) angle = SERVO_RIGHT_MAX;

    // Convert angle to microsecond pulse between 1000..2000 (tunable)
    // map angle range [SERVO_LEFT_MAX..SERVO_RIGHT_MAX] -> [1000..2000] us
    uint32_t left = 1000;
    uint32_t right = 2000;
    uint32_t pulse_us = left + ( ( (uint32_t)(angle - SERVO_LEFT_MAX) * (right - left) )
                                 / (uint32_t)(SERVO_RIGHT_MAX - SERVO_LEFT_MAX) );

    // TIM4 is set to 1us tick (Prescaler=71, Period=19999)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)pulse_us);
}


void Motor_SetSpeed(int8_t speed)
{
    // Constrain speed
    if(speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if(speed < MOTOR_MIN_SPEED) speed = MOTOR_MIN_SPEED;

    // compute absolute value safely
    uint8_t abs_speed = (speed >= 0) ? (uint8_t)speed : (uint8_t)(-speed);

    // scale 0-100 to timer compare range
    uint32_t pwm_max = MOTOR_PWM_MAX(); // 0..ARR
    uint32_t pwm = (abs_speed * pwm_max) / 100U;

    if(speed > 0) {
        // Forward (CH1)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    else if(speed < 0) {
        // Reverse (CH2)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
    }
    else {
        // Stop
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}


void publishSensorData(void)
{
    char buffer[200];

    // Format: $ENC,count;IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw*
    // Using $ as start, * as end for easy parsing
    // Values scaled as integers for efficient transmission

    int16_t ax_int = (int16_t)(mpu6050.accel_x * 1000);
    int16_t ay_int = (int16_t)(mpu6050.accel_y * 1000);
    int16_t az_int = (int16_t)(mpu6050.accel_z * 1000);
    int16_t gx_int = (int16_t)(mpu6050.gyro_x * 100);
    int16_t gy_int = (int16_t)(mpu6050.gyro_y * 100);
    int16_t gz_int = (int16_t)(mpu6050.gyro_z * 100);
    int16_t roll_int = (int16_t)(mpu6050.roll * 100);
    int16_t pitch_int = (int16_t)(mpu6050.pitch * 100);

    // Note: MPU6050 doesn't have magnetometer, so no true yaw
    // We'll integrate gyro_z for relative yaw

    sprintf(buffer, "$ENC,%ld;IMU,%d,%d,%d,%d,%d,%d,%d,%d*\r\n",
            encoder_count,
            ax_int, ay_int, az_int,
            gx_int, gy_int, gz_int,
            roll_int, pitch_int);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}

void parseCommand(uint8_t *buffer, uint16_t length)
{
    // Basic sanity checks
    if(length < 4) return;  // Too short to be valid
    if(buffer[0] != '#') return;  // Wrong start byte
    if(buffer[length-1] != '*') return;  // Wrong end byte

    // Null-terminate (overwrite '*' with '\0') so sscanf is safe
    if(length >= 1 && length <= RX_BUFFER_SIZE) {
        buffer[length-1] = '\0';
    } else return;

    // Now parse
    if(strncmp((char*)&buffer[1], "CMD,", 4) == 0)
    {
        int speed = 0;
        int angle = 90;

        // Parse: #CMD,speed,angle*
        // Start parsing after "#CMD,"
        sscanf((char*)&buffer[5], "%d,%d", &speed, &angle);

        // Clamp parsed values
        if(speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
        if(speed < MOTOR_MIN_SPEED) speed = MOTOR_MIN_SPEED;
        if(angle < SERVO_LEFT_MAX) angle = SERVO_LEFT_MAX;
        if(angle > SERVO_RIGHT_MAX) angle = SERVO_RIGHT_MAX;

        // Update commands
        motor_speed_cmd = (int8_t)speed;
        servo_angle_cmd = (uint8_t)angle;

        // Apply immediately
        Motor_SetSpeed(motor_speed_cmd);
        Servo_SetAngle(servo_angle_cmd);
    }
    else if(strncmp((char*)&buffer[1], "STOP", 4) == 0)
    {
        // Emergency stop
        motor_speed_cmd = 0;
        servo_angle_cmd = SERVO_CENTER;
        Motor_SetSpeed(0);
        Servo_SetAngle(SERVO_CENTER);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        // 5ms control loop (200Hz)
        updateEncoderCount();
        readIMU();

        // Publish sensor data at 10Hz
        sensor_publish_counter++;
        if(sensor_publish_counter >= SENSOR_PUBLISH_RATE)
        {
            sensor_publish_counter = 0;
            publishSensorData();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // store byte if space, otherwise reset buffer to avoid overflow
        if(rx_index < RX_BUFFER_SIZE) {
            rx_buffer[rx_index++] = rx_byte;
        } else {
            // overflow — reset
            rx_index = 0;
        }

        // Check for end of command
        if(rx_byte == '*' || rx_index >= RX_BUFFER_SIZE)
        {
            parseCommand(rx_buffer, rx_index);
            rx_index = 0;  // Reset buffer
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
        }

        // Continue receiving next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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

  // Start control loop timer (200Hz)
  HAL_TIM_Base_Start_IT(&htim1);

  // Start encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Start motor PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Start servo PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  Servo_SetAngle(SERVO_CENTER);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

  // Initialize MPU6050
  HAL_Delay(100);
  if (MPU6050_Init(&hi2c1) == 0)
  {
      mpu_initialized = 1;
  }

  // Start UART reception
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  // Send ready signal
  HAL_Delay(100);
  HAL_UART_Transmit(&huart1, (uint8_t*)"$READY*\r\n", 9, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // All processing in interrupts
      // Main loop can be used for error checking or low-priority tasks

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
