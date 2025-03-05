/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"           // Include main header with project definitions and prototypes
#include "usb_device.h"     // Include USB device header for USB functionalities for color sensor calibration
#include "usbd_cdc_if.h"    // Include USB CDC interface header for USB communication for color sensor calibration

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>          // Standard I/O library for sprintf() and related functions
#include <string.h>         // String manipulation functions (e.g., strlen, strcat)
#include <stdbool.h>        // Boolean type and values

/* Private typedef -----------------------------------------------------------*/
/* zoz: Color sensor pin definitions */
// Define pins and corresponding GPIO ports for the TCS3200 color sensor
#define S0_Pin         GPIO_PIN_4   // S0 control pin on PA4
#define S0_GPIO_Port   GPIOA          // GPIO Port A for S0
#define S1_Pin         GPIO_PIN_5   // S1 control pin on PA5
#define S1_GPIO_Port   GPIOA          // GPIO Port A for S1
#define S2_Pin         GPIO_PIN_6   // S2 control pin on PA6
#define S2_GPIO_Port   GPIOA          // GPIO Port A for S2
#define S3_Pin         GPIO_PIN_7   // S3 control pin on PA7
#define S3_GPIO_Port   GPIOA          // GPIO Port A for S3
#define OUT_Pin        GPIO_PIN_0   // Output pin from color sensor on PB0
#define OUT_GPIO_Port  GPIOB          // GPIO Port B for output pin

/* Private define ------------------------------------------------------------*/
/* zoz: Color sensor variables */
// Declare global variables to store pulse durations for Red, Blue, and Green colors
int Red = 0, Blue = 0, Green = 0;  // Pulse durations in microseconds for respective colors

/* Private macro -------------------------------------------------------------*/
/* zoz: Color sensor function prototypes */
// Function prototypes for helper functions related to timing and sensor reading
uint32_t micros(void);     // Returns current time in microseconds using DWT
uint32_t pulseIn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout);  // Measures pulse duration on a GPIO pin
void GetColors(void);      // Reads color values from the TCS3200 sensor

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;   // Handle for Timer4, used for PWM generation

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // Configures the system clock
static void MX_GPIO_Init(void);   // Initializes GPIO pins
static void MX_TIM4_Init(void);   // Initializes Timer4 for PWM

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Define motor control pins and IR sensor pins with their respective channels and GPIO pins

// Left motors (using Timer channels for PWM control)
#define motor_A1 TIM_CHANNEL_1  // Left motor control pin 1 (mapped to PB6)
#define motor_A2 TIM_CHANNEL_2 // Left motor control pin 2 (mapped to PB7)
#define motor_B1 TIM_CHANNEL_3 // Right motor control pin 1 (mapped to PB8)
#define motor_B2 TIM_CHANNEL_4  // Right motor control pin 2 (mapped to PB9)

// IR Sensor Array Pins definitions
// The IR sensor pins below should be updated to the actual GPIO pins used on your board.
// These comments indicate example mappings.
#define ir_1 GPIO_PIN_0 // IR sensor 1 (mapped to PA0)
#define ir_2 GPIO_PIN_1 // IR sensor 2 (mapped to PA1)
#define ir_3 GPIO_PIN_2 // IR sensor 3 (mapped to PA2)
#define ir_4 GPIO_PIN_3 // IR sensor 4 (mapped to PA3)


// Constants for motor speeds for different maneuvers
int const moving_speed = 160;   // Speed value for moving straight
int const rotate_speed = 150;   // Speed value for rotating or turning

/* zoz: State variables for new features */
// Variables to maintain robot state and sensor event counters
static bool isMoving = false;      // Tracks if the robot is in a moving state
static uint32_t redStopTime = 0;   // Timestamp for when red stop was initiated
static bool redDetected = false;   // Flag to indicate if red has been consistently detected
static bool lastWasWhite = false;  // Flag to track if the last detected color was white (for lap logic)
static int redCounter = 0;         // Counter for consecutive red detections
# define RED_COUNTER_THRESHOLD 3    // Threshold count of red detections to trigger a stop
static bool postRedActive = false; // Flag indicating if post-red state (waiting period) is active
static uint32_t postRedStartTime = 0; // Timestamp for the start of the post-red state

/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
// Function to send a message via USB CDC (virtual COM port)
void sendMessage(char *msg)
{
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));  // Transmit the string message over USB
}

// Function to simulate analogWrite using TIM4 PWM channels
void analogWrite(uint32_t Channel, uint16_t value)
{
    if (value > 255) value = 255; // Ensure PWM value is within the 0-255 range
    __HAL_TIM_SET_COMPARE(&htim4, Channel, value); // Set the PWM duty cycle for the specified channel
}

// Motor control function to set speeds for left and right motors
void move(int speedA, int speedB) {
  // Determine direction for left motor based on sign of speedA
  if (speedA > 0) {
    analogWrite(motor_A1, speedA); // Forward drive for left motor
    analogWrite(motor_A2, 0);        // No reverse drive
  } else {
    analogWrite(motor_A1, 0);        // No forward drive
    analogWrite(motor_A2, -speedA);  // Reverse drive for left motor
  }

  // Determine direction for right motor based on sign of speedB
  if (speedB > 0) {
    analogWrite(motor_B1, speedB); // Forward drive for right motor
    analogWrite(motor_B2, 0);        // No reverse drive
  } else {
    analogWrite(motor_B1, 0);        // No forward drive
    analogWrite(motor_B2, -speedB);  // Reverse drive for right motor
  }
}

// Function for line tracking using IR sensor array readings
void line_tracking(void) {
  // Read sensor values using HAL_GPIO_ReadPin from GPIOA for each IR sensor pin
  int s1 = (HAL_GPIO_ReadPin(GPIOA, ir_1) == GPIO_PIN_SET) ? 1 : 0; // Left Most Sensor reading
  int s2 = (HAL_GPIO_ReadPin(GPIOA, ir_2) == GPIO_PIN_SET) ? 1 : 0; // Left Sensor reading
  int s3 = (HAL_GPIO_ReadPin(GPIOA, ir_3) == GPIO_PIN_SET) ? 1 : 0; // Middle Sensor reading
  int s4 = (HAL_GPIO_ReadPin(GPIOA, ir_4) == GPIO_PIN_SET) ? 1 : 0; // Right Sensor reading

  // Decision making based on sensor states for turning maneuvers
  if (s3 == 1 && s4 == 1){
     move(-rotate_speed, rotate_speed); // If middle and right sensors are active, initiate a left turn
  }
  else if (s2 == 1 && s1 == 1) {
     move(rotate_speed, -rotate_speed); // If left sensors are active, initiate a right turn
  }
  else if (s1 == 1) {
     move(rotate_speed, -rotate_speed); // If only left-most sensor is active, adjust to the left by turning right
  }
  else if (s2 == 1 ){
    move(rotate_speed, -rotate_speed); // If left sensor (s2) is active, continue turning right
  }
  else if (s3 == 1) {
    move(-rotate_speed, rotate_speed); // If middle sensor (s3) is active, adjust by turning left
  }
  else  {
    move(moving_speed, moving_speed - 40); // Otherwise, move straight with a slight speed adjustment to the right
  }
}

/*  Color sensor helper functions */
// Function to get the current microsecond count using DWT
uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);  // Convert cycle count to microseconds
}

// Function to measure the duration of a pulse on a specified GPIO pin
uint32_t pulseIn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout)
{
    uint32_t startTick = HAL_GetTick(); // Get the current tick count in milliseconds
    // Wait while the pin is in the specified state
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == state)
    {
        if ((HAL_GetTick() - startTick) > timeout)
            return 0; // Return 0 if timeout is exceeded
    }
    // Wait for the pin to switch to the specified state
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != state)
    {
        if ((HAL_GetTick() - startTick) > timeout)
            return 0; // Return 0 if timeout is exceeded
    }
    uint32_t startTime = micros(); // Capture the start time in microseconds
    // Measure the duration while the pin stays in the specified state
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == state)
    {
        if ((HAL_GetTick() - startTick) > timeout)
            break; // Break if timeout occurs
    }
    uint32_t endTime = micros(); // Capture the end time in microseconds
    return (endTime - startTime); // Return the pulse duration in microseconds
}

// Function to read color values from the TCS3200 sensor 
void GetColors(void)
{
    // Set sensor filters for red: S2=LOW, S3=LOW
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET);
    // Measure the pulse width for red; determine the edge based on current state
    Red = pulseIn(OUT_GPIO_Port, OUT_Pin,
                  HAL_GPIO_ReadPin(OUT_GPIO_Port, OUT_Pin) == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET, 1000);
    HAL_Delay(5);  // Delay for sensor stabilization (5 ms)

    // Set sensor filters for blue: S2=LOW, S3=HIGH
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET);
    // Measure the pulse width for blue color
    Blue = pulseIn(OUT_GPIO_Port, OUT_Pin,
                   HAL_GPIO_ReadPin(OUT_GPIO_Port, OUT_Pin) == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET, 1000);
    HAL_Delay(5);  // Delay for sensor stabilization (5 ms)

    // Set sensor filters for green: S2=HIGH, S3=HIGH
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET);
    // Measure the pulse width for green color
    Green = pulseIn(OUT_GPIO_Port, OUT_Pin,
                    HAL_GPIO_ReadPin(OUT_GPIO_Port, OUT_Pin) == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET, 1000);
    HAL_Delay(5);  // Delay for sensor stabilization (5 ms)
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
  HAL_Init();  // Reset all peripherals and initialize the HAL library

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // Configure system clock parameters

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();   // Initialize all GPIO pins
  MX_TIM4_Init();   // Initialize Timer4 for PWM generation
  MX_USB_DEVICE_Init();  // Initialize the USB device for communication for color sensor calibration

  /* USER CODE BEGIN 2 */
  // Start PWM on Timer4 channels for motor control
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // Start PWM for Left motor control pin 1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // Start PWM for Left motor control pin 2
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  // Start PWM for Right motor control pin 1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  // Start PWM for Right motor control pin 2

  // Initialize DWT for high-resolution timing used by the TCS3200 color sensor
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable tracing
  DWT->CYCCNT = 0;                                // Reset cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter

  // Set the TCS3200 sensor frequency scaling to 100% (S0=HIGH, S1=HIGH)
  HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, GPIO_PIN_SET);  // Set S0 to HIGH for 100% scaling
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);  // Set S1 to HIGH for 100% scaling
  HAL_Delay(1000);  // Allow sensor to stabilize for 1 second
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Obtain the color sensor readings and store them in global variables
    GetColors();
    char buffer[100];  // Create a buffer for the message
    // Format the sensor values into the buffer string
    sprintf(buffer, "Red = %d, Blue = %d, Green = %d ", Red, Blue, Green);

    // Check if the system is in a red-stop state
    if (redDetected) {
        // If within 5 seconds of red detection, keep the robot stopped
        if ((HAL_GetTick() - redStopTime) < 5000) {
            move(0, 0);  // Stop both motors
            strcat(buffer, "- Color: Red (Stopped)\r\n"); // Append red stop message to buffer
            sendMessage(buffer);  // Send the message over USB
            continue;  // Skip further processing in this loop iteration
        } else {
            // If 5 seconds have passed, clear red-stop state and reset red counter
            redDetected = false;
            redCounter = 0;
            // Initiate a post-red phase where the robot resumes motion for 3 seconds
            postRedActive = true;
            postRedStartTime = HAL_GetTick();  // Record the start time of the post-red phase
        }
    }

    // Handle the post-red phase: resume line tracking for 3 seconds after a red stop
    if (postRedActive) {
        if ((HAL_GetTick() - postRedStartTime) < 3000) {
            line_tracking();  // Execute line tracking to continue following the path
            strcat(buffer, "- Post Red: Resuming Line Tracking\r\n"); // Append post-red message to buffer
            sendMessage(buffer);  // Send the message over USB
            continue;  // Wait until the 3-second period completes before processing further
        } else {
            postRedActive = false; // End the post-red phase after 3 seconds
        }
    }

    // Evaluate color sensor readings for green detection (interpreted as white or moving state)
    if (Red <= 25 && Green <= 25 && Blue <= 25) {
        strcat(buffer, "- Color: White\r\n");  // Append message indicating white color detected
        if (isMoving) {
            line_tracking();  // Continue line tracking if already in motion
        }
    }
    // Evaluate conditions for green color detection to continue moving
    else if (Green < Red && (Blue - Green) <= 35) {
        isMoving = true;  // Set the moving flag to true
        redCounter = 0;   // Reset the red detection counter
        strcat(buffer, "- Color: Green (Moving)\r\n");  // Append green moving message to buffer
        line_tracking();  // Continue line tracking
    }
    // Evaluate conditions for red color detection (using defined thresholds)
    else if (Red < Blue && Red <= Green && Red < 35) {
        redCounter++;  // Increment the red counter for consecutive detections
        // If the red counter exceeds the threshold, initiate a stop
        if (redCounter >= RED_COUNTER_THRESHOLD) {
            move(0, 0);  // Stop the robot by halting motor movement
            redDetected = true;  // Set the red-detected flag
            redStopTime = HAL_GetTick();  // Record the time at which red was detected
            isMoving = false;  // Set moving state to false
            strcat(buffer, "- Color: Red (Stop Initiated)\r\n");  // Append red stop message
            redCounter = 0;  // Reset the red counter after stopping
        } else {
            // If not enough red detections, maintain current motion if moving
            if (isMoving) {
                line_tracking();  // Continue with line tracking
            }
            strcat(buffer, "- Color: Possibly Red (Transient)\r\n");  // Append transient red detection message
        }
    }
    // For any other color readings, assume an unknown color and continue motion if already moving
    else {
        if (isMoving) {
            line_tracking();  // Continue line tracking
        }
        redCounter = 0;  // Reset the red detection counter since red is not present
        strcat(buffer, "- Color: Unknown\r\n");  // Append unknown color message to buffer
    }

    // Send the constructed message buffer over USB to monitor sensor and state information
    sendMessage(buffer);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Additional user code can be placed here if necessary.
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // Structures to configure oscillators and clocks
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters in RCC_OscInitTypeDef structure.
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // Use High-Speed External oscillator
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                   // Turn on the HSE
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;      // No division on HSE
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Turn on High-Speed Internal oscillator
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // Use HSE as PLL source
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;               // Multiply PLL clock by 9
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();  // Call error handler if oscillator configuration fails
  }

  /** Initializes the CPU, AHB and APB buses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; // Configure various clock types
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Use PLL as system clock source
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // No division on AHB clock
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         // Divide APB1 clock by 2
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // No division on APB2 clock

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();  // Call error handler if clock configuration fails
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;   // Select USB clock configuration
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5; // Set USB clock source from PLL divided by 1.5
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();  // Call error handler if peripheral clock configuration fails
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  // Local configuration structures for Timer4 initialization
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  // Place for pre-initialization code for Timer4 if needed.
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;                      // Use Timer4 peripheral
  htim4.Init.Prescaler = 0;                   // No prescaling; timer clock runs at system clock speed
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP; // Count up mode
  htim4.Init.Period = 255;                    // Set period for 8-bit resolution (0-255)
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // No clock division
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Disable auto-reload preload
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();  // Call error handler if PWM initialization fails
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // Set master output trigger to reset
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // Disable master/slave mode
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();  // Call error handler if master configuration fails
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;          // Set output compare mode to PWM1
  sConfigOC.Pulse = 0;                         // Initialize pulse width to 0 (motor off)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;    // Set output polarity to high
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;     // Disable fast mode for output compare
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();  // Call error handler if PWM channel 1 configuration fails
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();  // Call error handler if PWM channel 2 configuration fails
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();  // Call error handler if PWM channel 3 configuration fails
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();  // Call error handler if PWM channel 4 configuration fails
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  // Place for post-initialization code for Timer4 if needed.
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);  // Call the post initialization function for Timer4
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // Declare structure for GPIO configuration

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  // Place for any early GPIO initialization if needed.
  /* USER CODE END MX_GPIO_Init_1 */

  // Enable clocks for GPIO ports D, A, and B
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure initial output level for TCS3200 sensor control pins on GPIOA (S0, S1, S2, S3)
  HAL_GPIO_WritePin(GPIOA, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  // Configure IR sensor pins as inputs on GPIOA
  GPIO_InitStruct.Pin = outer_left_ir_Pin|inner_left_ir_Pin|inner_right_ir_Pin|outer_right_ir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;   // Set as input mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down resistors
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);      // Initialize IR sensor pins

  // Configure TCS3200 sensor control pins as output push-pull on GPIOA
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Set as output push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down resistors
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set speed to low frequency
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);      // Initialize sensor control pins

  // Configure the sensor output pin (color output) as input on its respective port
  GPIO_InitStruct.Pin = out_color_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // Set as input mode for reading sensor output
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pull-up or pull-down resistors
  HAL_GPIO_Init(out_color_GPIO_Port, &GPIO_InitStruct); // Initialize sensor output pin

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Place for additional GPIO configurations if needed.
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Additional user functions or interrupt handlers can be added here
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // Disable interrupts and enter an infinite loop in case of error
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
  // User can implement reporting of the file name and line number where the assert error occurred
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
