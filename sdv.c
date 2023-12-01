/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "bno055_stm32.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct pid_sytem
{
    float kp;
    float ki;
    float kd;
    float integral;
    float max_integral;
    int max_value;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int position = 0;
long ticks = 0;
long last_ticks = 0;
float angle = 0.0;
float rpm = 0.0;
float rev = 0.0;
float current_distance = 0.0;
float x_pos = 0.0;
float y_pos = 0.0;

float target_x = 0.0;
float target_y = 0.0;
float target_angle = 0.0;
float target_distance = 0.0;

float comp_angle = 0.0;

uint8_t ult_count = 0;

uint8_t btBuffer[12];
uint8_t txBuffer[10];
bool toggle_interrupt;

uint8_t iterations = 0;
bool obstacle = false;

int state = -3;

int calculate_pid(struct pid_sytem *system, float setpoint, float current){
    float error = setpoint - current;
    system->integral += error;
    // Saturation for integral values
    system->integral = system->integral > system->max_integral ? system->max_integral : system->integral;
    system->integral = system->integral < -system->max_integral ? -system->max_integral : system->integral;

    float output = system->kp * error + system->ki * system->integral;
    // Saturation for output values
    output = output > system->max_value ? system->max_value : output;
    output = output < -system->max_value ? -system->max_value : output;

    return (int)output;
}
void obstacle_handler(){
    ult_count = 0;
    HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET); // pull the TRIG pin low
    HAL_Delay(0.01);
    HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET); // pull the TRIG pin low

    bool ult_pulse = HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin);
    for (int i = 0; i < 50; i++){
        if (HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) != ult_pulse)
            ult_pulse = !ult_pulse;
        else
            ult_count++;
        HAL_Delay(0.01);
    }
    if (ult_count == 50){
        obstacle = true;
        htim4.Instance->CCR3 = 0;
        htim1.Instance->CCR1 = 1000;
    }
    else
        obstacle = false;

    iterations = 0;
}

void send_odometry(float ang){
    txBuffer[0] = current_distance > 0 ? '+' : '-';
    uint8_t tmp = (uint8_t)( current_distance > 0 ? current_distance*100 : -current_distance*100 );
    txBuffer[1] = tmp / 1000;
    tmp -= txBuffer[1] * 1000;
    txBuffer[2] = tmp / 100;
    tmp -= txBuffer[2] * 100;
    txBuffer[3] = tmp / 10;
    tmp -= txBuffer[3] * 10;
    txBuffer[4] = tmp;
    tmp = (uint8_t)( ang*100 );
    txBuffer[5] = tmp / 10000;
    tmp -= txBuffer[5] * 10000;
    txBuffer[6] = tmp / 1000;
    tmp -= txBuffer[6] * 1000;
    txBuffer[7] = tmp / 100;
    tmp -= txBuffer[7] * 100;
    txBuffer[8] = tmp / 10;
    tmp -= txBuffer[8] * 10;
    txBuffer[9] = tmp;

    HAL_UART_Transmit(&huart1, txBuffer, 10, 100);
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
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_I2C1_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, btBuffer, 12); // Enabling interrupt receive
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    bno055_assignI2C(&hi2c1);
    bno055_setup();
    bno055_setOperationModeNDOF();
    struct pid_sytem position;
    position.kp = 120000;
    position.ki = 1000;
    position.max_integral = 1000;
    position.max_value = 60535;

    struct pid_sytem direction;
    direction.kp = 14;
    direction.ki = 0.0;
    direction.max_integral = 50;
    direction.max_value = 400;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        rev = (float)ticks / ppr;
        rpm = (((float)(ticks - last_ticks) / ppr) / (float)HAL_GetTick()) * 600000.0; // Revolutions per minute
        last_ticks = ticks;
        current_distance = rev * pi * wheel_diameter * distance_offset;

        bno055_vector_t v = bno055_getVectorEuler();
        int motor_pwm = 0, direction_pwm = 0;

        switch (state){
        case 0: // Move forward target_distance meters
            angle = v.x > 180 ? v.x - 360 : v.x;
            motor_pwm = calculate_pid(&position, target_distance, current_distance);
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (10.00 - current_distance < target_tolerance){
                state = -3;
                htim4.Instance->CCR3 = motor_pwm = 0;
                HAL_Delay(4000);
            }
            break;
        case 1: // Turn 90 degrees
            angle = v.x > 270 ? v.x - 450 : v.x - 90;
            motor_pwm = min_pwm;
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (angle > -angle_tolerance){
                state = 2;
                ticks = 0;
                htim4.Instance->CCR3 = motor_pwm = 0;
                htim1.Instance->CCR1 = 1000;
                HAL_Delay(3000);
            }
            break;

        case 2: // Move forward target_distance meters
            angle = v.x > 270 ? v.x - 450 : v.x - 90;
            motor_pwm = calculate_pid(&position, target_distance, current_distance);
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (10.00 - current_distance < target_tolerance){
                state = 3;
                htim4.Instance->CCR3 = motor_pwm = 0;
                HAL_Delay(4000);
            }
            break;
        case 3: // Turn 90 degrees
            angle = v.x - 180;
            motor_pwm = min_pwm;
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (angle > -angle_tolerance){
                state = 4;
                ticks = 0;
                htim4.Instance->CCR3 = motor_pwm = 0;
                htim1.Instance->CCR1 = 1000;
                HAL_Delay(3000);
            }
            break;
        case 4: // Move forward target_distance meters
            angle = v.x - 180;
            motor_pwm = calculate_pid(&position, target_distance, current_distance);
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (10.00 - current_distance < target_tolerance){
                state = 5;
                htim4.Instance->CCR3 = motor_pwm = 0;
                HAL_Delay(4000);
            }
            break;
        case 5: // Turn 90 degrees
            angle = v.x < 90 ? 90 + v.x : v.x - 270;
            motor_pwm = min_pwm;
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (angle > -angle_tolerance){
                state = 6;
                ticks = 0.0;
                htim4.Instance->CCR3 = motor_pwm = 0;
                htim1.Instance->CCR1 = 1000;
                HAL_Delay(3000);
            }
            break;
        case 6: // Move forward target_distance meters
            angle = v.x < 90 ? -(90 + v.x) : v.x - 270;
            motor_pwm = calculate_pid(&position, target_distance, current_distance);
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (target_distance - current_distance < target_tolerance){
                state = -3;
                htim4.Instance->CCR3 = motor_pwm = 0;
                htim1.Instance->CCR1 = 1000;
                HAL_Delay(4000);
            }
            break;

        case -1: // Test speed
            break;
        case -2: // Test angle
            angle = v.x > 180 ? v.x - 360 : v.x;
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            break;
        case -3: // Stop
            htim4.Instance->CCR3 = motor_pwm = 0;
            htim1.Instance->CCR1 = 1000;
            break;
        case -4: // Turn to target_angle
        	comp_angle = target_angle + 180;
        	 if (comp_angle > 360){
        		 comp_angle -= 360;
        	     angle = v.x < comp_angle ? (360-target_angle) + v.x : v.x - target_angle;
			} else
				angle = v.x > comp_angle ? v.x - target_angle - 360 : v.x - target_angle;

            motor_pwm = 15000 * (angle > 0? angle : -angle);
            if(motor_pwm > 65000)
            		motor_pwm = 65000;
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (angle > -angle_tolerance && angle < angle_tolerance){
                state = -5;
                htim4.Instance->CCR3 = motor_pwm = 0;
                htim1.Instance->CCR1 = 1000;
                HAL_Delay(3000);
                ticks = 0;
            }
            break;
        case -5:
            comp_angle = target_angle + 180;
            if (comp_angle > 360){
                comp_angle -= 360;
                angle = v.x < comp_angle ? (360-target_angle) + v.x : v.x - target_angle;
            } else
                angle = v.x > comp_angle ? v.x - target_angle - 360 : v.x - target_angle;

            motor_pwm = calculate_pid(&position, target_distance, current_distance);
            direction_pwm = calculate_pid(&direction, 0.0, angle);
            if (target_distance - current_distance < target_tolerance && target_distance - current_distance > -target_tolerance){
                state = -3;
                htim4.Instance->CCR3 = motor_pwm = 0;
                HAL_Delay(4000);
            }
            break;
        default:
            break;
        }

        if (obstacle)
            motor_pwm = direction_pwm = 0;

        HAL_GPIO_WritePin(motor_a_GPIO_Port, motor_a_Pin, motor_pwm < 0 ? 1 : 0);
        HAL_GPIO_WritePin(motor_b_GPIO_Port, motor_b_Pin, motor_pwm > 0 ? 1 : 0);
        motor_pwm = motor_pwm > 0 ? motor_pwm : -motor_pwm;
        htim4.Instance->CCR3 = (motor_pwm > 5000 && motor_pwm < 25000)? min_pwm : motor_pwm;
        htim1.Instance->CCR1 = 1000 + direction_pwm;

        // print ticks
        HAL_Delay(10);
        iterations++;

        // Read ultrasonic
        if (iterations == 10){
            //obstacle_handler();
            //send_odometry(v.x);
        }
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void){
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 20000;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, trig_Pin | motor_a_Pin | motor_b_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : echo_Pin encoder_b_Pin */
    GPIO_InitStruct.Pin = echo_Pin | encoder_b_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : trig_Pin motor_a_Pin motor_b_Pin */
    GPIO_InitStruct.Pin = trig_Pin | motor_a_Pin | motor_b_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : encoder_a_Pin */
    GPIO_InitStruct.Pin = encoder_a_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(encoder_a_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if (HAL_GPIO_ReadPin(encoder_a_GPIO_Port, encoder_a_Pin)){
        if (!HAL_GPIO_ReadPin(encoder_b_GPIO_Port, encoder_b_Pin))
            ticks++;
        else
            ticks--;
    }
    else{
        if (!HAL_GPIO_ReadPin(encoder_b_GPIO_Port, encoder_b_Pin))
            ticks--;
        else
            ticks++;
    }
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1){
        if(btBuffer[0]=='s'){ //Square 
            state = 0;
            HAL_UART_Receive_IT(&huart1, btBuffer, 12);
            return;   
        }

        float f_buffer[12];
        for (int i = 0; i < 12; i++)
            f_buffer[i] = (float)(btBuffer[i]-'0');
        target_x = f_buffer[1]*10 + f_buffer[2] + f_buffer[4]*0.1 + f_buffer[5]*0.01;
        target_y = f_buffer[7]*10 + f_buffer[8] + f_buffer[10]*0.1 + f_buffer[11]*0.01;

        if(btBuffer[0]=='-')
        	target_x *= -1;
        if(btBuffer[6]=='-')
        	target_y *= -1;

        target_distance = sqrt(pow(target_x - x_pos, 2) + pow(target_y - y_pos, 2));
        target_angle = atan2(target_x - x_pos, target_y - y_pos) * 180 / pi;
        if (target_angle < 0)
            target_angle += 360;
        state = -4;

        HAL_UART_Receive_IT(&huart1, btBuffer, 12); // Enabling interrupt receive
        toggle_interrupt = !toggle_interrupt;
    }
}

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

#ifdef USE_FULL_ASSERT
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
