/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define encoder_a_Pin GPIO_PIN_5
#define encoder_a_GPIO_Port GPIOA
#define encoder_a_EXTI_IRQn EXTI9_5_IRQn
#define encoder_b_Pin GPIO_PIN_6
#define encoder_b_GPIO_Port GPIOA
#define servo_pwm_Pin GPIO_PIN_8
#define servo_pwm_GPIO_Port GPIOA
#define bt_tx_Pin GPIO_PIN_9
#define bt_tx_GPIO_Port GPIOA
#define bt_rx_Pin GPIO_PIN_10
#define bt_rx_GPIO_Port GPIOA
#define motor_a_Pin GPIO_PIN_11
#define motor_a_GPIO_Port GPIOA
#define motor_b_Pin GPIO_PIN_12
#define motor_b_GPIO_Port GPIOA
#define bno_scl_Pin GPIO_PIN_6
#define bno_scl_GPIO_Port GPIOB
#define bno_sda_Pin GPIO_PIN_7
#define bno_sda_GPIO_Port GPIOB
#define motor_pwm_Pin GPIO_PIN_8
#define motor_pwm_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Trajectory constants
static const float target_distance = 10.0;
static const float target_tolerance = 0.04;
static const float angle_tolerance = 1.0;
static const float distance_offset = 1.05;

// Motor constants
static const float wheel_diameter = 0.064;
static const float gear_ratio = 2;
static const float pi = 3.141592;
static const float ppr = 670;
static const int min_pwm = 40000;
// 1 motor rev = 2*pi*wheel_radius

// Speed PID constants
static const float speed_kp = 97.3574;
static const float speed_ki = 449.35;
static const float speed_kd = 0.0;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
