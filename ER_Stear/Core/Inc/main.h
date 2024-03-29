/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

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
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define SIDEB_ENC_X_Pin GPIO_PIN_4
#define SIDEB_ENC_X_GPIO_Port GPIOA
#define SIDEB_ENC_X_EXTI_IRQn EXTI4_IRQn
#define SIDEB_IN_Pin GPIO_PIN_6
#define SIDEB_IN_GPIO_Port GPIOA
#define SIDEB_IN__Pin GPIO_PIN_7
#define SIDEB_IN__GPIO_Port GPIOA
#define SIDEA_IN_Pin GPIO_PIN_0
#define SIDEA_IN_GPIO_Port GPIOB
#define SIDEA_ENC_A_Pin GPIO_PIN_8
#define SIDEA_ENC_A_GPIO_Port GPIOA
#define SIDEA_ENC_B_Pin GPIO_PIN_9
#define SIDEA_ENC_B_GPIO_Port GPIOA
#define SIDEA_ENC_X_Pin GPIO_PIN_10
#define SIDEA_ENC_X_GPIO_Port GPIOA
#define SIDEA_ENC_X_EXTI_IRQn EXTI15_10_IRQn
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define SIDEB_ENC_A_Pin GPIO_PIN_15
#define SIDEB_ENC_A_GPIO_Port GPIOA
#define SIDEB_ENC_B_Pin GPIO_PIN_3
#define SIDEB_ENC_B_GPIO_Port GPIOB
#define DIS_Pin GPIO_PIN_4
#define DIS_GPIO_Port GPIOB
#define SIDEA_IN__Pin GPIO_PIN_7
#define SIDEA_IN__GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
