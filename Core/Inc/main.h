/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STAT_Pin GPIO_PIN_2
#define STAT_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_4
#define SPI2_CS_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_0
#define SW_GPIO_Port GPIOB
#define LED_SW1_Pin GPIO_PIN_2
#define LED_SW1_GPIO_Port GPIOB
#define STEAM_Pin GPIO_PIN_7
#define STEAM_GPIO_Port GPIOE
#define LVL_Pin GPIO_PIN_8
#define LVL_GPIO_Port GPIOE
#define ENC_SW_Pin GPIO_PIN_10
#define ENC_SW_GPIO_Port GPIOE
#define WL_Pin GPIO_PIN_13
#define WL_GPIO_Port GPIOE
#define FLT_Pin GPIO_PIN_14
#define FLT_GPIO_Port GPIOE
#define SPI2_CS4_Pin GPIO_PIN_10
#define SPI2_CS4_GPIO_Port GPIOD
#define SPI2_CS3_Pin GPIO_PIN_11
#define SPI2_CS3_GPIO_Port GPIOD
#define SPI2_CS2_Pin GPIO_PIN_12
#define SPI2_CS2_GPIO_Port GPIOD
#define SPI2_CS1_Pin GPIO_PIN_13
#define SPI2_CS1_GPIO_Port GPIOD
#define SPI2_CS5_Pin GPIO_PIN_14
#define SPI2_CS5_GPIO_Port GPIOD
#define BLDC_DIR_Pin GPIO_PIN_9
#define BLDC_DIR_GPIO_Port GPIOC
#define BLDC_EN_Pin GPIO_PIN_8
#define BLDC_EN_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define DEBUG 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
