/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
void _Error_Handler(char *file, uint32_t line);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_TIMER_BASE_PERIOD 500
#define ADDR_SEL_Pin GPIO_PIN_14
#define ADDR_SEL_GPIO_Port GPIOC
#define ADDR_SEL_EXTI_IRQn EXTI15_10_IRQn
#define DIAG0_Pin GPIO_PIN_0
#define DIAG0_GPIO_Port GPIOA
#define DIAG0_EXTI_IRQn EXTI0_IRQn
#define DIAG1_Pin GPIO_PIN_1
#define DIAG1_GPIO_Port GPIOA
#define DIAG1_EXTI_IRQn EXTI1_IRQn
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_0
#define SPI1_NSS_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_1
#define RESET_GPIO_Port GPIOB
#define START_Pin GPIO_PIN_11
#define START_GPIO_Port GPIOA
#define FREEZE_Pin GPIO_PIN_15
#define FREEZE_GPIO_Port GPIOA
#define INTR_Pin GPIO_PIN_4
#define INTR_GPIO_Port GPIOB
#define INTR_EXTI_IRQn EXTI4_IRQn
#define TARGET_REACHED_Pin GPIO_PIN_5
#define TARGET_REACHED_GPIO_Port GPIOB
#define TARGET_REACHED_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
#define EEPROM_STORAGE_ADDR 0x57
#define EEPROM_STORAGE_TEST_AREA_START_ADDR 0x7B
#define EEPROM_EUI_ADDR 0xFA
#define R_SENSE 0.033f
#define HOLD_CURRENT_MULTIPLIER 0.5f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
