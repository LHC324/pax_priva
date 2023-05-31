/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifdef Error_Handler
#undef Error_Handler
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DI1_Pin GPIO_PIN_2
#define DI1_GPIO_Port GPIOE
#define DI1_EXTI_IRQn EXTI2_IRQn
#define DI2_Pin GPIO_PIN_3
#define DI2_GPIO_Port GPIOE
#define DI2_EXTI_IRQn EXTI3_IRQn
#define DI3_Pin GPIO_PIN_4
#define DI3_GPIO_Port GPIOE
#define DI3_EXTI_IRQn EXTI4_IRQn
#define DI4_Pin GPIO_PIN_5
#define DI4_GPIO_Port GPIOE
#define DI4_EXTI_IRQn EXTI9_5_IRQn
#define DI5_Pin GPIO_PIN_6
#define DI5_GPIO_Port GPIOE
#define DI5_EXTI_IRQn EXTI9_5_IRQn
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define Q22_Pin GPIO_PIN_12
#define Q22_GPIO_Port GPIOE
#define Q21_Pin GPIO_PIN_13
#define Q21_GPIO_Port GPIOE
#define Q20_Pin GPIO_PIN_14
#define Q20_GPIO_Port GPIOE
#define Q19_Pin GPIO_PIN_15
#define Q19_GPIO_Port GPIOE
#define Q18_Pin GPIO_PIN_12
#define Q18_GPIO_Port GPIOB
#define Q17_Pin GPIO_PIN_13
#define Q17_GPIO_Port GPIOB
#define Q16_Pin GPIO_PIN_14
#define Q16_GPIO_Port GPIOB
#define Q15_Pin GPIO_PIN_15
#define Q15_GPIO_Port GPIOB
#define Q14_Pin GPIO_PIN_8
#define Q14_GPIO_Port GPIOD
#define Q13_Pin GPIO_PIN_9
#define Q13_GPIO_Port GPIOD
#define Q12_Pin GPIO_PIN_10
#define Q12_GPIO_Port GPIOD
#define Q11_Pin GPIO_PIN_11
#define Q11_GPIO_Port GPIOD
#define Q10_Pin GPIO_PIN_12
#define Q10_GPIO_Port GPIOD
#define Q9_Pin GPIO_PIN_13
#define Q9_GPIO_Port GPIOD
#define Q8_Pin GPIO_PIN_14
#define Q8_GPIO_Port GPIOD
#define Q7_Pin GPIO_PIN_15
#define Q7_GPIO_Port GPIOD
#define Q6_Pin GPIO_PIN_6
#define Q6_GPIO_Port GPIOC
#define Q5_Pin GPIO_PIN_7
#define Q5_GPIO_Port GPIOC
#define Q4_Pin GPIO_PIN_8
#define Q4_GPIO_Port GPIOC
#define Q3_Pin GPIO_PIN_9
#define Q3_GPIO_Port GPIOC
#define Q2_Pin GPIO_PIN_8
#define Q2_GPIO_Port GPIOA
#define Q1_Pin GPIO_PIN_9
#define Q1_GPIO_Port GPIOA
#define Q0_Pin GPIO_PIN_10
#define Q0_GPIO_Port GPIOA
#define LTE_RESET_Pin GPIO_PIN_15
#define LTE_RESET_GPIO_Port GPIOA
#define LTE_NET_Pin GPIO_PIN_0
#define LTE_NET_GPIO_Port GPIOD
#define LTE_RELOAD_Pin GPIO_PIN_1
#define LTE_RELOAD_GPIO_Port GPIOD
#define WDI_Pin GPIO_PIN_3
#define WDI_GPIO_Port GPIOD
#define LORA_STATUS_Pin GPIO_PIN_4
#define LORA_STATUS_GPIO_Port GPIOD
#define LORA_WAKEUP_Pin GPIO_PIN_7
#define LORA_WAKEUP_GPIO_Port GPIOD
#define LORA_RELOAD_Pin GPIO_PIN_3
#define LORA_RELOAD_GPIO_Port GPIOB
#define LTE_LINK_Pin GPIO_PIN_4
#define LTE_LINK_GPIO_Port GPIOB
#define ETH_CFG_Pin GPIO_PIN_5
#define ETH_CFG_GPIO_Port GPIOB
#define ETH_RST_Pin GPIO_PIN_8
#define ETH_RST_GPIO_Port GPIOB
#define ETH_485EN_Pin GPIO_PIN_9
#define ETH_485EN_GPIO_Port GPIOB
#define ETH_LINK_Pin GPIO_PIN_0
#define ETH_LINK_GPIO_Port GPIOE
#define DI0_Pin GPIO_PIN_1
#define DI0_GPIO_Port GPIOE
#define DI0_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
