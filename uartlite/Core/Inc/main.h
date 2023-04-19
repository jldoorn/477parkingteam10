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
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define ESP_STA
#define SWITCH_DIR_OUT 0
#define SWITCH_DIR_IN 1

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WIFI_EN_Pin LL_GPIO_PIN_2
#define WIFI_EN_GPIO_Port GPIOC
#define KEYPADI1_Pin LL_GPIO_PIN_3
#define KEYPADI1_GPIO_Port GPIOC
#define KEYPADI2_Pin LL_GPIO_PIN_4
#define KEYPADI2_GPIO_Port GPIOC
#define KEYPADI3_Pin LL_GPIO_PIN_5
#define KEYPADI3_GPIO_Port GPIOC
#define DEBUG_7_Pin LL_GPIO_PIN_0
#define DEBUG_7_GPIO_Port GPIOB
#define DEBUG_8_Pin LL_GPIO_PIN_1
#define DEBUG_8_GPIO_Port GPIOB
#define OUTFLOW_BTN_Pin LL_GPIO_PIN_2
#define OUTFLOW_BTN_GPIO_Port GPIOB
#define LED2_Pin LL_GPIO_PIN_6
#define LED2_GPIO_Port GPIOC
#define KEYPADO1_Pin LL_GPIO_PIN_7
#define KEYPADO1_GPIO_Port GPIOC
#define USER_IN_SW4_Pin LL_GPIO_PIN_8
#define USER_IN_SW4_GPIO_Port GPIOC
#define USER_IN_SW3_Pin LL_GPIO_PIN_9
#define USER_IN_SW3_GPIO_Port GPIOC
#define USER_IN_SW2_Pin LL_GPIO_PIN_8
#define USER_IN_SW2_GPIO_Port GPIOA
#define USER_IN_SW1_Pin LL_GPIO_PIN_9
#define USER_IN_SW1_GPIO_Port GPIOA
#define PROX_TRIG_Pin LL_GPIO_PIN_10
#define PROX_TRIG_GPIO_Port GPIOA
#define PROX_MEAS_Pin LL_GPIO_PIN_11
#define PROX_MEAS_GPIO_Port GPIOA
#define WIFI_RST_Pin LL_GPIO_PIN_10
#define WIFI_RST_GPIO_Port GPIOC
#define STA_ID_0_Pin LL_GPIO_PIN_4
#define STA_ID_0_GPIO_Port GPIOB
#define STA_ID_1_Pin LL_GPIO_PIN_5
#define STA_ID_1_GPIO_Port GPIOB
#define STA_ID_2_Pin LL_GPIO_PIN_6
#define STA_ID_2_GPIO_Port GPIOB
#define STA_DIR_Pin LL_GPIO_PIN_7
#define STA_DIR_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
