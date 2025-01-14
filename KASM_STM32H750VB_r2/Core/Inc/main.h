/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM1_CH3_PHASE_Pin GPIO_PIN_2
#define TIM1_CH3_PHASE_GPIO_Port GPIOE
#define TIM1_CH4_PHASE_Pin GPIO_PIN_3
#define TIM1_CH4_PHASE_GPIO_Port GPIOE
#define TIM2_CH1_PHASE_Pin GPIO_PIN_4
#define TIM2_CH1_PHASE_GPIO_Port GPIOE
#define TIM4_CH4_PHASE_Pin GPIO_PIN_13
#define TIM4_CH4_PHASE_GPIO_Port GPIOC
#define TIM5_CH2_PHASE_Pin GPIO_PIN_14
#define TIM5_CH2_PHASE_GPIO_Port GPIOC
#define TIM5_CH3_PHASE_Pin GPIO_PIN_15
#define TIM5_CH3_PHASE_GPIO_Port GPIOC
#define LDC1_INTB_Pin GPIO_PIN_3
#define LDC1_INTB_GPIO_Port GPIOC
#define TIM2_CH4_LDC_Pin GPIO_PIN_3
#define TIM2_CH4_LDC_GPIO_Port GPIOA
#define LDC0_INTB_Pin GPIO_PIN_4
#define LDC0_INTB_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define TIM8_CH4_PHASE_Pin GPIO_PIN_7
#define TIM8_CH4_PHASE_GPIO_Port GPIOE
#define TIM12_CH2_PHASE_Pin GPIO_PIN_8
#define TIM12_CH2_PHASE_GPIO_Port GPIOE
#define TIM13_CH1_PHASE_Pin GPIO_PIN_10
#define TIM13_CH1_PHASE_GPIO_Port GPIOE
#define TIM14_CH1_PHASE_Pin GPIO_PIN_12
#define TIM14_CH1_PHASE_GPIO_Port GPIOE
#define TIM15_CH1_PHASE_Pin GPIO_PIN_15
#define TIM15_CH1_PHASE_GPIO_Port GPIOE
#define TIM4_CH1_PHASE_Pin GPIO_PIN_10
#define TIM4_CH1_PHASE_GPIO_Port GPIOB
#define TIM4_CH2_PHASE_Pin GPIO_PIN_11
#define TIM4_CH2_PHASE_GPIO_Port GPIOB
#define SPI6_MUX_OE_n_Pin GPIO_PIN_13
#define SPI6_MUX_OE_n_GPIO_Port GPIOB
#define HRTIM_CHC1_PHASE_Pin GPIO_PIN_8
#define HRTIM_CHC1_PHASE_GPIO_Port GPIOD
#define HRTIM_CHC2_PHASE_Pin GPIO_PIN_9
#define HRTIM_CHC2_PHASE_GPIO_Port GPIOD
#define HRTIM_CHD1_PHASE_Pin GPIO_PIN_10
#define HRTIM_CHD1_PHASE_GPIO_Port GPIOD
#define HRTIM_CHD2_PHASE_Pin GPIO_PIN_11
#define HRTIM_CHD2_PHASE_GPIO_Port GPIOD
#define ACTUATOR_ENABLE_MCU_Pin GPIO_PIN_15
#define ACTUATOR_ENABLE_MCU_GPIO_Port GPIOA
#define TIM4_CH3_PHASE_Pin GPIO_PIN_12
#define TIM4_CH3_PHASE_GPIO_Port GPIOC
#define TIM15_CH2_PHASE_Pin GPIO_PIN_0
#define TIM15_CH2_PHASE_GPIO_Port GPIOD
#define TIM16_CH1_PHASE_Pin GPIO_PIN_1
#define TIM16_CH1_PHASE_GPIO_Port GPIOD
#define HRTIM_CHA1_PHASE_Pin GPIO_PIN_4
#define HRTIM_CHA1_PHASE_GPIO_Port GPIOD
#define HRTIM_CHA2_PHASE_Pin GPIO_PIN_5
#define HRTIM_CHA2_PHASE_GPIO_Port GPIOD
#define HRTIM_CHB1_PHASE_Pin GPIO_PIN_6
#define HRTIM_CHB1_PHASE_GPIO_Port GPIOD
#define HRTIM_CHB2_PHASE_Pin GPIO_PIN_7
#define HRTIM_CHB2_PHASE_GPIO_Port GPIOD
#define TIM1_CH1_PHASE_Pin GPIO_PIN_0
#define TIM1_CH1_PHASE_GPIO_Port GPIOE
#define TIM1_CH2_PHASE_Pin GPIO_PIN_1
#define TIM1_CH2_PHASE_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
