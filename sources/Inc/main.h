/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f0xx_hal.h"

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
#define CHARGER_EN_Pin GPIO_PIN_13
#define CHARGER_EN_GPIO_Port GPIOC
#define PWR_LED_Pin GPIO_PIN_14
#define PWR_LED_GPIO_Port GPIOC
#define FAN_EN_Pin GPIO_PIN_15
#define FAN_EN_GPIO_Port GPIOC
#define AMP_SHDN_Pin GPIO_PIN_0
#define AMP_SHDN_GPIO_Port GPIOF
#define TEMP1_Pin GPIO_PIN_0
#define TEMP1_GPIO_Port GPIOA
#define TEMP2_Pin GPIO_PIN_1
#define TEMP2_GPIO_Port GPIOA
#define TEMP3_Pin GPIO_PIN_2
#define TEMP3_GPIO_Port GPIOA
#define V_FUSE_Pin GPIO_PIN_3
#define V_FUSE_GPIO_Port GPIOA
#define SOUND_Pin GPIO_PIN_4
#define SOUND_GPIO_Port GPIOA
#define V_CAP__Pin GPIO_PIN_5
#define V_CAP__GPIO_Port GPIOA
#define V_CAP_A6_Pin GPIO_PIN_6
#define V_CAP_A6_GPIO_Port GPIOA
#define V_VCC_Pin GPIO_PIN_7
#define V_VCC_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_0
#define FAN_GPIO_Port GPIOB
#define TACHO_Pin GPIO_PIN_1
#define TACHO_GPIO_Port GPIOB
#define DISCHARGE_Pin GPIO_PIN_2
#define DISCHARGE_GPIO_Port GPIOB
#define FOOT_SW_Pin GPIO_PIN_10
#define FOOT_SW_GPIO_Port GPIOB
#define FOOT_SW_EXTI_IRQn EXTI4_15_IRQn
#define TRIGGER_Pin GPIO_PIN_11
#define TRIGGER_GPIO_Port GPIOB
#define FLSH_CS_Pin GPIO_PIN_12
#define FLSH_CS_GPIO_Port GPIOB
#define FLSH_SCK_Pin GPIO_PIN_13
#define FLSH_SCK_GPIO_Port GPIOB
#define FLSH_MISO_Pin GPIO_PIN_14
#define FLSH_MISO_GPIO_Port GPIOB
#define FLSH_MOSI_Pin GPIO_PIN_15
#define FLSH_MOSI_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_8
#define LED_BLUE_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_10
#define LED_RED_GPIO_Port GPIOA
#define ST7735_CS_Pin GPIO_PIN_15
#define ST7735_CS_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_3
#define LCD_SCK_GPIO_Port GPIOB
#define ST7735_DC_Pin GPIO_PIN_4
#define ST7735_DC_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_5
#define LCD_MOSI_GPIO_Port GPIOB
#define ST7735_RES_Pin GPIO_PIN_6
#define ST7735_RES_GPIO_Port GPIOB
#define ROT_B_Pin GPIO_PIN_7
#define ROT_B_GPIO_Port GPIOB
#define ROT_SW_Pin GPIO_PIN_8
#define ROT_SW_GPIO_Port GPIOB
#define ROT_A_Pin GPIO_PIN_9
#define ROT_A_GPIO_Port GPIOB
#define ROT_A_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
