/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LCD_WR_L_Pin GPIO_PIN_13
#define LCD_WR_L_GPIO_Port GPIOC
#define LCD_DATA_COMMAND_Pin GPIO_PIN_14
#define LCD_DATA_COMMAND_GPIO_Port GPIOC
#define LCD_RESET_L_Pin GPIO_PIN_15
#define LCD_RESET_L_GPIO_Port GPIOC
#define DB0_Pin GPIO_PIN_0
#define DB0_GPIO_Port GPIOC
#define DB1_Pin GPIO_PIN_1
#define DB1_GPIO_Port GPIOC
#define DB2_Pin GPIO_PIN_2
#define DB2_GPIO_Port GPIOC
#define DB3_Pin GPIO_PIN_3
#define DB3_GPIO_Port GPIOC
#define DB4_Pin GPIO_PIN_4
#define DB4_GPIO_Port GPIOC
#define DB5_Pin GPIO_PIN_5
#define DB5_GPIO_Port GPIOC
#define KP_COL1_Pin GPIO_PIN_0
#define KP_COL1_GPIO_Port GPIOB
#define KP_COL2_Pin GPIO_PIN_1
#define KP_COL2_GPIO_Port GPIOB
#define KP_COL3_Pin GPIO_PIN_2
#define KP_COL3_GPIO_Port GPIOB
#define KP_R3_Pin GPIO_PIN_10
#define KP_R3_GPIO_Port GPIOB
#define KP_R4_Pin GPIO_PIN_11
#define KP_R4_GPIO_Port GPIOB
#define KP_R5_Pin GPIO_PIN_12
#define KP_R5_GPIO_Port GPIOB
#define KP_R6_Pin GPIO_PIN_13
#define KP_R6_GPIO_Port GPIOB
#define KP_RFU_Pin GPIO_PIN_14
#define KP_RFU_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_6
#define DB6_GPIO_Port GPIOC
#define DB7_Pin GPIO_PIN_7
#define DB7_GPIO_Port GPIOC
#define LCD_OE_L_Pin GPIO_PIN_8
#define LCD_OE_L_GPIO_Port GPIOC
#define LCD_DIR_Pin GPIO_PIN_9
#define LCD_DIR_GPIO_Port GPIOC
#define HEAT1_Pin GPIO_PIN_11
#define HEAT1_GPIO_Port GPIOA
#define HEAT2_Pin GPIO_PIN_12
#define HEAT2_GPIO_Port GPIOA
#define LCD_RD_L_Pin GPIO_PIN_10
#define LCD_RD_L_GPIO_Port GPIOC
#define LCD_CS1_L_Pin GPIO_PIN_11
#define LCD_CS1_L_GPIO_Port GPIOC
#define LCD_CS2_L_Pin GPIO_PIN_12
#define LCD_CS2_L_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB
#define KP_R1_Pin GPIO_PIN_8
#define KP_R1_GPIO_Port GPIOB
#define KP_R2_Pin GPIO_PIN_9
#define KP_R2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/// @brief  Possible STM32 system reset causes
typedef enum reset_cause_e
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_LOW_POWER_RESET,
    RESET_CAUSE_WINDOW_WATCHDOG_RESET,
    RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
    RESET_CAUSE_SOFTWARE_RESET,
    RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
    RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
    RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
