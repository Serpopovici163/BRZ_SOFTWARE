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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_SPI_CAN_nCS_Pin GPIO_PIN_3
#define CAN_SPI_CAN_nCS_GPIO_Port GPIOE
#define EEPROM_nHOLD_Pin GPIO_PIN_4
#define EEPROM_nHOLD_GPIO_Port GPIOE
#define CAN_SPI_EEPROM_nCS_Pin GPIO_PIN_13
#define CAN_SPI_EEPROM_nCS_GPIO_Port GPIOC
#define M1_4_TRIG_Pin GPIO_PIN_0
#define M1_4_TRIG_GPIO_Port GPIOC
#define M1_STAT_34_Pin GPIO_PIN_1
#define M1_STAT_34_GPIO_Port GPIOC
#define HORN_nCLIP_Pin GPIO_PIN_2
#define HORN_nCLIP_GPIO_Port GPIOC
#define HORN_nRST_Pin GPIO_PIN_3
#define HORN_nRST_GPIO_Port GPIOC
#define M1_3_TRIG_Pin GPIO_PIN_0
#define M1_3_TRIG_GPIO_Port GPIOA
#define M1_2_TRIG_Pin GPIO_PIN_1
#define M1_2_TRIG_GPIO_Port GPIOA
#define M1_STAT_12_Pin GPIO_PIN_2
#define M1_STAT_12_GPIO_Port GPIOA
#define M1_1_TRIG_Pin GPIO_PIN_3
#define M1_1_TRIG_GPIO_Port GPIOA
#define MOS_SPI_M0_ADC_nCS_Pin GPIO_PIN_4
#define MOS_SPI_M0_ADC_nCS_GPIO_Port GPIOA
#define M0_4_TRIG_Pin GPIO_PIN_4
#define M0_4_TRIG_GPIO_Port GPIOC
#define M0_3_TRIG_Pin GPIO_PIN_5
#define M0_3_TRIG_GPIO_Port GPIOC
#define M0_34_STAT_Pin GPIO_PIN_0
#define M0_34_STAT_GPIO_Port GPIOB
#define M0_2_TRIG_Pin GPIO_PIN_1
#define M0_2_TRIG_GPIO_Port GPIOB
#define M0_1_TRIG_Pin GPIO_PIN_2
#define M0_1_TRIG_GPIO_Port GPIOB
#define M0_STAT_12_Pin GPIO_PIN_7
#define M0_STAT_12_GPIO_Port GPIOE
#define VCC_I_nFLT_Pin GPIO_PIN_8
#define VCC_I_nFLT_GPIO_Port GPIOE
#define LED_A_SUP_nFLT_Pin GPIO_PIN_10
#define LED_A_SUP_nFLT_GPIO_Port GPIOE
#define LED_B_SUP_nFLT_Pin GPIO_PIN_11
#define LED_B_SUP_nFLT_GPIO_Port GPIOE
#define LED_C_SUP_nFLT_Pin GPIO_PIN_12
#define LED_C_SUP_nFLT_GPIO_Port GPIOE
#define LED_D_SUP_nFLT_Pin GPIO_PIN_13
#define LED_D_SUP_nFLT_GPIO_Port GPIOE
#define BCM_SUP_nFLT_Pin GPIO_PIN_14
#define BCM_SUP_nFLT_GPIO_Port GPIOE
#define HORN_SUP_nFLT_Pin GPIO_PIN_15
#define HORN_SUP_nFLT_GPIO_Port GPIOE
#define HORN_AMP_EN_Pin GPIO_PIN_11
#define HORN_AMP_EN_GPIO_Port GPIOB
#define HORN_I_nFLT_Pin GPIO_PIN_12
#define HORN_I_nFLT_GPIO_Port GPIOB
#define HORN_nFLT_Pin GPIO_PIN_15
#define HORN_nFLT_GPIO_Port GPIOB
#define MOS_SPI_ADC_0_nCS_Pin GPIO_PIN_8
#define MOS_SPI_ADC_0_nCS_GPIO_Port GPIOD
#define MOS_SPI_M3_ADC_nCS_Pin GPIO_PIN_9
#define MOS_SPI_M3_ADC_nCS_GPIO_Port GPIOD
#define MOS_SPI_ADC_2_nCS_Pin GPIO_PIN_10
#define MOS_SPI_ADC_2_nCS_GPIO_Port GPIOD
#define MOS_SPI_ADC_1_nCS_Pin GPIO_PIN_11
#define MOS_SPI_ADC_1_nCS_GPIO_Port GPIOD
#define IO_CHK_0_INTB_Pin GPIO_PIN_13
#define IO_CHK_0_INTB_GPIO_Port GPIOD
#define IO_CHK_0_INTA_Pin GPIO_PIN_14
#define IO_CHK_0_INTA_GPIO_Port GPIOD
#define IO_CHK_0_nRST_Pin GPIO_PIN_15
#define IO_CHK_0_nRST_GPIO_Port GPIOD
#define MOS_SPI_M1_ADC_nCS_Pin GPIO_PIN_7
#define MOS_SPI_M1_ADC_nCS_GPIO_Port GPIOC
#define MOS_SPI_M2_ADC_nCS_Pin GPIO_PIN_8
#define MOS_SPI_M2_ADC_nCS_GPIO_Port GPIOC
#define M3_4_TRIG_Pin GPIO_PIN_8
#define M3_4_TRIG_GPIO_Port GPIOA
#define M3_STAT_34_Pin GPIO_PIN_9
#define M3_STAT_34_GPIO_Port GPIOA
#define M3_3_TRIG_Pin GPIO_PIN_10
#define M3_3_TRIG_GPIO_Port GPIOA
#define M3_2_TRIG_Pin GPIO_PIN_11
#define M3_2_TRIG_GPIO_Port GPIOA
#define M3_STAT_12_Pin GPIO_PIN_12
#define M3_STAT_12_GPIO_Port GPIOA
#define M3_1_TRIG_Pin GPIO_PIN_15
#define M3_1_TRIG_GPIO_Port GPIOA
#define M2_4_TRIG_Pin GPIO_PIN_0
#define M2_4_TRIG_GPIO_Port GPIOD
#define M2_STAT_34_Pin GPIO_PIN_1
#define M2_STAT_34_GPIO_Port GPIOD
#define M2_3_TRIG_Pin GPIO_PIN_3
#define M2_3_TRIG_GPIO_Port GPIOD
#define M2_2_TRIG_Pin GPIO_PIN_4
#define M2_2_TRIG_GPIO_Port GPIOD
#define M2_STAT_12_Pin GPIO_PIN_5
#define M2_STAT_12_GPIO_Port GPIOD
#define M2_1_TRIG_Pin GPIO_PIN_6
#define M2_1_TRIG_GPIO_Port GPIOD
#define IO_CHK_1_nRST_Pin GPIO_PIN_7
#define IO_CHK_1_nRST_GPIO_Port GPIOD
#define IO_CHK_1_INTB_Pin GPIO_PIN_4
#define IO_CHK_1_INTB_GPIO_Port GPIOB
#define IO_CHK_1_INTA_Pin GPIO_PIN_5
#define IO_CHK_1_INTA_GPIO_Port GPIOB
#define MCU_G_LED_Pin GPIO_PIN_0
#define MCU_G_LED_GPIO_Port GPIOE
#define MCU_R_LED_Pin GPIO_PIN_1
#define MCU_R_LED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
