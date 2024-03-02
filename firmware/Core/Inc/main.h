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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

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
#define GP_LED_Pin LL_GPIO_PIN_3
#define GP_LED_GPIO_Port GPIOE
#define ICM40609D_INT1_Pin LL_GPIO_PIN_0
#define ICM40609D_INT1_GPIO_Port GPIOA
#define ICM40609D_INT2_Pin LL_GPIO_PIN_2
#define ICM40609D_INT2_GPIO_Port GPIOA
#define ICM40609D_CS_Pin LL_GPIO_PIN_4
#define ICM40609D_CS_GPIO_Port GPIOA
#define BMP_INT_Pin LL_GPIO_PIN_10
#define BMP_INT_GPIO_Port GPIOE
#define BMP_CS_Pin LL_GPIO_PIN_11
#define BMP_CS_GPIO_Port GPIOE
#define BN_CS_Pin LL_GPIO_PIN_12
#define BN_CS_GPIO_Port GPIOB
#define BN_INT_Pin LL_GPIO_PIN_8
#define BN_INT_GPIO_Port GPIOD
#define BN_NRST_Pin LL_GPIO_PIN_9
#define BN_NRST_GPIO_Port GPIOD
#define SDIO_nDET_Pin LL_GPIO_PIN_10
#define SDIO_nDET_GPIO_Port GPIOD
#define BN_PS0_WAKE_Pin LL_GPIO_PIN_11
#define BN_PS0_WAKE_GPIO_Port GPIOD
#define ICM42688P_CS_Pin LL_GPIO_PIN_15
#define ICM42688P_CS_GPIO_Port GPIOA
#define ICM42688P_INT1_Pin LL_GPIO_PIN_6
#define ICM42688P_INT1_GPIO_Port GPIOB
#define ICM42688P_INT2_Pin LL_GPIO_PIN_7
#define ICM42688P_INT2_GPIO_Port GPIOB
#define EN_3V3_Pin LL_GPIO_PIN_1
#define EN_3V3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define ENABLE_3V3_SENSOR_BOARD     LL_GPIO_SetOutputPin(EN_3V3_GPIO_Port, EN_3V3_Pin)
#define DISABLE_3V3_SENSOR_BOARD    LL_GPIO_ResetOutputPin(EN_3V3_GPIO_Port, EN_3V3_Pin)
#define CS_SELECT(x)                LL_GPIO_ResetOutputPin(x##_CS_GPIO_Port, x##_CS_Pin)
#define CS_DESELECT(x)              LL_GPIO_SetOutputPin(x##_CS_GPIO_Port, x##_CS_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
