/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define PWMA_Pin GPIO_PIN_2
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_3
#define PWMB_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOA
#define XDA_Pin GPIO_PIN_8
#define XDA_GPIO_Port GPIOE
#define XCL_Pin GPIO_PIN_9
#define XCL_GPIO_Port GPIOE
#define ADC_Pin GPIO_PIN_10
#define ADC_GPIO_Port GPIOE
#define INT_Pin GPIO_PIN_11
#define INT_GPIO_Port GPIOE
#define OLED_CLK_Pin GPIO_PIN_13
#define OLED_CLK_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_15
#define OLED_MOSI_GPIO_Port GPIOB
#define ENCOA2_Pin GPIO_PIN_12
#define ENCOA2_GPIO_Port GPIOD
#define ENCOA1_Pin GPIO_PIN_13
#define ENCOA1_GPIO_Port GPIOD
#define ENCOB2_Pin GPIO_PIN_6
#define ENCOB2_GPIO_Port GPIOC
#define ENCOB1_Pin GPIO_PIN_7
#define ENCOB1_GPIO_Port GPIOC
#define STEER1_Pin GPIO_PIN_8
#define STEER1_GPIO_Port GPIOC
#define STEER2_Pin GPIO_PIN_9
#define STEER2_GPIO_Port GPIOC
#define STEER3_Pin GPIO_PIN_8
#define STEER3_GPIO_Port GPIOA
#define CAM_TX_Pin GPIO_PIN_9
#define CAM_TX_GPIO_Port GPIOA
#define CAM_RX_Pin GPIO_PIN_10
#define CAM_RX_GPIO_Port GPIOA
#define STEER4_Pin GPIO_PIN_11
#define STEER4_GPIO_Port GPIOA
#define SM1_TX_Pin GPIO_PIN_10
#define SM1_TX_GPIO_Port GPIOC
#define SM1_RX_Pin GPIO_PIN_11
#define SM1_RX_GPIO_Port GPIOC
#define SM2_TX_Pin GPIO_PIN_12
#define SM2_TX_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOD
#define ENA1_Pin GPIO_PIN_1
#define ENA1_GPIO_Port GPIOD
#define SM2_RX_Pin GPIO_PIN_2
#define SM2_RX_GPIO_Port GPIOD
#define IR_Sensor_TX_Pin GPIO_PIN_5
#define IR_Sensor_TX_GPIO_Port GPIOD
#define IR_Sensor_RX_Pin GPIO_PIN_6
#define IR_Sensor_RX_GPIO_Port GPIOD
#define PUL1_Pin GPIO_PIN_4
#define PUL1_GPIO_Port GPIOB
#define PUL2_Pin GPIO_PIN_5
#define PUL2_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_8
#define DIR2_GPIO_Port GPIOB
#define ENA2_Pin GPIO_PIN_9
#define ENA2_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_0
#define OLED_CS_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_1
#define OLED_DC_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
