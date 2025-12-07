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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "OLED.h"
#include "PLL.h"
#include "arm_math.h"
#include "CNTL_PI_F.h"
#include "Totem_PFC.h"
#include "Notch_Fltr_F.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define 	period 				1800
#define 	deta_theat 			0.01570796f

	/*  ADC_Calibration_Coefficient  */
#define 	Vac_Offset 			2077.0f
#define		Vac_Factor			0.02571f

#define 	IL_Offset  	 		2077.0f
#define 	IL_Factor			0.003409f

#define		Vbus_Offset			0.0f
#define		Vbus_Factor			0.011713f
#define		Vbus_Factor_B		0.0f

#define		Ibus_Offset			20.0f
#define		Ibus_Factor			0.002786f
	/*  END  */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define PWMA_H_Pin GPIO_PIN_8
#define PWMA_H_GPIO_Port GPIOA
#define PWMB_H_Pin GPIO_PIN_9
#define PWMB_H_GPIO_Port GPIOA
#define PWMA_L_Pin GPIO_PIN_11
#define PWMA_L_GPIO_Port GPIOA
#define PWMB_L_Pin GPIO_PIN_12
#define PWMB_L_GPIO_Port GPIOA
#define Gate_Driver_ENABLE_Pin GPIO_PIN_3
#define Gate_Driver_ENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
