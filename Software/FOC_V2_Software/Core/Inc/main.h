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

#define DATA_HEAD 0x55
#define DATA_TAIL_PREV 0x0d
#define DATA_TAIL 0x0a
    
#define MODE_CSP 0x08
#define MODE_CSV 0x09
#define MODE_CST 0x0a
    
#define JOINT_ID 0
#define JOINT_ID_OFFSET (JOINT_ID * 0x1000)
    
#define ADDR_MODE_OF_OPERATION (0X3001 + JOINT_ID_OFFSET)
#define ADDR_TARGET_POSITION (0X3002 + JOINT_ID_OFFSET)
#define ADDR_TARGET_VELOCITY (0X3003 + JOINT_ID_OFFSET)
#define ADDR_VELOCITY_OFFSET (0X3004 + JOINT_ID_OFFSET)
#define ADDR_TARGET_CURRENT_Q (0X3005 + JOINT_ID_OFFSET)
#define ADDR_TORQUE_OFFSET (0X3006 + JOINT_ID_OFFSET)

#define ADDR_MODE_OF_OPERATION_DISPLAY (0X3081 + JOINT_ID_OFFSET)
#define ADDR_ACTUAL_POSITION (0X3082 + JOINT_ID_OFFSET)
#define ADDR_FOLLOWING_ERROR (0X3084 + JOINT_ID_OFFSET)
#define ADDR_ACTUAL_VELOCITY (0X3085 + JOINT_ID_OFFSET)
#define ADDR_ACTUAL_CURRENT_Q (0X330E + JOINT_ID_OFFSET)
#define ADDR_ACTUAL_CURRENT_D (0X330D + JOINT_ID_OFFSET)







    
    
    
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SENS_DIR_Pin GPIO_PIN_4
#define SENS_DIR_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_15
#define DRV_EN_GPIO_Port GPIOB
#define SENS_OUT_Pin GPIO_PIN_5
#define SENS_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
