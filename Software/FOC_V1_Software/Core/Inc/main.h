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

#define LED_BLINK_READ_FLASH 10
#define LED_BLINK_SAVE_FLASH 4
#define LED_BLINK_CALIB 2

#define PWM_PERIOD (1500-1)


#define DATA_WRITE_HEAD 0x55
#define DATA_READ_HEAD 0x56
#define DATA_TAIL_PREV 0x0d
#define DATA_TAIL 0x0a
    
#define DATA_READ_FLAG 0xaaaa
#define DATA_ERROR 0xffff
    
#define MODE_CSP 0x08
#define MODE_CSV 0x09
#define MODE_CST 0x0a
#define MODE_NO  0x00
    
#define PID_INDEX_POS 0x08
#define PID_INDEX_VEL 0x09
#define PID_INDEX_CUR_Q 0x0a
#define PID_INDEX_CUR_D 0x0b

#define FILTER_INDEX_POS 0x08
#define FILTER_INDEX_VEL 0x09
#define FILTER_INDEX_CUR_Q 0x0a
#define FILTER_INDEX_CUR_D 0x0b
    
/*****************************************************    PDO ADDR    *****************************************************/
    
#define ADDR_MODE_OF_OPERATION 0X3001
#define ADDR_TARGET_POSITION 0X3002
#define ADDR_TARGET_VELOCITY 0X3003
#define ADDR_VELOCITY_OFFSET 0X3004
#define ADDR_TARGET_CURRENT_Q 0X3005
#define ADDR_TORQUE_OFFSET 0X3006

#define ADDR_MODE_OF_OPERATION_DISPLAY 0X3081
#define ADDR_ACTUAL_POSITION 0X3082
#define ADDR_FOLLOWING_ERROR 0X3084
#define ADDR_ACTUAL_VELOCITY 0X3085
#define ADDR_ACTUAL_CURRENT_Q 0X308E
#define ADDR_ACTUAL_CURRENT_D 0X308D

/*****************************************************    PDO ADDR    *****************************************************/


/*****************************************************    SDO ADDR    *****************************************************/


#define ADDR_PDO_SEND 0x3080

#define ADDR_ID 0x3000
#define ADDR_ENC_ZERO_POSITION_H 0X3500
#define ADDR_ENC_ZERO_POSITION_L 0X3502

#define ADDR_PID_POS_KP_INT 0x3600
#define ADDR_PID_POS_KP_DEC 0x3602
#define ADDR_PID_POS_KI_INT 0x3604
#define ADDR_PID_POS_KI_DEC 0x3606
#define ADDR_PID_POS_MAX 0x3608
#define ADDR_PID_POS_INTMAX 0x360a

#define ADDR_PID_VEL_KP_INT 0x3610
#define ADDR_PID_VEL_KP_DEC 0x3612
#define ADDR_PID_VEL_KI_INT 0x3614
#define ADDR_PID_VEL_KI_DEC 0x3616
#define ADDR_PID_VEL_MAX 0x3618
#define ADDR_PID_VEL_INTMAX 0x361a

#define ADDR_PID_CUR_Q_KP_INT 0x3620
#define ADDR_PID_CUR_Q_KP_DEC 0x3622
#define ADDR_PID_CUR_Q_KI_INT 0x3624
#define ADDR_PID_CUR_Q_KI_DEC 0x3626
#define ADDR_PID_CUR_Q_MAX 0x3628
#define ADDR_PID_CUR_Q_INTMAX 0x362a

#define ADDR_PID_CUR_D_KP_INT 0x3630
#define ADDR_PID_CUR_D_KP_DEC 0x3632
#define ADDR_PID_CUR_D_KI_INT 0x3634
#define ADDR_PID_CUR_D_KI_DEC 0x3636
#define ADDR_PID_CUR_D_MAX 0x3638
#define ADDR_PID_CUR_D_INTMAX 0x363a

#define ADDR_FILT_VEL_CUTOFF_FREQ 0x3702
#define ADDR_FILT_CUR_Q_CUTOFF_FREQ 0x3704
#define ADDR_FILT_CUR_D_CUTOFF_FREQ 0x3706

#define ADDR_POSITION_LOWER_LIMIT 0x3800
#define ADDR_POSITION_UPPER_LIMIT 0x3802
#define ADDR_VELOCITY_LOWER_LIMIT 0x3804
#define ADDR_VELOCITY_UPPER_LIMIT 0x3806
#define ADDR_CURRENT_Q_LOWER_LIMIT 0x3808
#define ADDR_CURRENT_Q_UPPER_LIMIT 0x380a
#define ADDR_CURRENT_D_LOWER_LIMIT 0x380c
#define ADDR_CURRENT_D_UPPER_LIMIT 0x380e

#define ADDR_ELEC_ZERO_POSITION 0x3900

#define ADDR_ADC0_OFFSET 0x3a00
#define ADDR_ADC1_OFFSET 0x3a02

#define ADDR_READ_FLAG 0x3f00
#define ADDR_READ_FLASH 0x3ffc
#define ADDR_SAVE_FLASH 0x3ffe



/*****************************************************    SDO ADDR    *****************************************************/
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
#define KEY_Pin GPIO_PIN_5
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI9_5_IRQn
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
