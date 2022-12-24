/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <main.h>
     
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

uint16_t CONST_JOINT_NUMBER = 0xd3;
uint16_t CONST_POLAR_PAIRS = 14;
uint16_t CONST_SUPPLY_VOLTAGE = 24;
uint16_t CONST_ENC_RESOLUTION = 4096;
uint16_t CONST_ENC_OFFSET = 0;
uint16_t CONST_ELEC_ANG_OFFSET = 0;
uint16_t CONST_ADC_RESOLUTION = 4096;
uint16_t CONST_ADC_OFFSET_0 = 0;
uint16_t CONST_ADC_OFFSET_1 = 0;
uint16_t CONST_DIRECT_CUR_POS = 1;
uint16_t CONST_SAMPLE_RESISTANCE = 10;
uint16_t CONST_SAMPLE_GAIN = 50;
uint16_t CONST_MCU_VOLTAGE = 3300;
uint16_t CONST_POS_HIGHER_BOUND = 4096;
uint16_t CONST_POS_LOWER_BOUND = 0;
uint16_t CONST_VEL_HIGHER_BOUND = 40960;
uint16_t CONST_VEL_LOWER_BOUND = 0;
uint16_t CONST_CUR_Q_HIGHER_BOUND = 4096;
uint16_t CONST_CUR_Q_LOWER_BOUND = 0;
uint16_t CONST_CUR_D_HIGHER_BOUND = 4096;
uint16_t CONST_CUR_D_LOWER_BOUND = 0;
     
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void USB_HP_CAN1_TX_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
/* USER CODE BEGIN EFP */

//void ADC_Select_Channel(uint32_t ch);
//void ADC_get_voltage(void);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
