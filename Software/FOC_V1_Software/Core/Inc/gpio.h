/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "controll.h"
#include "flash.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define BLINK_LOW 500
#define BLINK_MID 200
#define BLINK_FAST 100
    
#define KEY_HOLD 100  // 长按的时间长度（单位10mS）
#define KEY_DOUBLE 10 // 双击的时间长度（单位20mS）
#define KEY_PRESS 0
    
    
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void led_blink(uint16_t blink_speed, uint16_t blink_time);
void key_read(void);
void key_read_click(void);
void key_read_double_click(void);
void key_read_hold(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

