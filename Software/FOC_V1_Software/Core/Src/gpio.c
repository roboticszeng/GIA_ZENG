/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENS_DIR_GPIO_Port, SENS_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SENS_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENS_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DRV_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SENS_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENS_OUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 2 */

void led_blink(uint16_t blink_speed, uint16_t blink_time)
{
  for (unsigned int i = 0; i < blink_time * 2; i++)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(blink_speed);
  }
}



void key_read(void)
{
  static uint8_t hold_time, double_time, click_time;

  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == KEY_PRESS)
  { // 再次判断

    while ((HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == KEY_PRESS) && (hold_time < KEY_HOLD))
    { // 循环判断长按，到时跳转

      hold_time++;
      HAL_Delay(10); // 长按判断的计时
    }

    if (hold_time >= KEY_HOLD)
    { // 长键处理
      /* 长按后执行的程序放到此处 */

      key_read_hold();

      /* 长按后执行的程序放到此处 */
      while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == KEY_PRESS)
        ;
    }
    else
    { // 单击处理

      for (double_time = 0; double_time < KEY_DOUBLE; double_time++)
      { // 检测双击
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == KEY_PRESS)
        {
          click_time = 1;
          /* 双击后执行的程序放到此处 */

          key_read_double_click();

          /* 双击后执行的程序放到此处 */
          while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == KEY_PRESS);
        }
      }
      if (click_time == 0)
      { // 判断单击
        /* 单击后执行的程序放到此处 */
        
        key_read_click();

        /* 单击后执行的程序放到此处 */
      }
    }
    click_time = 0;
    hold_time = 0; // 参数清0
  }

} // 按键判断在此结束

void key_read_click(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

extern sdo_typedef* oSdo;
extern state_typedef* oState;
extern pdo_typedef* oPdo;

void key_read_double_click(void)
{
  for (unsigned int i = 0; i < 10; i++)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(100);
  }

  // 手动电角度对正
  calib_elec_angle(oState, oSdo);

  // 手动ADC对正
  calib_adc_offset(oPdo, oSdo);
  
  // 保存配置
  save_config(oSdo);
}
void key_read_hold(void)
{
  for (unsigned int i = 0; i < 10; i++)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
  }
  save_config(oSdo);
}


/* USER CODE END 2 */
