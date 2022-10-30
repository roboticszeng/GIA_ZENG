/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_as5600.h"
#include "FOC_kernal_3.h"
#include "PID.h"
#include "adc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

DQCurrent_s Current;
PhaseCurrent_s PhaCurrent;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POLEPAIRS 14
#define AD1_OFFSET 116
#define AD2_OFFSET 123


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
float angle;
float angle_el = 0.0;
float angle_prev;
//extern float Ts;
float vel;

float y_current_q_prev;
float y_current_d_prev;
float y_vel_prev;

int AD_Value_0, AD_Value_1;
float I_alpha, I_beta;

#define Rcs 0.01														//采样电阻阻值
#define Gain 50	


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
float currentQ_sp = 0.0;
float currentD_sp = 0.0;
float vel_sp;
float ang_sp;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
    // 采样定时器 频率1kHz
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    
    
    angle = bsp_as5600GetAngle();
    angle_el = angle * POLEPAIRS;
    
    
    AD_Value_0 = HAL_ADC_GetValue(&hadc1);
    AD_Value_1 = HAL_ADC_GetValue(&hadc2);
    AD_Value_0 += AD1_OFFSET;
    AD_Value_1 += AD1_OFFSET;

    PhaCurrent.a=((3.3*((float)AD_Value_0/4096))-1.65)/Rcs/Gain;      //相电流物理值=（采样电压-偏置）/Rcs/增益  ;  单位：A
    PhaCurrent.b=((3.3*((float)AD_Value_1/4096))-1.65)/Rcs/Gain;     
    
    I_alpha = PhaCurrent.a;
    I_beta = _1_SQRT3 * PhaCurrent.a + _2_SQRT3 * PhaCurrent.b;
    angle_el=_normalizeAngle(angle_el);
    Current.d= I_alpha *_cos(angle_el) + I_beta *_sin(angle_el);
    Current.q= I_beta *_cos(angle_el) - I_alpha *_sin(angle_el);
    
    Current.d = LPF_current_d(Current.d);
    Current.q = LPF_current_q(Current.q);
    y_current_q_prev = Current.q;
    y_current_d_prev = Current.d;

    vel = (angle - angle_prev) / 1e-3;
    vel = LPF_velocity(vel);
    

    angle_prev = angle;
    
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    // 速度环、位置环定时器（暂时也有电流环）频率500Hz
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
    
    //期望频率500Hz

    setPhaseVoltage(PID_current_Q(currentQ_sp - Current.q), -PID_current_D(currentD_sp - Current.d), angle_el);

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    static float t = 0;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
    
    // 频率250Hz
    
    ang_sp = 3.14 / 3 * _sin(t * 6.28) + 5;
    t = t + 0.004;
    if(t > 1){
        t = 0;
    }
    
    
    
    vel_sp = PID_angle(ang_sp - angle);
    currentQ_sp = -PID_velocity(vel_sp - vel);
//    currentQ_sp = 0.5;
    
//    currentD_sp = 0.5 * _sin(t * 6.28 * 10);
//    currentD_sp = 0.0;
//    t = t + 0.004;
//    
//    if(t > 0.1){
//        t = 0;
//    }
    
    currentD_sp = 0;
    
    
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */


float LPF_current_q(float x)
{
	float y = 0.9*y_current_q_prev + 0.1*x;
	
	y_current_q_prev=y;
	
	return y;
}
/******************************************************************************/
float LPF_current_d(float x)
{
	float y = 0.9*y_current_d_prev + 0.1*x;
	
	y_current_d_prev=y;
	
	return y;
}
/******************************************************************************/
/***********************************反馈转速低通滤波*******************************************/
float LPF_velocity(float x)
{
	float y = 0.8*y_vel_prev + 0.2*x;
	
	y_vel_prev=y;
	
	return y;
}
/* USER CODE END 1 */
