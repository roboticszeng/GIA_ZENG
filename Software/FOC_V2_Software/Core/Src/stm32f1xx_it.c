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

#include "controll.h"
#include "as5600.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

extern CAN_RxHeaderTypeDef Can_Rx;
extern uint8_t Rxdata[8] ;
extern float rxdata;
uint8_t can_rx_finish_flag;//接收完成标志位

extern pid_typedef* oPidVelocity;
extern encoder_typedef* oEncoder;
extern sdo_typedef* oConfig;
extern pdo_typedef* oPdo;
extern filter_typedef* oFilterVelocity;

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
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles USB high priority or CAN TX interrupts.
  */
void USB_HP_CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 0 */

  /* USER CODE END USB_HP_CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 1 */
    

  /* USER CODE END USB_HP_CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */
    
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
    // TIM1 -- 电流采样 频率 XHZ

//    static _iq Ia, Ib, I_alpha, I_beta, I_d, I_q;
    
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
    
    
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // 电流采样 实测时间42.5us
//    ADC_get_voltage();
//    
//    
//    Ia = _IQmpy(_IQ(actualCurA), _IQ(16e-4)) - _IQ(3.3);
//    Ib = _IQmpy(_IQ(actualCurB), _IQ(16e-4)) - _IQ(3.3);
//    
//    I_alpha = Ia;
//    I_beta = _IQmpy(_IQ(_1_SQRT3), Ia) + _IQmpy(_IQ(_2_SQRT3), Ib);
//    I_d = _IQmpy(I_alpha, _IQcos(_IQ(angle_elec))) + _IQmpy(I_beta, _IQsin(_IQ(angle_elec)));
//    I_q = _IQmpy(I_beta, _IQcos(_IQ(angle_elec))) - _IQmpy(I_alpha, _IQsin(_IQ(angle_elec)));
    
    
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
//    static uint16_t lastPos;
    // TIM2 编码器计数 频率 XHZ

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    
//     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // 获取角度原始信息
//    lastPos = actualPos;
    as5600_get_angle(oEncoder, oPdo);
    
//    can_send_pdo();
//    get_angle_elec();
//    actualVel = (actualPos - lastPos) * 1000;
    
//     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
//    static float a;
    // TIM3 电流环控制 频率 XHZ
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
     
    
//    PIDController_Update(&PID_Current_Q, 0.2, tempb / 24);
//    setPhaseVoltage(PID_Current_Q.out, 0, angle_elec);
//    setPhaseVoltage(0.2, 0, angle_elec);
    
//    targetQ = PIDController_Update(&PID_Velocity, 2000, actualVel);
//    setPhaseVoltage(targetQ, 0, angle_elec);
    
    
//     a = a + 0.002;
//     setPhaseVoltage(0.1, 0, a);
    
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    
//    

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    // TIM4 速度位置环控制 频率 XHZ
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
//    targetQ = PIDController_Update(&PID_Velocity, 2000, actualVel);
//    targetQ = PIDController_Update(&PID_Velocity, 2000, actualVel);
    
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan1)
{
	HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&Can_Rx,Rxdata);
    can_rx_finish_flag=1;
    
    if (_check_data(Rxdata[0], DATA_HEAD) == 0){
        return;
    }
    
    switch (Can_Rx.ExtId){
        
        case ADDR_TARGET_POSITION:
            oPdo->target_position = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
        break;
        
        case ADDR_TARGET_VELOCITY:
            oPdo->target_velocity = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
        break;
        
        case ADDR_TARGET_CURRENT_Q:
            oPdo->target_current_q = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
        
        case ADDR_MODE_OF_OPERATION:
            oPdo->mode = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
        break;
        
    }
    
    
    
    
//     printf("RX ID:0x%X\r\n",Can_Rx.ExtId);
//	 printf("RX DATA: %02X%02X%02X%02X%02X%02X%02X%02X\r\n",Rxdata[0],Rxdata[1],Rxdata[2],Rxdata[3],Rxdata[4],Rxdata[5],Rxdata[6],Rxdata[7]);
}
/* USER CODE END 1 */
