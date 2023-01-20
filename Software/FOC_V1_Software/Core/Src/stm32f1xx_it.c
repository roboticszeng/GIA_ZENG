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
#include "adc.h"
#include "flash.h"
#include "tim.h"
#include "gpio.h"

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
extern uint8_t Rxdata[8];
extern float rxdata;
uint8_t can_rx_finish_flag; // 接收完成标志位

_iq const_point;

extern encoder_typedef *oEncoder;
extern config_typedef *oConfig;
extern state_typedef *oState;

extern pid_typedef *oPidPosition;
extern pid_typedef *oPidVelocity;
extern pid_typedef *oPidCurrentD;
extern pid_typedef *oPidCurrentQ;

extern filter_typedef *oFilterVelocity;
extern filter_typedef *oFilterCurrentD;
extern filter_typedef *oFilterCurrentQ;

extern pdo_typedef *oPdo;

extern uint16_t ADDR_ARRAY[];


// extern sdo_typedef* oSdo;

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
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt.
 */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
  /*! TIM1 -- 电流采样 频率 16kHZ 周期 62.5us !*/

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
  //    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  ADC_get_voltage(oState);
  //    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /*! TIM2 -- 编码器通信 频率 1kHZ 周期 1ms !*/
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  as5600_get_angle(oEncoder, oState);
  can_send_pdo();
  //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /*! TIM3 -- 电流环控制 频率 6.25kHZ 周期 160us !*/
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  //    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  static _iq a0, a1, b1, w;
  a0 = _IQ(0.1395);
  a1 = _IQ(-0.09069);
  b1 = _IQ(-1.089);
  w = _IQ(0.9858);
  

  if (oPdo->PDO_MODE_OF_OPERATION == MODE_CST)
  {
    // 电流环正弦力矩控制
//    static _iq t = 0.0;
//    t = t + _IQ(oConfig->CONST_CURRENT_CONTROL_TIME);
//    oState->iqTargQ = _IQsin(_IQmpy(t, _IQmpyI32(_IQ(_2PI), 1)));
      
    // 电流环恒定力矩控制
    //        oState->iqTargQ = _IQ(1.0);
    oState->iqTargQ = convert_pulse_to_current(oPdo->PDO_TARGET_CURRENT_Q);
    //        oState->iqTargQ = -_IQmpy(_IQ(0.85), _IQsin(oState->iqPos));
    //        oState->iqTargQ = _IQmpy(a1, _IQsin(_IQmpy(b1, oState->iqPos) + c1)) + \
//                    _IQmpy(a2, _IQsin(_IQmpy(b2, oState->iqPos) + c2));
    //        oState->iqTargQ = a0 + _IQmpy(a1, _IQcos(_IQmpy(w, oState->iqPos))) + _IQmpy(b1, _IQsin(_IQmpy(w, oState->iqPos)));
  }
  else if (oPdo->PDO_MODE_OF_OPERATION == MODE_NO)
  {
    // 0模式，无输出
    oState->iqTargQ = _IQ(0.0);
  }
  oState->iqTargD = _IQ(0.0);
  // 电流环PID
  oState->iqVoltQ = pid_update(oPidCurrentQ, oState->iqTargQ, oState->iqCurQ);
  oState->iqVoltD = pid_update(oPidCurrentD, oState->iqTargD, oState->iqCurD);
  // 生成svpwm
  compute_svpwm(oState->iqVoltQ, oState->iqVoltD, oState->iqPosElec);
  // SVPWM功能测试
  //    static float a;
  //    a += 0.02;
  //    compute_svpwm(_IQ(0.2), _IQ(0.0), _IQ(a));
  //    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /*! TIM4 -- 速度位置环控制 频率 3.125kHZ 周期 320us !*/
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  //    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  if (oPdo->PDO_MODE_OF_OPERATION == MODE_CSP)
  {
    /* 位置环 */
    // 位置环恒定位置控制
    // oState->iqTargP = const_point;
    // 位置环正弦位置控制
    //    static _iq t = 0.0;
    //    t = t + oConfig->CONST_POSITION_CONTROL_TIME;
    //    oState->iqTargP = _IQmpy(_IQ(1.0), _IQsin(_IQmpy(t, _IQmpy(_IQ(_2PI), _IQ(1.0))))) + _IQ(4.7);
    oState->iqTargP = convert_pulse_to_position(oPdo->PDO_TARGET_POSITION);
    // 位置环PID
    oState->iqTargV = pid_update(oPidPosition, oState->iqTargP, oState->iqPos);
    oState->iqTargQ = pid_update(oPidVelocity, oState->iqTargV, oState->iqVel);
  }
  else if (oPdo->PDO_MODE_OF_OPERATION == MODE_CSV)
  {
    /* 速度环 */
    // 速度环恒定速度控制
    //        oState->iqTargV = _IQ(5.0);
    // 速度环正弦速度控制
    //    static _iq t = 0.0;
    //    t = t + oConfig->CONST_POSITION_CONTROL_TIME;
    //    oState->iqTargV = _IQmpy(_IQ(10.0), _IQsin(_IQmpy(t, _IQmpyI32(_IQ(_2PI), 2))));
    // 涉及正负，需要-32768??没想好
    oState->iqTargV = convert_pulse_to_velocity(oPdo->PDO_TARGET_VELOCITY);
    oState->iqTargQ = pid_update(oPidVelocity, oState->iqTargV, oState->iqVel);
  }
  // 速度环PID
  //    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

extern sdo_typedef *oSdo;
extern pid_typedef *oPidPosition;
extern pid_typedef *oPidVelocity;
extern pid_typedef *oPidCurrentD;
extern pid_typedef *oPidCurrentQ;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  int temp;
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &Can_Rx, Rxdata);
  can_rx_finish_flag = 1;

  if ((_check_data(Rxdata[0], DATA_WRITE_HEAD) == 0 && _check_data(Rxdata[0], DATA_READ_HEAD) == 0) || _check_data(Rxdata[1], oConfig->ID) == 0)
  {
    // check: 1.data头是读取头OR写入头，2.id，二者缺一则判断为数据错误
    return;
  }
  temp = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
  // 读数据
  if (_check_data(Rxdata[0], DATA_READ_HEAD) == 1)
  {
    convert_data_to_tx(Can_Rx.ExtId, get_can_value(_get_addr_std(Can_Rx.ExtId)));
    return;
  }
  switch (_get_addr_std(Can_Rx.ExtId))
  {
  /* PDO 暂时保留这种遍历的方式*/
  case ADDR_TARGET_POSITION:
    if (_check_range(temp, oSdo->SDO_POSITION_LOWER_LIMIT, oSdo->SDO_POSITION_UPPER_LIMIT) == 1)
    {
      oPdo->PDO_TARGET_POSITION = temp;
    }
    break;

  case ADDR_TARGET_VELOCITY:
    if (_check_range(temp, oSdo->SDO_VELOCITY_LOWER_LIMIT, oSdo->SDO_VELOCITY_UPPER_LIMIT) == 1)
    {
      oPdo->PDO_TARGET_VELOCITY = temp;
    }
    break;

  case ADDR_TARGET_CURRENT_Q:
    if (_check_range(temp, oSdo->SDO_CURRENT_Q_LOWER_LIMIT, oSdo->SDO_CURRENT_Q_UPPER_LIMIT) == 1)
    {
      oPdo->PDO_TARGET_CURRENT_Q = temp;
    }
    break;

  case ADDR_MODE_OF_OPERATION:
    oPdo->PDO_MODE_OF_OPERATION = temp;
    break;

  /* save */
  case ADDR_SAVE_FLASH:

    save_config(oSdo);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    break;
  }
  // sdo写入判断
  uint16_t *point_sdo = (uint16_t *)oSdo;
  for(unsigned int i = 0; i < sizeof(*oSdo) / sizeof(uint16_t); i++)
  //  先check这里的长度对不对
  {
    if(_get_addr_std(Can_Rx.ExtId) == ADDR_ARRAY[i])
    {
      // 地址匹配时 sdo的当前成员赋值rx
      *(point_sdo) = _convert_8bit_to_16bit(Rxdata[2], Rxdata[3]);
        // init all para
        para_init(oSdo, oConfig);
      if(_get_addr_std(Can_Rx.ExtId) == ADDR_ID)
      {
        // can id写入还需要重新init can
        CAN_User_Init(&hcan);
      }
      // 这里的break可否跳出？查C语言规范
      // break;
    }
    // 不管地址判断是否正确，指针都向后移一位
    point_sdo++;
  }
  // 写入成功后读配置来确认
  convert_data_to_tx(Can_Rx.ExtId, get_can_value(_get_addr_std(Can_Rx.ExtId)));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    HAL_Delay(10);
    key_read();
  }
}





/* USER CODE END 1 */
