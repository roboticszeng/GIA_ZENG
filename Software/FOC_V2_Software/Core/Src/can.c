/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "as5600.h"
#include "controll.h"
extern CAN_TxHeaderTypeDef Can_Tx;
extern uint8_t Txdata[8];

extern pid_typedef* oPidVelocity;
extern encoder_typedef* oEncoder;
extern sdo_typedef* oConfig;
extern pdo_typedef* oPdo;
extern filter_typedef* oFilterVelocity;

void CAN_User_Init(CAN_HandleTypeDef* hcan )   //用户初始化函数
{
    int mask;
  CAN_FilterTypeDef  sFilterConfig;
  HAL_StatusTypeDef  HAL_Status;
  sFilterConfig.FilterActivation = ENABLE;  	//激活过滤器
  sFilterConfig.FilterBank = 1;                       //过滤器1
  sFilterConfig.FilterMode =  CAN_FILTERMODE_IDMASK;  //设为掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;    //设为32位
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;    //接收到的报文放入到FIFO0中
    
  sFilterConfig.FilterIdHigh = ((JOINT_ID + 3) & 0x06) >> 1;   // 0x01
  sFilterConfig.FilterIdLow  = ((JOINT_ID + 3) & 0x01) << 15; // 0x8000
    
  // 固定的
  sFilterConfig.FilterMaskIdHigh = 0x03;
  sFilterConfig.FilterMaskIdLow  = 0x8000;
    
  sFilterConfig.SlaveStartFilterBank  = 0;
 
  HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
  HAL_Status=HAL_CAN_Start(hcan);  //开启CAN
 
//  if(HAL_Status!=HAL_OK){
//	printf("开启CAN失败\r\n");
// }
 HAL_Status=HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
// if(HAL_Status!=HAL_OK){
//	printf("开启挂起中段允许失败\r\n");
//  }
}
 
/*
 发送命令函数
 StdId 			标准帧ID
 ExtId 			扩展帧ID  当标志位 IDE为CAN_ID_STD时 扩展帧无效
 IDE 			扩展帧标志位  CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
 RTR  			0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
 DLC  			数据长度
*/
void sendOrder(uint32_t StdId,uint32_t ExtId,uint8_t IDE,uint8_t  RTR, uint8_t DLC)
{
	uint32_t pTxMailbox = 0;
 
	Can_Tx.StdId = StdId;//标准ID
	Can_Tx.ExtId = ExtId;//扩展ID
	Can_Tx.IDE = IDE;//CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
	Can_Tx.RTR = RTR;					//0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
	Can_Tx.DLC = DLC;					//数据长度
//	printf("TX ID:0x%X\r\n",ExtId);
//	printf("TX DATA:%02X%02X%02X%02X%02X%02X%02X%02X\r\n",Txdata[0],Txdata[1],Txdata[2],Txdata[3],Txdata[4],Txdata[5],Txdata[6],Txdata[7]);
	HAL_CAN_AddTxMessage(&hcan,&Can_Tx,Txdata,&pTxMailbox);
 
}

extern pdo_typedef* oPdo;
void can_send_pdo(void)
{

    convert_pdo_to_tx(ADDR_MODE_OF_OPERATION, oPdo->mode);
    convert_pdo_to_tx(ADDR_TARGET_POSITION, oPdo->target_position);
    convert_pdo_to_tx(ADDR_TARGET_VELOCITY, oPdo->target_velocity);
    convert_pdo_to_tx(ADDR_TARGET_CURRENT_Q, oPdo->target_current_q);
    
    convert_pdo_to_tx(ADDR_MODE_OF_OPERATION_DISPLAY, oPdo->mode);
    convert_pdo_to_tx(ADDR_ACTUAL_POSITION, oPdo->actual_position);
    convert_pdo_to_tx(ADDR_FOLLOWING_ERROR, oPdo->following_error);
    convert_pdo_to_tx(ADDR_ACTUAL_VELOCITY, oPdo->actual_velocity);
    convert_pdo_to_tx(ADDR_ACTUAL_CURRENT_Q, oPdo->actual_current_q);
    convert_pdo_to_tx(ADDR_ACTUAL_CURRENT_D, oPdo->actual_current_d);
    

}

void convert_pdo_to_tx(uint32_t ext_id, uint32_t data){
    CAN_TxHeaderTypeDef can_tx_local;
    static uint32_t pTxMailbox = 0;
    static uint8_t can_tx_data[8];
	uint8_t i;
	can_tx_local.StdId = JOINT_ID;//标准ID
	
	can_tx_local.IDE = CAN_ID_EXT;//CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
	can_tx_local.RTR = 0;					//0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
	can_tx_local.DLC = 8;					//数据长度
    
    can_tx_data[0] = DATA_HEAD;
    can_tx_data[1] = JOINT_ID;
    can_tx_data[6] = DATA_TAIL_PREV;
    can_tx_data[7] = DATA_TAIL;
    
    can_tx_local.ExtId = ext_id;
    can_tx_data[2] = _get_high_byte_uint(data);
    can_tx_data[3] = _get_low_byte_uint(data);
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)==0);
	HAL_CAN_AddTxMessage(&hcan, &can_tx_local, can_tx_data, &pTxMailbox);
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
