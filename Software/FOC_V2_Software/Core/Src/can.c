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

int std_id = 0x7e9;
int ext_id = 0x3507;

int joint_id = 2;

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
    
  sFilterConfig.FilterIdHigh = ((joint_id + 2) & 0x06) >> 1;   // 0x01
  sFilterConfig.FilterIdLow  = ((joint_id + 2) & 0x01) << 15; // 0x8000
    
  // 固定的
  sFilterConfig.FilterMaskIdHigh = 0x03;
  sFilterConfig.FilterMaskIdLow  = 0x8000;
    

    
  sFilterConfig.SlaveStartFilterBank  = 0;
 
  HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
  HAL_Status=HAL_CAN_Start(hcan);  //开启CAN
 
  if(HAL_Status!=HAL_OK){
//	printf("开启CAN失败\r\n");
 }
 HAL_Status=HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
 if(HAL_Status!=HAL_OK){
	//printf("开启挂起中段允许失败\r\n");
  }
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
void sendmessage(uint32_t StdId,uint32_t ExtId,uint8_t IDE,uint8_t  RTR, uint8_t DLC,float send_data)
{
	uint32_t pTxMailbox = 0;
	uint8_t i;
	Can_Tx.StdId = StdId;//标准ID
	Can_Tx.ExtId = ExtId;//扩展ID
	Can_Tx.IDE = IDE;//CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
	Can_Tx.RTR = RTR;					//0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
	Can_Tx.DLC = DLC;					//数据长度
	//将浮点数转化成4个字节存在tdata[4]----tdata[7]中
	send_data=send_data*100;
//		Txdata[4] = (int)send_data&0x00ff;
//		Txdata[3] = (int)send_data>>8;
//		Txdata[1] = 0x01;
    Txdata[0] = _IQint(oPdo->angle_elec);
//		printf("TX ID:0x%X\r\n",Can_Tx.ExtId);
//	printf("TX DATA:%02X%02X%02X%02X%02X%02X%02X%02X\r\n",Txdata[0],Txdata[1],Txdata[2],Txdata[3],Txdata[4],Txdata[5],Txdata[6],Txdata[7]);
	HAL_CAN_AddTxMessage(&hcan,&Can_Tx,Txdata,&pTxMailbox);
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
