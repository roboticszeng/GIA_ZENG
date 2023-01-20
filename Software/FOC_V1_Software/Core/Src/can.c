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

extern pdo_typedef *oPdo;
extern sdo_typedef *oSdo;
extern pid_typedef *oPidVelocity;
extern encoder_typedef *oEncoder;
extern config_typedef *oConfig;
extern state_typedef *oState;
extern filter_typedef *oFilterVelocity;

void CAN_User_Init(CAN_HandleTypeDef *hcan) // 用户初始化函数
{
  int mask;
  CAN_FilterTypeDef sFilterConfig;
  HAL_StatusTypeDef HAL_Status;
  sFilterConfig.FilterActivation = ENABLE;               // 激活过滤器
  sFilterConfig.FilterBank = 1;                          // 过滤器1
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;      // 设为掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;     // 设为32位
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 接收到的报文放入到FIFO0中

  sFilterConfig.FilterIdHigh = ((oConfig->ID + 3) & 0x06) >> 1; // 0x01
  sFilterConfig.FilterIdLow = ((oConfig->ID + 3) & 0x01) << 15; // 0x8000

  // 固定的
  sFilterConfig.FilterMaskIdHigh = 0x03;
  sFilterConfig.FilterMaskIdLow = 0x8000;

  sFilterConfig.SlaveStartFilterBank = 0;

  HAL_Status = HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
  HAL_Status = HAL_CAN_Start(hcan); // 开启CAN

  //  if(HAL_Status!=HAL_OK){
  //	printf("开启CAN失败\r\n");
  // }
  HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
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
void sendOrder(uint32_t StdId, uint32_t ExtId, uint8_t IDE, uint8_t RTR, uint8_t DLC)
{
  uint32_t pTxMailbox = 0;

  Can_Tx.StdId = StdId; // 标准ID
  Can_Tx.ExtId = ExtId; // 扩展ID
  Can_Tx.IDE = IDE;     // CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
  Can_Tx.RTR = RTR;     // 0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
  Can_Tx.DLC = DLC;     // 数据长度
  HAL_CAN_AddTxMessage(&hcan, &Can_Tx, Txdata, &pTxMailbox);
}

void can_send_pdo(void)
{

  oPdo->PDO_ACTUAL_POSITION = convert_position_to_pulse(oState->iqPos);
  oPdo->PDO_ACTUAL_VELOCITY = convert_velocity_to_pulse(oState->iqVel);
  oPdo->PDO_ACTUAL_CURRENT_Q = convert_current_to_pulse(oState->iqCurQ);
  oPdo->PDO_ACTUAL_CURRENT_D = convert_current_to_pulse(oState->iqCurD);

  CAN_TxHeaderTypeDef can_tx_local;
  static uint32_t pTxMailbox = 0;
  static uint8_t can_tx_data[8];
  uint8_t i;
  can_tx_local.StdId = oConfig->ID; // 标准ID

  can_tx_local.IDE = CAN_ID_EXT; // CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
  can_tx_local.RTR = 0;          // 0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
  can_tx_local.DLC = 8;          // 数据长度

  //    can_tx_data[0] = DATA_READ_HEAD;
  //    can_tx_data[1] = oConfig->ID;
  //    can_tx_data[6] = DATA_TAIL_PREV;
  //    can_tx_data[7] = DATA_TAIL;
  //    can_tx_data[2] = _get_high_byte_uint(data);
  //    can_tx_data[3] = _get_low_byte_uint(data);
  can_tx_local.ExtId = _get_addr_joint(ADDR_PDO_SEND);

  can_tx_data[0] = _get_high_byte_uint(oPdo->PDO_MODE_OF_OPERATION);
  can_tx_data[1] = _get_low_byte_uint(oPdo->PDO_MODE_OF_OPERATION);
  can_tx_data[2] = _get_high_byte_uint(oPdo->PDO_ACTUAL_POSITION);
  can_tx_data[3] = _get_low_byte_uint(oPdo->PDO_ACTUAL_POSITION);
  can_tx_data[4] = _get_high_byte_uint(oPdo->PDO_ACTUAL_VELOCITY);
  can_tx_data[5] = _get_low_byte_uint(oPdo->PDO_ACTUAL_VELOCITY);
  can_tx_data[6] = _get_high_byte_uint(oPdo->PDO_ACTUAL_CURRENT_Q);
  can_tx_data[7] = _get_low_byte_uint(oPdo->PDO_ACTUAL_CURRENT_Q);

  //    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)==0);
  HAL_CAN_AddTxMessage(&hcan, &can_tx_local, can_tx_data, &pTxMailbox);

  //    convert_data_to_tx(ADDR_MODE_OF_OPERATION, oPdo->PDO_MODE_OF_OPERATION);
  //    convert_data_to_tx(ADDR_TARGET_POSITION, oPdo->PDO_TARGET_POSITION);
  //    convert_data_to_tx(ADDR_TARGET_VELOCITY, oPdo->PDO_TARGET_VELOCITY);
  //    convert_data_to_tx(ADDR_TARGET_CURRENT_Q, oPdo->PDO_TARGET_CURRENT_Q);
  //    convert_data_to_tx(_get_addr_joint(ADDR_ACTUAL_POSITION), oPdo->PDO_ACTUAL_POSITION);
  //    convert_data_to_tx(_get_addr_joint(ADDR_ACTUAL_VELOCITY), oPdo->PDO_ACTUAL_VELOCITY);
  //    convert_data_to_tx(_get_addr_joint(ADDR_ACTUAL_CURRENT_Q), oPdo->PDO_ACTUAL_CURRENT_Q);
  //    convert_data_to_tx(ADDR_ACTUAL_CURRENT_D, oPdo->PDO_ACTUAL_CURRENT_D);
}

void convert_data_to_tx(uint32_t ext_id, uint32_t data)
{
  CAN_TxHeaderTypeDef can_tx_local;
  static uint32_t pTxMailbox = 0;
  static uint8_t can_tx_data[8];
  uint8_t i;
  can_tx_local.StdId = oConfig->ID; // 标准ID

  can_tx_local.IDE = CAN_ID_EXT; // CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
  can_tx_local.RTR = 0;          // 0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
  can_tx_local.DLC = 8;          // 数据长度

  can_tx_data[0] = DATA_READ_HEAD;
  can_tx_data[1] = oConfig->ID;
  can_tx_data[6] = DATA_TAIL_PREV;
  can_tx_data[7] = DATA_TAIL;

  can_tx_local.ExtId = ext_id;
  can_tx_data[2] = _get_high_byte_uint(data);
  can_tx_data[3] = _get_low_byte_uint(data);
  //    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)==0);
  HAL_CAN_AddTxMessage(&hcan, &can_tx_local, can_tx_data, &pTxMailbox);
}

extern pdo_typedef *oPdo;
extern sdo_typedef *oSdo;
extern uint16_t ADDR_ARRAY[];

uint16_t get_can_value(uint32_t addr)
{
  uint16_t value;

  /* PDO 暂时不变 */
  switch (addr)
  {
  case ADDR_MODE_OF_OPERATION:
    value = oPdo->PDO_MODE_OF_OPERATION;
    break;
  case ADDR_TARGET_POSITION:
    value = oPdo->PDO_TARGET_POSITION;
    break;
  case ADDR_TARGET_VELOCITY:
    value = oPdo->PDO_TARGET_VELOCITY;
    break;
  case ADDR_VELOCITY_OFFSET:
    value = oPdo->PDO_VELOCITY_OFFSET;
    break;
  case ADDR_TARGET_CURRENT_Q:
    value = oPdo->PDO_TARGET_CURRENT_Q;
    break;
  case ADDR_TORQUE_OFFSET:
    value = oPdo->PDO_TORQUE_OFFSET;
    break;
  case ADDR_MODE_OF_OPERATION_DISPLAY:
    value = oPdo->PDO_MODE_OF_OPERATION_DISPLAY;
    break;
  case ADDR_ACTUAL_POSITION:
    value = oPdo->PDO_ACTUAL_POSITION;
    break;
  case ADDR_FOLLOWING_ERROR:
    value = oPdo->PDO_FOLLOWING_ERROR;
    break;
  case ADDR_ACTUAL_VELOCITY:
    value = oPdo->PDO_ACTUAL_VELOCITY;
    break;
  case ADDR_ACTUAL_CURRENT_Q:
    value = oPdo->PDO_ACTUAL_CURRENT_Q;
    break;
  case ADDR_ACTUAL_CURRENT_D:
    value = oPdo->PDO_ACTUAL_CURRENT_D;
    break;
  }

  /* sdo 遍历 */
  uint16_t *point_sdo = (uint16_t *)oSdo;
  for(int i = 0; i < sizeof(*oSdo) / sizeof(uint16_t); i++)
  {
    if(addr == ADDR_ARRAY[i])
    {
      value = *(point_sdo);
      break;
    }
    // 这里是不是可以不要*
    point_sdo++;
  }

  return value;
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

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
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

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
