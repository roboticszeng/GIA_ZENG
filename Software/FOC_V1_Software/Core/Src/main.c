/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
  return ch;
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

#include "controll.h"
#include "AS5600.h"
#include "can.h"
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t ADDR_ARRAY[] = {ADDR_ID,
                         ADDR_ENC_ZERO_POSITION_H, ADDR_ENC_ZERO_POSITION_L, 
                         ADDR_PID_POS_KP_INT, ADDR_PID_POS_KP_DEC, ADDR_PID_POS_KI_INT, ADDR_PID_POS_KI_DEC, ADDR_PID_POS_MAX, ADDR_PID_POS_INTMAX,
                         ADDR_PID_VEL_KP_INT, ADDR_PID_VEL_KP_DEC, ADDR_PID_VEL_KI_INT, ADDR_PID_VEL_KI_DEC, ADDR_PID_VEL_MAX, ADDR_PID_VEL_INTMAX,
                         ADDR_PID_CUR_Q_KP_INT, ADDR_PID_CUR_Q_KP_DEC, ADDR_PID_CUR_Q_KI_INT, ADDR_PID_CUR_Q_KI_DEC, ADDR_PID_CUR_Q_MAX, ADDR_PID_CUR_Q_INTMAX,
                         ADDR_PID_CUR_D_KP_INT, ADDR_PID_CUR_D_KP_DEC, ADDR_PID_CUR_D_KI_INT, ADDR_PID_CUR_D_KI_DEC, ADDR_PID_CUR_D_MAX, ADDR_PID_CUR_D_INTMAX,
                         ADDR_FILT_VEL_CUTOFF_FREQ, ADDR_FILT_CUR_Q_CUTOFF_FREQ, ADDR_FILT_CUR_D_CUTOFF_FREQ,
                         ADDR_POSITION_LOWER_LIMIT, ADDR_POSITION_UPPER_LIMIT, ADDR_VELOCITY_LOWER_LIMIT, ADDR_VELOCITY_UPPER_LIMIT,
                         ADDR_CURRENT_Q_LOWER_LIMIT, ADDR_CURRENT_Q_UPPER_LIMIT, ADDR_CURRENT_D_LOWER_LIMIT, ADDR_CURRENT_D_UPPER_LIMIT,
                         ADDR_ELEC_ZERO_POSITION,
                         ADDR_ADC0_OFFSET, ADDR_ADC1_OFFSET};

// can test
CAN_TxHeaderTypeDef Can_Tx;
CAN_RxHeaderTypeDef Can_Rx;
uint8_t Rxdata[8];
uint8_t Txdata[8] = {0};
extern uint8_t can_rx_finish_flag;

extern _iq const_point;
extern uint16_t adc_dma_buf[NUMBER_ADC_CHANNEL * NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL];

encoder_typedef *oEncoder;
config_typedef *oConfig;
state_typedef *oState;

pid_typedef *oPidPosition;
pid_typedef *oPidVelocity;
pid_typedef *oPidCurrentD;
pid_typedef *oPidCurrentQ;

filter_typedef *oFilterVelocity;
filter_typedef *oFilterCurrentD;
filter_typedef *oFilterCurrentQ;

sdo_typedef *oSdo;
pdo_typedef *oPdo;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SENS_DIR_GPIO_Port, SENS_DIR_Pin, GPIO_PIN_RESET);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, NUMBER_ADC_CHANNEL * NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL);
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
  // 上电时的先后逻辑需要重新check


  // 分配所有空间
  oEncoder = as5600_new();
  oEncoder->i2c_handle = &hi2c1;
  oEncoder->dir_port = SENS_DIR_GPIO_Port;
  oEncoder->dir_pin = SENS_DIR_Pin;
  oState = state_new();
  oPdo = pdo_new();
  oSdo = sdo_new();
  oConfig = config_new();

  oPidPosition = pid_new();
  oPidVelocity = pid_new();
  oPidCurrentQ = pid_new();
  oPidCurrentD = pid_new();

  oFilterVelocity = filter_new();
  oFilterCurrentD = filter_new();
  oFilterCurrentQ = filter_new();


  // 初始化as5600
  as5600_init(oEncoder);
  
  // 初始化pdo，状态量由pdo生成；主要是对目标值进行初始化：位置pi，速度0，电流0，模式0
  pdo_init(oPdo);
  // 初始化状态量，感觉这里可有可无，待测试
  state_init(oState);



  // 初始化sdo
  read_config(oSdo);

  // 初始化config, pid, filter
  para_init(oSdo, oConfig);

  CAN_User_Init(&hcan);
  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    printf("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, ", \
        _IQtoF(oState->iqPos), _IQtoF(oState->iqTargP), _IQtoF(oState->iqVel), _IQtoF(oState->iqTargV), \
        _IQtoF(oState->iqCurQ), _IQtoF(oState->iqTargQ), _IQtoF(oState->iqVoltQ));
      
//      printf("%.4f, %.4f, %.4f, %.4f, ", _IQ15toF(oPidPosition->Kp), _IQ15toF(oPidPosition->Ki), _IQ15toF(oPidPosition->integrator), \
//        _IQ15toF(oPidPosition->out));
      
      printf("%.4f, %.4f, %.4f, %.4f, ", _IQ15toF(oPidVelocity->Kp), _IQ15toF(oPidVelocity->Ki), _IQ15toF(oPidVelocity->integrator), \
        _IQ15toF(oPidVelocity->out));

//      printf("%.4f, %.4f, %.4f, %.4f, ", _IQ15toF(oPidCurrentQ->Kp), _IQ15toF(oPidCurrentQ->Ki), _IQ15toF(oPidCurrentQ->integrator), \
//        _IQ15toF(oPidCurrentQ->out));
      
//      printf("%.4f, %.4f, %.4f, %.4f, ", _IQ15toF(oPidCurrentD->Kp), _IQ15toF(oPidCurrentD->Ki), _IQ15toF(oPidCurrentD->integrator), \
//        _IQ15toF(oPidCurrentD->out));
      
//      printf("%d, %.4f, %d\r\n", oConfig->ELEC_ZERO_POSITION, _IQtoF(oState->iqPosElec), oPdo->PDO_ACTUAL_POSITION);
//      printf("%d, %d\r\n", oConfig->CONST_ADC0_OFFSET, oConfig->CONST_ADC1_OFFSET);

      printf("\r\n");
    HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
