/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdlib.h>
#include<string.h>
#include "FM24CL64B.h"
#include "ADS1115.h"
#include "AD7682.h"
#include <math.h>
#include "stm32f4xx_dma.h"
#include "Application.h"
#include "Calibration.h"
#include "Initialization.h"
#include <time.h>


#define ITM_Port32(n)	(*((volatile unsigned long *)(0xe0000000 +4*n)))
#define SINGLE_SHOT 0
#define TEST	0
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
extern uint8_t uart_rx_buffer;			/*< Will receive single byte from UART and will transfer it to the RX buffer array.*/
extern uint8_t uart_rx_buffer_uart1;	/*< Will receive single byte from UART and will transfer it to the RX buffer array.*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

void ProcessCommandCommands(void);

void PWM_update(void);//test

float LinearEqCalc(float x, float x1, float x2, float y1, float y2);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t p[10];

uint16_t SlotCode[8];

int card_action = 0;
int calFlag = 0;

uint8_t cardScan = 1;
extern ADS1115_Config ADS1115_ReadConfigutationHandle_t;
typedef union{
	uint8_t bytes[2];
	uint16_t word;
}Data;
Data data_;

//AutoZero flag
uint8_t performAUTOZERO;

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}

static inline getconfig(void)
{
	data_.word = ADS1115_OperationReadConfiguration();
	ADS1115_ReadConfigutationHandle_t.ByteData[1] = data_.bytes[0];
	ADS1115_ReadConfigutationHandle_t.ByteData[0] = data_.bytes[1];
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //Delay for other peripherals to initialise
//  HAL_Delay(20);
  //unsigned char SlotNo = 0;


  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  ITM_Port32(31) = 1;
#if MODBUS_MULTI_DROP
  //HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);
#else
  //Configure DMA for UART 6 RX
  DMA_UART6_RX_Init();
#endif

  //Configure DMA for UART 1 RX
  //DMA_UART1_RX_Init();
#if QUERRY_RX_INIT_LEN_6
  DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[0]),DMA_FIRST_TRANSACTION_NO); /*< Initialize the DMA2 Stream 5 with DMA_FIRST_TRANSACTION_NO*/
#else
  //DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[0]),8); /*< Initialize the DMA2 Stream 5 with DMA_FIRST_TRANSACTION_NO*/
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  ITM_Port32(31) = 2;

  //Enable the DMA 2 Stream for UART 6 RX
  //DMAPeripheralEnable(DMA2_Stream2, ENABLE);

  //Start the reception in UART6 using interrupt
  HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);

  //Enable the DMA 2 Stream for UART 1 RX
  //DMAPeripheralEnable(DMA2_Stream5, ENABLE);

  //Start the reception in UART6 using interrupt
  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer_uart1, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ADM_CLTR_LOW();

  ADM_2_CLTR_LOW();
  //test
//  HoldingRegisterdefaultData();
  ModbusReadConfiguration();
  HoldingRegisterdefaultData();
  /**************************/

  PWM_setup();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(SYS_LED_STATUS_GPIO_Port, SYS_LED_STATUS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SYS_LED_ERROR_GPIO_Port, SYS_LED_ERROR_Pin, GPIO_PIN_SET);

  //Read the ID in every SLOT
  ReadSlotID();

  //this is set to 0x01 when the start ADC command is received
  TimerStart = 0x02;
  //printf("Hello AWA!!!\n");
  //memset(&InputRegister_t.bytes[sizeof(PVhandle_t)],'\0',sizeof(InputRegister_t.bytes));
//  int flag_= 0;
  //FRAM_OperationWrite(FRAM_ADDRESS_LASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t)],sizeof(InputRegister_t.bytes));
  //int size = sizeof(PVhandle_t) + sizeof(InputRegister_t.COD_lastCalibration);
#if TEST
  memset(&CurrentOutputCalibration_t,'\0',sizeof(CurrentOutputCalibration_t));
  FRAM_OperationWrite(FRAM_ADDRESS_AO_ELEC_CALIB, (uint8_t*)&CurrentOutputCalibration_t, sizeof(CurrentOutputCalibration_t));
#endif
  //float arrayADC [1000];
  uint16_t samples = 100;
  //float avgs = (float)samples;

  /*Start the internal ADC*/
  InternalADCStartConversion();

  while (1)
  {
	  //Process the commands from the HMI
	  ProcessModesCommands();

	  //Relay outputs and analog outputs
	  Application_AWAIOProcess();

#if 0
	  if(AWAOperationStatus_t.ElectronicCal_AO == 0 \
			  && AWAOperationStatus_t.CalibrationMode != 1 \
			  && AWAOperationStatus_t.FactoryMode != 1)
	  {
		  ParameterIOutProcess();/*26/8/2021*/
//		  ParameterRelayAlarmProcess();/*2/9/2021*/
	  }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Parses the data received from the MODBUS master and execute the command received
	  Application_MODBUSParseQuery();

	  //Stores the process variables and calibration data if the respective flag is raised
	  Application_StoreData();

	  //Will if data is transmitted by the slave and is ready for new query
	  Application_MODBUSRXprocess();

	  /*COD process controls*/
	  FlowSensorReadStatus();
	  MILSwitchReadStatus();


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 24999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  htim2.Instance->SR = 0;
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  htim3.Instance->SR = 0;
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 25;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 800;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  htim4.Instance->SR = 0;
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 2499;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 24999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
	htim6.Instance->SR = 0; // clear interrupt flag
	htim6.Instance->DIER &= ~TIM_IT_UPDATE; //DISnable timer interrupt flag
	htim6.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2499;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /*Start the timer in interrupt mode*/
  //HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  //huart1.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
  huart1.Instance->SR &= !USART_SR_TC;
  huart1.Instance->CR3 |= USART_CR3_DMAR;
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  //huart6.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
  //Enable DMA receive
  huart6.Instance->SR &= !USART_SR_TC;
  //huart6.Instance->CR3 |= USART_CR3_DMAR;
  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CARD_7_SPI_SS_Pin|RELAY_6_Pin|RELAY_7_Pin|RELAY_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CARD_7_PWR_Pin|CARD_7_SEL1_Pin|CARD_7_SEL2_Pin|DO_LV_CH8_Pin 
                          |RELAY_3_Pin|RELAY_4_Pin|CARD_5_SEL2_Pin|CARD_5_SEL1_Pin 
                          |ADM_2_CLTR_Pin|CARD_3_SPI_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CARD_6_SPI_SS_Pin|CARD_6_PWR_Pin|CARD_6_SEL1_Pin|CARD_6_SEL2_Pin 
                          |DO_LV_CH3_Pin|DO_LV_CH4_Pin|DO_LV_CH6_Pin|SYS_LED_ERROR_Pin 
                          |CARD_1_SPI_SS_Pin|CARD_1_PWR_Pin|CARD_1_SEL1_Pin|CARD_1_SEL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_5_Pin|ADM_CLTR_Pin|CARD_3_SEL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CARD_5_PWR_Pin|CARD_5_SPI_SS_Pin|SYS_LED_STATUS_Pin|RELAY_1_Pin 
                          |RELAY_2_Pin|CARD_8_PWR_Pin|CARD_8_SEL1_Pin|CARD_8_SEL2_Pin 
                          |CARD_8_SPI_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CARD_2_SPI_SS_Pin|CARD_3_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, CARD_2_PWR_Pin|CARD_2_SEL1_Pin|CARD_2_SEL2_Pin|CARD_3_SEL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CARD_8_ID3_Pin CARD_7_ID1_Pin CARD_7_ID2_Pin CARD_7_ID3_Pin 
                           CARD_1_ID3_Pin RESERVE_PE15_Pin CARD_8_ID1_Pin CARD_8_ID2_Pin */
  GPIO_InitStruct.Pin = CARD_8_ID3_Pin|CARD_7_ID1_Pin|CARD_7_ID2_Pin|CARD_7_ID3_Pin 
                          |CARD_1_ID3_Pin|RESERVE_PE15_Pin|CARD_8_ID1_Pin|CARD_8_ID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_7_SPI_SS_Pin RELAY_6_Pin RELAY_7_Pin RELAY_8_Pin */
  GPIO_InitStruct.Pin = CARD_7_SPI_SS_Pin|RELAY_6_Pin|RELAY_7_Pin|RELAY_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_7_PWR_Pin CARD_7_SEL1_Pin CARD_7_SEL2_Pin DO_LV_CH8_Pin 
                           RELAY_3_Pin RELAY_4_Pin CARD_5_SEL2_Pin CARD_5_SEL1_Pin 
                           ADM_2_CLTR_Pin CARD_3_SPI_SS_Pin */
  GPIO_InitStruct.Pin = CARD_7_PWR_Pin|CARD_7_SEL1_Pin|CARD_7_SEL2_Pin|DO_LV_CH8_Pin 
                          |RELAY_3_Pin|RELAY_4_Pin|CARD_5_SEL2_Pin|CARD_5_SEL1_Pin 
                          |ADM_2_CLTR_Pin|CARD_3_SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_6_ID1_Pin CARD_6_ID2_Pin CARD_6_ID3_Pin */
  GPIO_InitStruct.Pin = CARD_6_ID1_Pin|CARD_6_ID2_Pin|CARD_6_ID3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_6_SPI_SS_Pin CARD_6_PWR_Pin CARD_6_SEL1_Pin CARD_6_SEL2_Pin 
                           DO_LV_CH3_Pin DO_LV_CH4_Pin DO_LV_CH6_Pin SYS_LED_ERROR_Pin 
                           CARD_1_SPI_SS_Pin CARD_1_PWR_Pin CARD_1_SEL1_Pin CARD_1_SEL2_Pin */
  GPIO_InitStruct.Pin = CARD_6_SPI_SS_Pin|CARD_6_PWR_Pin|CARD_6_SEL1_Pin|CARD_6_SEL2_Pin 
                          |DO_LV_CH3_Pin|DO_LV_CH4_Pin|DO_LV_CH6_Pin|SYS_LED_ERROR_Pin 
                          |CARD_1_SPI_SS_Pin|CARD_1_PWR_Pin|CARD_1_SEL1_Pin|CARD_1_SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : MIL_SWITCH_Pin CARD_5_ID1_Pin CARD_5_ID2_Pin CARD_5_ID3_Pin 
                           CARD_3_ID1_Pin CARD_3_ID2_Pin */
  GPIO_InitStruct.Pin = MIL_SWITCH_Pin|CARD_5_ID1_Pin|CARD_5_ID2_Pin|CARD_5_ID3_Pin 
                          |CARD_3_ID1_Pin|CARD_3_ID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_5_Pin ADM_CLTR_Pin CARD_3_SEL2_Pin */
  GPIO_InitStruct.Pin = RELAY_5_Pin|ADM_CLTR_Pin|CARD_3_SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_5_PWR_Pin CARD_5_SPI_SS_Pin SYS_LED_STATUS_Pin RELAY_1_Pin 
                           RELAY_2_Pin CARD_8_PWR_Pin CARD_8_SEL1_Pin CARD_8_SEL2_Pin 
                           CARD_8_SPI_SS_Pin */
  GPIO_InitStruct.Pin = CARD_5_PWR_Pin|CARD_5_SPI_SS_Pin|SYS_LED_STATUS_Pin|RELAY_1_Pin 
                          |RELAY_2_Pin|CARD_8_PWR_Pin|CARD_8_SEL1_Pin|CARD_8_SEL2_Pin 
                          |CARD_8_SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_1_ID1_Pin CARD_1_ID2_Pin CARD_2_ID1_Pin CARD_2_ID2_Pin 
                           CARD_2_ID3_Pin RESERVE_PG8_Pin CARD_3_ID3_Pin RESERVE_PG11_Pin 
                           RESERVE_PG12_Pin RESERVE_PG13_Pin RESERVE_PG14_Pin RESERVE_PG15_Pin */
  GPIO_InitStruct.Pin = CARD_1_ID1_Pin|CARD_1_ID2_Pin|CARD_2_ID1_Pin|CARD_2_ID2_Pin 
                          |CARD_2_ID3_Pin|RESERVE_PG8_Pin|CARD_3_ID3_Pin|RESERVE_PG11_Pin 
                          |RESERVE_PG12_Pin|RESERVE_PG13_Pin|RESERVE_PG14_Pin|RESERVE_PG15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : RESERVE_PD11_Pin RESERVE_PD12_Pin RESERVE_PD13_Pin RESERVE_PD14_Pin */
  GPIO_InitStruct.Pin = RESERVE_PD11_Pin|RESERVE_PD12_Pin|RESERVE_PD13_Pin|RESERVE_PD14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_2_SPI_SS_Pin CARD_3_PWR_Pin */
  GPIO_InitStruct.Pin = CARD_2_SPI_SS_Pin|CARD_3_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_2_PWR_Pin CARD_2_SEL1_Pin CARD_2_SEL2_Pin CARD_3_SEL1_Pin */
  GPIO_InitStruct.Pin = CARD_2_PWR_Pin|CARD_2_SEL1_Pin|CARD_2_SEL2_Pin|CARD_3_SEL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ReadSlotInfo(void)
{

	//SLOT 1
	SlotCode[1] = 0;//Slot1Code = 0;
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5))  // Mux1Code_bit3	PE5
	{
		SlotCode[1] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4))  // Mux1Code_bit2	PE4
	{
		SlotCode[1] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3))  // Mux1Code_bit1	PE3
	{
		SlotCode[1] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))  // Mux1Code_bit0	PE2
	{
		SlotCode[1] |= 0x01;
	}

	//SLOT 2
	SlotCode[2] = 0;//Slot2Code = 0;
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3))  // Mux2Code_bit3	PF3
	{
		SlotCode[2] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2))  // Mux2Code_bit2	PF2
	{
		SlotCode[2] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1))  // Mux2Code_bit1	PF1
	{
		SlotCode[2] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0))  // Mux2Code_bit0	PF0
	{
		SlotCode[2] |= 0x01;
	}

	//SLOT 3
	SlotCode[3] = 0;//Slot3Code = 0;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))  // Mux3Code_bit3	PA7
	{
		SlotCode[3] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))  // Mux3Code_bit2	PA6
	{
		SlotCode[3] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))  // Mux3Code_bit1	PA5
	{
		SlotCode[3] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))  // Mux3Code_bit0	PA4
	{
		SlotCode[3] |= 0x01;
	}

	//SLOT 4
	SlotCode[4] = 0;//Slot4Code = 0;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))  // Mux4Code_bit3	PE8
	{
		SlotCode[4] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))  // Mux4Code_bit2	PE7
	{
		SlotCode[4] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1))  // Mux4Code_bit1	PG1
	{
		SlotCode[4] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0))  // Mux4Code_bit0	PG0
	{
		SlotCode[4] |= 0x01;
	}

	//SLOT 5
	SlotCode[5] = 0;//Slot5Code = 0;
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7))  // Mux5Code_bit3	PG8
	{
		SlotCode[5] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7))  // Mux5Code_bit2	PG7
	{
		SlotCode[5] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6))  // Mux5Code_bit1	PG6
	{
		SlotCode[5] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5))  // Mux5Code_bit0	PG5
	{
		SlotCode[5] |= 0x01;
	}

	//SLOT 6
	SlotCode[6] = 0;//Slot6Code = 0;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))  // Mux6Code_bit3	PA12
	{
		SlotCode[6] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))  // Mux6Code_bit2	PA11
	{
		SlotCode[6] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))  // Mux6Code_bit1	PA10
	{
		SlotCode[6] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9))  // Mux6Code_bit0	PA9
	{
		SlotCode[6] |= 0x01;
	}

	//SLOT 7
	SlotCode[7] = 0;//Slot7Code = 0;
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14))  // Mux7Code_bit3	PG14
	{
		SlotCode[7] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13))  // Mux7Code_bit2	PG13
	{
		SlotCode[7] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12))  // Mux7Code_bit1	PG12
	{
		SlotCode[7] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11))  // Mux7Code_bit0	PG10
	{
		SlotCode[7] |= 0x01;
	}

	//SLOT 8
	SlotCode[8] = 0;//Slot8Code = 0;
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))  // Mux8Code_bit3	PB4
	{
		SlotCode[8] |= 0x08;
	}
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))  // Mux8Code_bit2	PB5
	{
		SlotCode[8] |= 0x04;
	}
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))  // Mux8Code_bit1	PB6
	{
		SlotCode[8] |= 0x02;
	}
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))  // Mux8Code_bit0	PB7
	{
		SlotCode[8] |= 0x01;
	}

}



void ProcessCommandCommands(void)
{
	switch(HoldingRegister_t.ModeCommand_t.CommonCommand)
	{
		case Sample_pump:
		{
			PumpOperation(0x01);
			break;
		}
		case Clean_pump:
		{
			PumpOperation(0x02);
			break;
		}
		case Measure_sample:
		{
			break;
		}
	}
}


void IOCalculation(uint16_t channel)
{
	switch (channel)
	{
		case AO1:
		{
			break;
		}
	}
}

/*
 * 	LinearEqCalc - Straight line equation. For given two points (x1, y1) and (x2, y2) this function
 * 		calculates and returns y for given x. This is extensively used throughout the project for
 * 		calculation tasks - such as - electronic/Transmitter calibration, sensor calibration or any
 * 		linear relationship.
 * 		Note: convert to macro, if type-less variables are required in future. (Now for float type)
 * 	Parameters -
 * 		float		x		given x value - for which y will be computed
 * 		float		x1		x1 of point (x1, y1)
 * 		float		x2		x2 of point (x2, y2)
 * 		float		y1		y1 of point (x1, y1)
 * 		float		y2		y2 of point (x2, y2)
 * 	Returns -
 * 		float				y computed for given x
 */
float LinearEqCalc(float x, float x1, float x2, float y1, float y2)
{
	return (y1 +  (x - x1) * (y2 - y1) / (x2 - x1));	// equation of straight line for given 2 points. Ref - your school math's textbook!


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
