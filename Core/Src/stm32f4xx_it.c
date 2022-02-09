/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_dma.h"
#include "Initialization.h"
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
volatile uint16_t TIM2Flag = TIMER_TICKS_46MS, TIM2State = TIME_46MS;
volatile uint32_t tst = 0, tst1 = 0;
volatile uint8_t RxFlag = 0, ReadADCFlag = 0;

//For RTU MODBUS
volatile uint8_t MOD2_RxFlag;
extern volatile uint8_t MOD2_Rxbuff[310];
extern volatile uint8_t MOD2_Txbuff[310];
extern volatile uint16_t MOD2_Rxptr, MOD2_Txptr, MOD2_RxBytes, MOD2_TxBytes;

uint8_t state = 1;
extern uint8_t codSlotNO;
extern uint8_t phSlotNO;

extern DMA_Handle_t DMA_UART6_RX_handle_t,DMA_UART6_TX_handle_t,DMA_UART1_RX_handle_t,DMA_UART1_TX_handle_t;

uint8_t MODBUS_DMA_querry_count = 0;/*<if first half of the MODBUS query yet to be received then 0
 	 	 	 	 	 	 	 	 	   incremented to 1 after receiving 1st half of the query*/
extern volatile uint8_t RxCRCIndex;/*<For extracting the CRC value from RxBuff[]*/
extern volatile uint8_t RxCRCDataPacketLength;/*<For extracting the data from RxBUff[] except CRC value*/
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
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  //if((htim2.Instance->SR & TIM_FLAG_UPDATE) == TIM_FLAG_UPDATE)
  {
	  //if((htim2.Instance->DIER & TIM_IT_UPDATE) == TIM_IT_UPDATE)
	  {
		  //htim2.Instance->SR &= ~TIM_IT_UPDATE; // clear interrupt flag
		 // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1);

		 // if(SoftTimerOn == 1)
		  {
			  (TIM2Flag > 0)?(TIM2Flag--):(TIM2Flag = 0);

				  if(TIM2State == TIME_6MS)
				  {
					  //if(TIM2Flag == 50 && ReadADCFlag == 0)
					  if(TIM2Flag == 50 && ReadADCFlag == 0)//1ms
					  {
						  //GPIOC->ODR |= (1<<14);//toggle
						  ReadADCFlag = 0x01;
					  }
					  //if(TIM2Flag == 10)//5ms
						  //GPIOC->ODR ^= (1<<14);//toggle
				  }

				  if(TIM2State == TIME_8MS)
				  {
					  //if(TIM2Flag == 60 && ReadADCFlag == 0)
					  if(TIM2Flag == 70 && ReadADCFlag == 0)//1ms
					  {
						  //GPIOC->ODR |= (1<<14);//toggle
						  ReadADCFlag = 0x01;
					  }
					  //if(TIM2Flag == 10)//7ms
						  //GPIOC->ODR ^= (1<<14);//toggle
				  }

			  if(TIM2Flag == 0)
			  {
				  switch(TIM2State)
				  {
					  case TIME_46MS:
					  {
//						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // MUX1 SEL2
						  //CARD_1_SEL2_OFF();
						  //CARD_6_SEL2_OFF();
						  COD_SEL2_OFF(codSlotNO);
						  TIM2Flag = TIMER_TICKS_6MS;
						  TIM2State = TIME_6MS; //60;
						  break;
					  }
					  case TIME_6MS:
					  {
						  tst++;
						//  ReadADCFlag = 0x00;
//						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // MUX1 SEL2
						  //CARD_1_SEL2_ON();
						  //CARD_6_SEL2_ON();
						  COD_SEL2_ON(codSlotNO);
						  TIM2Flag = TIMER_TICKS_1MS;
						  TIM2State = TIME_1MS;//10;
						  break;
					  }
					  case TIME_1MS:
					  {
//						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // MUX1 SEL2
//						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // MUX1 PWR
						  //CARD_1_SEL2_OFF();
						  //CARD_1_LAMP_ON();

						  //CARD_6_SEL2_OFF();
						  //CARD_6_LAMP_ON();

						  COD_SEL2_OFF(codSlotNO);
						  COD_LAMP_ON(codSlotNO);

						  htim4.Instance->DIER |= TIM_IT_UPDATE; // start timer 4
						  htim4.Instance->CR1 |= TIM_CR1_CEN;
						  TIM2Flag = TIMER_TICKS_8MS;
						  TIM2State = TIME_8MS;//80;
						  break;
					  }
					  case TIME_8MS:
					  {
						  tst1++;
						//  ReadADCFlag = 0x00;
//						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // MUX1 SEL2
						  //CARD_1_SEL2_ON();
						  //CARD_6_SEL2_ON();
						  COD_SEL2_ON(codSlotNO);
						  TIM2Flag = TIMER_TICKS_46MS;
						  TIM2State = TIME_46MS;
						  break;
					  }
					  default:
						  break;

				  }
			  }
		  }

//		  (TIM2Flag > 0)?(TIM2Flag--):(TIM2Flag = 0);
//
//		  if(TIM2Flag == 0)
//		  {
//			  switch(TIM2State)
//			  {
//				  case 460:
//				  {
//					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // MUX1 SEL2
//					  TIM2Flag = 60;
//					  TIM2State = 60;
//					  break;
//				  }
//				  case 60:
//				  {
//					  tst++;
//					  ReadADCFlag = 0x00;
//					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // MUX1 SEL2
//					  TIM2Flag = 10;
//					  TIM2State = 10;
//					  break;
//				  }
//				  case 10:
//				  {
//					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // MUX1 SEL2
//					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // MUX1 PWR
//					  htim4.Instance->DIER |= TIM_IT_UPDATE; // start timer 4
//					  htim4.Instance->CR1 |= TIM_CR1_CEN;
//					  TIM2Flag = 80;
//					  TIM2State = 80;
//					  break;
//				  }
//				  case 80:
//				  {
//					  tst1++;
//					  ReadADCFlag = 0x00;
//					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // MUX1 SEL2
//					  TIM2Flag = 460;
//					  TIM2State = 460;
//					  break;
//				  }
//				  default:
//					  break;
//
//			  }
//		  }

	  }
  }

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
//	if((htim3.Instance->SR & TIM_FLAG_UPDATE) == TIM_FLAG_UPDATE)
//	{
//	  if((htim3.Instance->DIER & TIM_IT_UPDATE) == TIM_IT_UPDATE)
//	  {
		htim3.Instance->SR &= ~TIM_IT_UPDATE; // clear interrupt flag
		htim3.Instance->DIER &= ~TIM_IT_UPDATE; //DISnable timer interrupt flag
		htim3.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer
		RxBytes = Rxptr;
		Rxptr = 0;
		huart3.Instance->CR1 &= ~UART_MODE_RX;// Rx disable
		huart3.Instance->CR1 &= ~UART_IT_RXNE; // UART RX not empty interrupt disable
		RxFlag = 0x01;
		// for testing
		/*TxBytes = RxBytes;
		Txptr = 0;
		huart3.Instance->CR1 |= UART_IT_TXE; // tx empty interrupt enable
		huart3.Instance->CR1 |= UART_MODE_TX;// Tx mode enable*/
//	  }
//	}

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	//if((htim4.Instance->SR & TIM_FLAG_UPDATE) == TIM_FLAG_UPDATE)
	  {
		  //if((htim4.Instance->DIER & TIM_IT_UPDATE) == TIM_IT_UPDATE)
		  {
			  htim4.Instance->SR &= ~TIM_IT_UPDATE; // clear interrupt flag
			  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // MUX1 PWR// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // MUX1 PWR
			  //CARD_1_LAMP_OFF();
			  //CARD_6_LAMP_OFF();
			  COD_LAMP_OFF(codSlotNO);
			  // disable timer
			  htim4.Instance->DIER &= ~TIM_IT_UPDATE; // stop timer 4
			  htim4.Instance->CR1 &= ~TIM_CR1_CEN;

		  }
	  }

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  // data received on UART
	if(((huart3.Instance->SR & USART_SR_RXNE) == USART_SR_RXNE) && ((huart3.Instance->CR1 & USART_CR1_RXNEIE)== USART_CR1_RXNEIE))
	{
		// if first character, start timer
		if(Rxptr == 0)
		{
			htim3.Instance->DIER |= TIM_IT_UPDATE; // enable timer interrupt flag
			htim3.Instance->CR1 |= TIM_CR1_CEN; // enable timer
		}
		else
		htim3.Instance->CNT = 0x00000000; // clear timer
		// copy data received on UART to RAM buffer
		Rxbuff[Rxptr++] = (uint8_t)(huart3.Instance->DR & (uint8_t)0x00FF);
		if(Rxptr > 309)
			Rxptr = 1;

	}

	// data to be transmitted
	if(((huart3.Instance->SR & USART_SR_TXE) == USART_SR_TXE) &&
			((huart3.Instance->CR1 & USART_CR1_TXEIE)== USART_CR1_TXEIE) &&
			((huart3.Instance->CR1 & UART_MODE_TX) == UART_MODE_TX))
	{
		if(Txptr > TxBytes) // transmit string is completed
		{
			huart3.Instance->CR1 &= ~UART_IT_TXE; // tx empty interrupt disable
			huart3.Instance->CR1 &= ~UART_MODE_TX; // Tx mode disable
			huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
			huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
			Txptr = 0;
			TxBytes = 0;
			if(TimerStart == 0x01)TimerStart = 0x02;
		}
		else
		{
			huart3.Instance->DR = Txbuff[Txptr++];

		}

	}


  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
//	htim6.Instance->DIER &= ~TIM_IT_UPDATE; // enable timer interrupt flag
//	htim6.Instance->CR1 &= ~TIM_CR1_CEN; // enable timer
  PUMPControlHandle_t.u16TIM6_count += 1;
  if(PUMPControlHandle_t.u16TIM6_count >= PUMPControlHandle_t.u16TIM6_limit)
  {
	  if(PUMPControlHandle_t.u8PUMP_no == PUMP1)//0x01)//pump 1
		  PUMPControlHandle_t.u8PUMP_action = PUMP1_TURN_OFF;//0x05;//turn off the pump 1
	  if(PUMPControlHandle_t.u8PUMP_no == PUMP2)//0x02)//pump 2
		  PUMPControlHandle_t.u8PUMP_action = PUMP2_TURN_OFF;//0x06;//turn off the pump 2
	  if(PUMPControlHandle_t.u8PUMP_delayaction == 0x01)
		  PUMPControlHandle_t.u8PUMP_action = 0x07;// for pump delay time
  }else
	  PUMPControlHandle_t.u8PUMP_action = 0x03;//in operation
#if 0
  if(g_u16TIM6_count == HoldingRegister_t.ModeCommand_t.PUMP1_ONTIME)
  {
	  //TEST
	  state = !state;
	  HAL_GPIO_WritePin(SYS_LED_ERROR_GPIO_Port, SYS_LED_ERROR_Pin, state);
	  //
	  g_u16TIM6_count = 0;
  }
#endif
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/*
 * DMA 2 Stream 2 IRQ Handler
 * 1. Resets the interrupt flag.
 * 2. Clears the UART 6 TC (Transfer complete) flag.
 * 3. Disable the DMA2 Stream 2
 */
void DMA2_Stream2_IRQHandler(void)
{
	/*Clear the FLAG*/
	DMAInterruptHandle(&DMA_UART6_RX_handle_t);

	/*Disable the Stream 2 IRQ*/
	HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);

	/*Enable the MODBUS Query process flag*/
	MOD2_RxFlag = 0x01;

	/*Clear the UART6 Transfer complete flag*/
	huart6.Instance->SR &= !USART_SR_TC;

	/*Disable the DMA2 Stream 2*/
	DMAPeripheralEnable(DMA2_Stream2,DISABLE);
}

void DMA2_Stream6_IRQHandler(void)
{
	/*Clear the FLAG*/
	DMAInterruptHandle(&DMA_UART6_TX_handle_t);
	//MOD2_RxFlag = 0x01;

	/*Disable the DMA2 Stream 6*/
	DMAPeripheralEnable(DMA2_Stream6,DISABLE);

	/*Clear the UART6 Transfer complete flag*/
	huart6.Instance->SR &= !USART_SR_TC;

	/*Disable the UART DMA Tx*/
	huart6.Instance->CR3 &= !USART_CR3_DMAT;

	/*Enable the UART DMA Rx*/
	huart6.Instance->CR3 |= USART_CR3_DMAR;

	/*Enable the Stream 2 IRQ*/
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	/*Enable the Stream 2*/
	DMAPeripheralEnable(DMA2_Stream2,ENABLE);

	/*Make the Direction GPIO pin LOW*/
	dma_tx_flag_AT = 1;
}

/*UART 1 RX*/
void DMA2_Stream5_IRQHandler(void)
{
	//test
	static uint8_t multipresetcoilgetdataflag = 0;
	/*Clear the FLAG*/
	DMAInterruptHandle(&DMA_UART1_RX_handle_t);
	/*CLear DMAR*/
	huart1.Instance->CR3 &= !USART_CR3_DMAR;

	if(!MODBUS_DMA_querry_count)
	{
		//TODO: Reconfigure the DMA Rx for remaining MODBUS transaction.
		DMAPeripheralEnable(DMA2_Stream5,DISABLE);

		switch(Rxbuff[1])
		{
		case MODBUS_FUNCODE_READ_HOLDING:
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
								DMA_SECOND_HOLDINGREAD_TRANSACTION_NO);
			RxCRCIndex = 7;
			break;
		case MODBUS_FUNCODE_READ_INPUT:
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
								DMA_SECOND_INPUTREAD_TRANSACTION_NO);
			RxCRCIndex = 7;
			RxCRCDataPacketLength = 6;
			break;
		case MODBUS_FUNCODE_SINGLE_PRESET_HOLDING:
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
								DMA_SECOND_SINGLEPRESETHOLDING_TRANSACTION_NO);
			RxCRCIndex = 7;
			break;
		case MODBUS_FUNCODE_MULTI_PRESET_HOLDING:
		{
			uint16_t byteCount = Rxbuff[5];//lower byte
			byteCount |= (Rxbuff[4] << 8);//higher byte
			byteCount *= 2;//number of words to number of bytes
			uint16_t noOfTransaction = byteCount + 2 + 1;//Byte count + Data + CRC
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
					noOfTransaction);
			RxCRCIndex = 8 + byteCount;
			break;
		}
		case MODBUS_FUNCODE_READ_INPUTCOIL:
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
								DMA_SECOND_INPUTCOILREAD_TRANSACTION_NO);
			RxCRCIndex = 7;
			break;
		case MODBUS_FUNCODE_SINGLE_PRESET_INPUTCOIL:
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
								DMA_SECOND_SINGLEPRESETHOLDING_TRANSACTION_NO);
			RxCRCIndex = 7;
			break;
		case MODBUS_FUNCODE_READ_STATUSCOIL:
			DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
								DMA_SECOND_STATUSCOILREAD_TRANSACTION_NO);
			RxCRCIndex = 7;
			break;
		case MODBUS_FUNCODE_MULTI_PRESET_INPUTCOIL:
		{
			uint8_t regCount = 0;
			if(!multipresetcoilgetdataflag)
			{
				DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[6]),
									1);
				multipresetcoilgetdataflag = 1;
			}
			else
			{
				regCount = Rxbuff[6] + 2;
				DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[7]),
									regCount);
				multipresetcoilgetdataflag = 0;

			}
			RxCRCIndex = 6 + 1 + regCount + 1;
			break;
		}
		}
		/*Enable the DMAR*/
		huart1.Instance->CR3 |= USART_CR3_DMAR;
		//TODO: Increment the MODBUS_DMA_querry_count to 1.
		DMAPeripheralEnable(DMA2_Stream5,ENABLE);

		if(multipresetcoilgetdataflag)
		{
			MODBUS_DMA_querry_count = 0;
		}
		else
		{
			MODBUS_DMA_querry_count = 1;
		}
	}
	else if(MODBUS_DMA_querry_count == 1)
	{
		//TODO: Reset the MODBUS_DMA_querry_count to 0.
		MODBUS_DMA_querry_count = 0;

		//TODO: Paste the below code.
		/*Disable the Stream 5 IRQ*/
		HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);

		/*Enable the MODBUS Query process flag*/
		RxFlag = 0x01;

		/*Clear the UART1 Transfer complete flag*/
		huart1.Instance->SR &= !USART_SR_TC;

		/*Disable the DMA2 Stream 5*/
		DMAPeripheralEnable(DMA2_Stream5,DISABLE);

	}

}

void DMA2_Stream7_IRQHandler(void)
{
	/*Clear the FLAG*/
	DMAInterruptHandle(&DMA_UART1_TX_handle_t);
	//MOD2_RxFlag = 0x01;

	/*Disable the DMA2 Stream 7*/
	DMAPeripheralEnable(DMA2_Stream7,DISABLE);

	/*Clear the UART1 Transfer complete flag*/
	while(!(huart1.Instance->SR & USART_SR_TC));
	huart1.Instance->SR &= !USART_SR_TC;

	/*Disable the UART DMA Tx*/
	huart1.Instance->CR3 &= !USART_CR3_DMAT;

	/*Enable the UART DMA Rx*/
	huart1.Instance->CR3 |= USART_CR3_DMAR;

	/*Configure the Stream 5 Rx with DMA_FIRST_TRANSACTION_NO*/
	DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[0]),DMA_FIRST_TRANSACTION_NO);

	/*Enable the Stream 5 IRQ*/
	//HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	/*Enable the Stream 5*/
	DMAPeripheralEnable(DMA2_Stream5,ENABLE);

	/*Make the Direction GPIO pin LOW*/
	dma_tx_flag_uart1 = 1;
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
