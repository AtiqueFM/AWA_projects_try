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
#include "Application.h"
#include <string.h>
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

uint8_t uart_rx_buffer;						/*< Will receive single byte from UART and will transfer it to the RX buffer array.*/
uint8_t uart_rx_buffer_uart1;				/*< Will receive single byte from UART and will transfer it to the RX buffer array.*/
uint8_t uart_rx_buffer_uart3;				/*< Will receive single byte from UART and will transfer it to the RX buffer array.*/
volatile uint8_t uart_rx_bytes; 			/*< Will contain the received byte count.*/
volatile uint8_t uart_rx_bytes_uart1; 		/*< Will contain the received byte count.*/
volatile uint8_t uart_rx_bytes_uart3; 		/*< Will contain the received byte count.*/
volatile uint8_t uart_rcv_bytes;			/*< Flag will be set when the first byte will be received, will be served in the timer ISR*/
volatile uint16_t uart_rx_timeout_counter;
volatile uint16_t uart_rx_timeout_counter_uart1;
volatile uint16_t uart_rx_timeout_counter_uart3;
volatile uint8_t uart_rx_process_query;		/*< This flag will be set when the slave ID is correct and the query needs to be processed.*/
uint32_t budrate_9600 = 38400;//19200;//9600;		/*<For testing, lateron will be replaced by the moodbus register.*/
uint32_t baudrate_uart1 = 9600;		/*<For testing, lateron will be replaced by the moodbus register.*/
uint32_t baudrate_uart3 = 9600;		/*<For testing, lateron will be replaced by the moodbus register.*/
uint8_t tx_byte_count = 0;
uint8_t rx_byte_count = 0;
uint8_t tx_byte_count_uart1 = 0;
uint8_t rx_byte_count_uart1 = 0;
uint8_t tx_byte_count_uart3 = 0;
uint8_t rx_byte_count_uart3 = 0;

//For RTU MODBUS
volatile uint8_t MOD2_RxFlag;
volatile uint8_t MOD3_RxFlag;
extern volatile uint8_t MOD2_Rxbuff[310];
extern volatile uint8_t MOD2_Txbuff[310];
extern volatile uint16_t MOD2_Rxptr, MOD2_Txptr, MOD2_RxBytes, MOD2_TxBytes;
extern volatile uint8_t MOD3_Rxbuff[310];
extern volatile uint8_t MOD3_Txbuff[310];
extern volatile uint16_t MOD3_Rxptr, MOD3_Txptr, MOD3_RxBytes, MOD3_TxBytes;
extern uint16_t uart6_tx_length;
extern MODBUS_config_t PORT_1;
extern MODBUS_config_t PORT_2;
extern MODBUS_config_t PORT_3;

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
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim12;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
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
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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
#if 0
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
#endif

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
  uint8_t flag = 0;
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
  uart_rx_timeout_counter_uart3 += 1;

  //switch(baudrate_uart3)/*TODO: this variable will be changed to a modbus register (USER SELECTABLE)*/
  switch(PORT_3.baudrate)/*TODO: this variable will be changed to a modbus register (USER SELECTABLE)*/
  {
  case 9600:
	  //3.7ms timeout
	  if(uart_rx_timeout_counter_uart3 >= 37)
	  {
		  flag = SET;
	  	  HAL_TIM_Base_Stop_IT(&htim12);
	  }
	  else{}
	  break;
  case 19200:
	  //1.9ms timeout
	  if(uart_rx_timeout_counter_uart3 >= 19)
	  {
		  flag = SET;
	  	  HAL_TIM_Base_Stop_IT(&htim12);
	  }
	  else{}
	  break;
  case 38400:
	  //911us timeout
	  if(uart_rx_timeout_counter_uart3 >= 10)
	  {
		  flag = SET;
		  HAL_TIM_Base_Stop_IT(&htim12);
	  }
	  else{}
	  break;
  case 115200:
	  //300us timeout
	  if(uart_rx_timeout_counter_uart3 >= 3)
	  {
		  flag = SET;
		  HAL_TIM_Base_Stop_IT(&htim12);
	  }
	  else{}
	  break;
  }

  if(flag == SET)
  {
	  /* TODO:	if the timer expires; the frame has been received from the master,
	   * 		now process the query received.
	   */
	  flag = RESET;
	  /*1. Check for the slave ID*/
	  if(MOD3_Rxbuff[SLAVE_ID] == PORT_3.slave_ID)
      //if(Rxbuff[SLAVE_ID] == 1)
	  {
		  /*2. If slave ID is correct, set the flag for processing the query, this flag will be served in the super / while loop.*/
		  //uart_rx_process_query = SET;
		  MOD3_RxFlag = SET;
		  //Copy the no of bytes received to a global variable for its use is MODBUS file.
		  rx_byte_count_uart3 = uart_rx_bytes_uart3;
		  //Reset the UART 6 timeout counter.
		  uart_rx_timeout_counter_uart3 = 0;
		  /*3. Stop the timer*/
		  //HAL_TIM_Base_Stop_IT(&htim7);
	  }
	  else
	  {
		  //HAL_UART_DMAStop(&huart6);
		  //Abort the UART Rx interupt
		  HAL_UART_AbortReceive_IT(&huart3);
		  //Reset the timer 7 timeout counter
		  uart_rx_timeout_counter_uart3 = 0;
		  //HAL_TIM_Base_Stop_IT(&htim7);
		  /*3. if the Slave ID is not correct the flush the RX buffer*/
		  //memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		  memset(MOD3_Rxbuff,'\0',sizeof(MOD3_Rxbuff));
		  //Reset the rx byte counter
		  uart_rx_bytes_uart3 = 0;
		  //Reset the query processing flag.
		  //uart_rx_process_query = RESET;
		  //re-start the uart reception interrupt
		  HAL_UART_Receive_IT(&huart3, &uart_rx_buffer_uart3, 1);
		  //HAL_TIM_Base_Start_IT(&htim7);//this of commented as after restart of rx interrupt timer will be turned on.

	  }

  }
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  int flag = 0;
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
  uart_rx_timeout_counter_uart1 += 1;

  //switch(baudrate_uart1)/*TODO: this variable will be changed to a modbus register (USER SELECTABLE)*/
  switch(PORT_2.baudrate)
  {
  case 9600:
	  //3.7ms timeout
	  if(uart_rx_timeout_counter_uart1 >= 37)
	  {
		  flag = SET;
	  	  HAL_TIM_Base_Stop_IT(&htim5);
	  }
	  else{}
	  break;
  case 19200:
	  //1.9ms timeout
	  if(uart_rx_timeout_counter_uart1 >= 19)
	  {
		  flag = SET;
	  	  HAL_TIM_Base_Stop_IT(&htim5);
	  }
	  else{}
	  break;
  case 38400:
	  //911us timeout
	  if(uart_rx_timeout_counter_uart1 >= 10)
	  {
		  flag = SET;
		  HAL_TIM_Base_Stop_IT(&htim5);
	  }
	  else{}
	  break;
  case 115200:
	  //300us timeout
	  if(uart_rx_timeout_counter_uart1 >= 3)
		  flag = SET;
	  else{}
	  break;
  }

  if(flag == SET)
  {
	  /* TODO:	if the timer expires; the frame has been received from the master,
	   * 		now process the query received.
	   */
	  flag = RESET;
	  /*1. Check for the slave ID*/
	  //if(MOD2_Rxbuff[SLAVE_ID] == 2)
      if(MOD2_Rxbuff[SLAVE_ID] == PORT_2.slave_ID)
	  {
		  /*2. If slave ID is correct, set the flag for processing the query, this flag will be served in the super / while loop.*/
		  //uart_rx_process_query = SET;
		  RxFlag = SET;//Query processing flag
		  //Copy the no of bytes received to a global variable for its use is MODBUS file.
		  rx_byte_count_uart1 = uart_rx_bytes_uart1;
		  //Reset the UART 6 timeout counter.
		  uart_rx_timeout_counter_uart1 = 0;
		  /*3. Stop the timer*/
		  //HAL_TIM_Base_Stop_IT(&htim7);
	  }
	  else
	  {
		  //HAL_UART_DMAStop(&huart6);
		  //Abort the UART Rx interupt
		  HAL_UART_AbortReceive_IT(&huart1);
		  //Reset the timer 7 timeout counter
		  uart_rx_timeout_counter_uart1 = 0;
		  //HAL_TIM_Base_Stop_IT(&htim7);
		  /*3. if the Slave ID is not correct the flush the RX buffer*/
		  //memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		  memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		  //Reset the rx byte counter
		  uart_rx_bytes_uart1 = 0;
		  //Reset the query processing flag.
		  //uart_rx_process_query = RESET;
		  //re-start the uart reception interrupt
		  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer_uart1, 1);
		  //HAL_TIM_Base_Start_IT(&htim7);//this of commented as after restart of rx interrupt timer will be turned on.

	  }
  }
  /* USER CODE END TIM5_IRQn 1 */
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
//  if(PUMPControlHandle_t.u8PUMP_no == PUMP1)//0x01)//pump 1
//	  CoilStatusRegister_t.CoilStatus_t.read_acid = !CoilStatusRegister_t.CoilStatus_t.read_acid;
//  if(PUMPControlHandle_t.u8PUMP_no == PUMP2)//0x01)//pump 1
//	  CoilStatusRegister_t.CoilStatus_t.read_sample = !CoilStatusRegister_t.CoilStatus_t.read_sample;
  if(PUMPControlHandle_t.u16TIM6_count >= PUMPControlHandle_t.u16TIM6_limit)
  {
	  if(PUMPControlHandle_t.u8PUMP_no == PUMP1)//0x01)//pump 1
	  {
		  PUMPControlHandle_t.u8PUMP_action = PUMP1_TURN_OFF;//0x05;//turn off the pump 1
//		  CoilStatusRegister_t.CoilStatus_t.read_acid = SET;
	  }
	  if(PUMPControlHandle_t.u8PUMP_no == PUMP2)//0x02)//pump 2
	  {
		  PUMPControlHandle_t.u8PUMP_action = PUMP2_TURN_OFF;//0x06;//turn off the pump 2
//		  CoilStatusRegister_t.CoilStatus_t.read_sample = SET;
	  }
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

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  int flag = 0;
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  //Will be incremented every 100us
  uart_rx_timeout_counter += 1;

  switch(budrate_9600)/*TODO: this variable will be changed to a modbus register (USER SELECTABLE)*/
  {
  case 9600:
	  //3.7ms timeout
	  if(uart_rx_timeout_counter >= 37)
	  {
		  flag = SET;
	  	  HAL_TIM_Base_Stop_IT(&htim7);
	  }
	  else{}
	  break;
  case 19200:
	  //1.9ms timeout
	  if(uart_rx_timeout_counter >= 19)
	  {
		  flag = SET;
	  	  HAL_TIM_Base_Stop_IT(&htim7);
	  }
	  else{}
	  break;
  case 38400:
	  //911us timeout
	  if(uart_rx_timeout_counter >= 10)
	  {
		  flag = SET;
		  HAL_TIM_Base_Stop_IT(&htim7);
	  }
	  else{}
	  break;
  case 115200:
	  //300us timeout
	  if(uart_rx_timeout_counter >= 3)
		  flag = SET;
	  else{}
	  break;
  }

  if(flag == SET)
  {
	  /* TODO:	if the timer expires; the frame has been received from the master,
	   * 		now process the query received.
	   */
	  flag = RESET;
	  /*1. Check for the slave ID*/
	  //if(MOD2_Rxbuff[SLAVE_ID] == 2)
//		int temp = 0;
//		while(huart6.Instance->DR != 0)
//		{
//			huart6.Instance->DR = 0;
//		}
      if(Rxbuff[SLAVE_ID] == 1)
	  {
		  /*2. If slave ID is correct, set the flag for processing the query, this flag will be served in the super / while loop.*/
		  //uart_rx_process_query = SET;
		  MOD2_RxFlag = SET;
		  //Copy the no of bytes received to a global variable for its use is MODBUS file.
		  rx_byte_count = uart_rx_bytes;
		  //Reset the UART 6 timeout counter.
		  uart_rx_timeout_counter = 0;
		  /*3. Stop the timer*/
		  //HAL_TIM_Base_Stop_IT(&htim7);
	  }
	  else
	  {
		  //HAL_UART_DMAStop(&huart6);
		  //Abort the UART Rx interupt
		  HAL_UART_AbortReceive_IT(&huart6);
		  //Reset the timer 7 timeout counter
		  uart_rx_timeout_counter = 0;
		  //HAL_TIM_Base_Stop_IT(&htim7);
		  /*3. if the Slave ID is not correct the flush the RX buffer*/
		  //memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		  memset(Rxbuff,'\0',sizeof(Rxbuff));
		  //Reset the rx byte counter
		  uart_rx_bytes = 0;
		  //Reset the query processing flag.
		  uart_rx_process_query = RESET;
		  //re-start the uart reception interrupt
		  HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);
		  //HAL_TIM_Base_Start_IT(&htim7);//this of commented as after restart of rx interrupt timer will be turned on.

	  }

  }
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	if(huart == &huart6)
	{
		//Read the received data from UART from Data Register.
	//	HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);
		//Reset the timer 7 UART 6 timeout counter.
		uart_rx_timeout_counter = 0;

		//Fill the RX array buffer
		//MOD2_Rxbuff[uart_rx_bytes] = uart_rx_buffer;
		Rxbuff[uart_rx_bytes] = uart_rx_buffer;

		//increment the counter
		uart_rx_bytes += 1;

		/*1. if the first byte is received i.e; uart_rx_bytes == 1, then load the timer counter / start the timer counter.
		* also set the flag rcv_flag for the processing of the query in the timer ISR, when timeout occurs.*/
		if(uart_rx_bytes == 1)
		{
			//Set a flag to process the modbus query.
			uart_rcv_bytes = SET;
			//MOD2_RxFlag = uart_rcv_bytes;
			HAL_TIM_Base_Start_IT(&htim7);

			/*Depending on the baud rate selected load the timer with that counter (ALREADY DONE IN THE TIMER ISR)*/
		}

		/**TEST***/
//		if(uart_rx_bytes == 8)
//		{
//			uart_rx_bytes = 8;
//		}
//		if(uart_rx_bytes == 6)
//		{
//			uart_rx_bytes = 6;
//		}
		/*****/

		//Start the reception of the next byte.
		HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);

		/*2. else reset the timer counter*/
		htim7.Instance->CNT = 0x00;
	}

	if(huart == &huart1)
	{
		//Reset the timer 5 UART 1 timeour counter
		uart_rx_timeout_counter_uart1 = 0;

		//Fill the RX buffer for RTU
		MOD2_Rxbuff[uart_rx_bytes_uart1] = uart_rx_buffer_uart1;

		//increment the counter
		uart_rx_bytes_uart1 += 1;

		if(uart_rx_bytes_uart1 == 1)
		{
			//Set a flag to process the modbus query.
			//uart_rcv_bytes = SET;
			//MOD2_RxFlag = uart_rcv_bytes;
			HAL_TIM_Base_Start_IT(&htim5);
		}

		//Start the reception of the next byte.
		HAL_UART_Receive_IT(&huart1, &uart_rx_buffer_uart1, 1);

		/*2. else reset the timer counter*/
		htim5.Instance->CNT = 0x00;
	}
	if(huart == &huart3)
	{
		//Reset the timer 5 UART 1 timeour counter
		uart_rx_timeout_counter_uart3 = 0;

		//Fill the RX buffer for RTU
		MOD3_Rxbuff[uart_rx_bytes_uart3] = uart_rx_buffer_uart3;

		//increment the counter
		uart_rx_bytes_uart3 += 1;

		if(uart_rx_bytes_uart3 == 1)
		{
			//Set a flag to process the modbus query.
			//uart_rcv_bytes = SET;
			//MOD2_RxFlag = uart_rcv_bytes;
			HAL_TIM_Base_Start_IT(&htim12);
		}

		//Start the reception of the next byte.
		HAL_UART_Receive_IT(&huart3, &uart_rx_buffer_uart3, 1);

		/*2. else reset the timer counter*/
		htim12.Instance->CNT = 0x00;
	}
}

/*********************DEPRICATED CODE******************************/
#if 1
/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if(huart == &huart6)
	{
		//increment the counter on byte transmitted
		//DMA_TX_FLAG = 1;//Caused delay in transmission

#if 1
		HAL_TIM_Base_Stop_IT(&htim7);
		/*3. if the Slave ID is not correct the flush the RX buffer*/
		//memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		memset((uint16_t*)Rxbuff,'\0',sizeof(Rxbuff));
		//Reset the rx byte counter
		uart_rx_bytes = 0;
		//Reset the query processing flag.
		uart_rx_process_query = RESET;
		//re-start the uart reception interrupt

		ADM_2_CLTR_LOW();
		HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);
		HAL_TIM_Base_Start_IT(&htim7);
#endif

	//	if(tx_byte_count >= uart6_tx_length)
	//	{
	//		tx_byte_count = 0;
	//	}
	}
	if(huart == &huart1)
	{
		HAL_TIM_Base_Stop_IT(&htim5);
		/*3. if the Slave ID is not correct the flush the RX buffer*/
		//memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		memset((uint16_t*)Rxbuff,'\0',sizeof(Rxbuff));
		//Reset the rx byte counter
		uart_rx_bytes_uart1 = 0;
		//Reset the query processing flag.
		//uart_rx_process_query = RESET;
		//re-start the uart reception interrupt

		ADM_CLTR_LOW();
		HAL_UART_Receive_IT(&huart1, &uart_rx_buffer_uart1, 1);
		HAL_TIM_Base_Start_IT(&htim5);
	}

	if(huart == &huart3)
	{
		HAL_TIM_Base_Stop_IT(&htim12);
		/*3. if the Slave ID is not correct the flush the RX buffer*/
		//memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		memset((uint16_t*)MOD3_Rxbuff,'\0',sizeof(MOD3_Rxbuff));
		//Reset the rx byte counter
		uart_rx_bytes_uart3 = 0;
		//Reset the query processing flag.
		//uart_rx_process_query = RESET;
		//re-start the uart reception interrupt

		ADM_3_CLTR_LOW();
		HAL_UART_Receive_IT(&huart3, &uart_rx_buffer_uart3, 1);
		HAL_TIM_Base_Start_IT(&htim12);
	}

}
#endif
#if 0
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
#if 0
	/*Clear the UART6 Transfer complete flag*/
	while(!(huart6.Instance->SR & USART_SR_TC));
	huart6.Instance->SR &= !USART_SR_TC;

	/*Disable the UART DMA Tx*/
	huart6.Instance->CR3 &= !USART_CR3_DMAT;

	/*Enable the UART DMA Rx*/
	huart6.Instance->CR3 |= USART_CR3_DMAR;

	/*Enable the Stream 2 IRQ*/
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	/*Enable the Stream 2*/
	DMAPeripheralEnable(DMA2_Stream2,ENABLE);
#endif
	/*Make the Direction GPIO pin LOW*/
	DMA_TX_FLAG = 1;
}

/*UART 1 RX*/
void DMA2_Stream5_IRQHandler(void)
{
#if !QUERRY_RX_INIT_LEN_6
	static uint8_t startTransmission = RESET;
	static uint8_t presetWriteTransaction = RESET;
	static uint8_t transactionCount = 0;
#endif
	//test
	static uint8_t multipresetcoilgetdataflag = 0;
	/*Clear the FLAG*/
	DMAInterruptHandle(&DMA_UART1_RX_handle_t);
	/*CLear DMAR*/
	huart1.Instance->CR3 &= !USART_CR3_DMAR;
#if QUERRY_RX_INIT_LEN_6
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
#else
	//TODO: Reconfigure the DMA Rx for remaining MODBUS transaction.
	DMAPeripheralEnable(DMA2_Stream5,DISABLE);

	if(presetWriteTransaction == RESET)
	{
		/*
		 * Will enter this only during first query.
		 */
		switch(Rxbuff[1])
		{
			case MODBUS_FUNCODE_MULTI_PRESET_INPUTCOIL:
			case MODBUS_FUNCODE_MULTI_PRESET_HOLDING:
			{
				uint16_t byteCount = Rxbuff[6];//lower byte
				//byteCount |= (Rxbuff[4] << 8);//higher byte
				//byteCount *= 2;//number of words to number of bytes
				uint16_t noOfTransaction = byteCount + 1;//Byte count + Data + CRC
				DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[8]),
						noOfTransaction);
				RxCRCIndex = 7 + byteCount + 2;//Decrements while extracting CRC data
				//TODO: Increment the MODBUS_DMA_querry_count to 1.
				DMAPeripheralEnable(DMA2_Stream5,ENABLE);
				/*Enable the DMAR*/
				huart1.Instance->CR3 |= USART_CR3_DMAR;
				//TODO: Increment the MODBUS_DMA_querry_count to 1.
				//DMAPeripheralEnable(DMA2_Stream5,ENABLE);
				presetWriteTransaction = SET;
				break;
			}
			case MODBUS_FUNCODE_READ_HOLDING:
			case MODBUS_FUNCODE_READ_STATUSCOIL:
			case MODBUS_FUNCODE_SINGLE_PRESET_INPUTCOIL:
			case MODBUS_FUNCODE_READ_INPUTCOIL:
			case MODBUS_FUNCODE_READ_INPUT:
			case MODBUS_FUNCODE_SINGLE_PRESET_HOLDING:
				//TODO: End reception and start the Transmission
				startTransmission = SET;
				//Give the CRC index value from RX buffer
				RxCRCIndex = 7;
				break;
		}

	}else
	{
		startTransmission = SET;
		presetWriteTransaction = RESET;
	}

	if(startTransmission == SET)
	{
		//Reset the transmission flag
		startTransmission = RESET;

		//Enable reception
		/*Disable the Stream 5 IRQ*/
		HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);

		/*Enable the MODBUS Query process flag*/
		RxFlag = 0x01;

		/*Clear the UART1 Transfer complete flag*/
		huart1.Instance->SR &= !USART_SR_TC;

		/*Disable the DMA2 Stream 5*/
		DMAPeripheralEnable(DMA2_Stream5,DISABLE);
	}
#endif
}

void DMA2_Stream7_IRQHandler(void)
{
	/*Clear the FLAG*/
	DMAInterruptHandle(&DMA_UART1_TX_handle_t);
	//MOD2_RxFlag = 0x01;

	/*Disable the DMA2 Stream 7*/
	DMAPeripheralEnable(DMA2_Stream7,DISABLE);

#if 0
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
#endif
	/*Make the Direction GPIO pin LOW*/
	DMA_TX_FLAG_HMI = 1;
}
#endif
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
