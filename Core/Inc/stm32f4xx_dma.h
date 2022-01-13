/*
 * stm32f4xx_dma.h
 *
 *  Created on: Dec 30, 2021
 *      Author: atiqueshaikh
 */

#ifndef INC_STM32F4XX_DMA_H_
#define INC_STM32F4XX_DMA_H_

#include "stm32f407xx.h"

#define NVIC_ISER0					((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1					((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2					((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3					((volatile uint_2_t*)0xE000E10C)

#define NVIC_ICER0					((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1					((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2					((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3					((volatile uint32_t*)0xE000E18C)

//Interrupt IRQ numbers
/* DMA1 */
#define		IRQ_NO_DMA1_Stream0			11
#define		IRQ_NO_DMA1_Stream1			12
#define		IRQ_NO_DMA1_Stream2			13
#define		IRQ_NO_DMA1_Stream3			14
#define		IRQ_NO_DMA1_Stream4			15
#define		IRQ_NO_DMA1_Stream5			16
#define		IRQ_NO_DMA1_Stream6			17
#define		IRQ_NO_DMA1_Stream7			47
/* DMA2 */
#define		IRQ_NO_DMA2_Stream0			56
#define		IRQ_NO_DMA2_Stream1			57
#define		IRQ_NO_DMA2_Stream2			58
#define		IRQ_NO_DMA2_Stream3			59
#define		IRQ_NO_DMA2_Stream4			60
#define		IRQ_NO_DMA2_Stream5			68
#define		IRQ_NO_DMA2_Stream6			69
#define		IRQ_NO_DMA2_Stream7			70

//DMA Configuration handle
typedef struct{
	uint8_t StreamSelect;
	uint8_t ChannelSelect; //CHSEL
	uint8_t PeripheralDataSize;//PSIZE destination and peripheral size same in direct mode
	uint8_t MemoryDataSize;//MSIZE
	uint8_t MemoryIncrementEnable;//MINC
	uint8_t PeripheralIncrementEnable;//PINC
	uint8_t DataFlowDirection;//DIR
	uint8_t DirectModeEnable;//DMDIS = 0 IN SxFCR
	uint32_t SourceAddress;
	uint32_t DestinationAddress;
	uint16_t NumberOfTransaction;//SxNDTR
	uint8_t EnableAllInterrupt;//TCIE = 1,TEIE = 1, DMEIE = 1
	uint8_t SelectPriority;
	uint8_t DMATransferCompeleteFLag;
	uint8_t DMADirectModeErrorFlag;
}DMA_Config_t;

//DMA register handle
typedef struct{
	DMA_TypeDef *pDMAx;
	DMA_Stream_TypeDef *pDMAStreamx;
	DMA_Config_t pDMAConfig;
}DMA_Handle_t;


void DMAInit(DMA_Handle_t *pDMAHandle);
void DMAEnable(DMA_TypeDef *pDMAx,uint8_t ENOrDi);//RCC clock
void DMAPeripheralEnable(DMA_Stream_TypeDef *pDMAStreamx,uint8_t ENOrDi);// DMA_EN (Stream enable)
void DMA_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void DMAInterruptHandle(DMA_Handle_t *pDMAHandle);
#endif /* INC_STM32F4XX_DMA_H_ */
