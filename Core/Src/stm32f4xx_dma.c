/*
 * stm32f4xx_dma.c
 *
 *  Created on: Dec 30, 2021
 *      Author: atiqueshaikh
 */

#include "stm32f4xx_dma.h"
#include "main.h"

void DMAInit(DMA_Handle_t *pDMAHandle)
{

	uint32_t tempReg = 0;

	//Stream configuration
	tempReg = 0;
	//Select the channel
	tempReg |= (pDMAHandle->pDMAConfig.ChannelSelect << 25);//1
	//Select the priority of the channel
	tempReg |= (pDMAHandle->pDMAConfig.SelectPriority << 16);//11
	//Select the Data transmission size
	tempReg |= (pDMAHandle->pDMAConfig.PeripheralDataSize << 11);
	//Select memory data size
	tempReg |= (pDMAHandle->pDMAConfig.MemoryDataSize << 13);
	//Enable or disable the memory increment
	tempReg |= (pDMAHandle->pDMAConfig.MemoryIncrementEnable << 10);//1
	//Enable or disable memory increment in peripheral side
	tempReg |= (pDMAHandle->pDMAConfig.PeripheralIncrementEnable << 9);//0
	//Select the data direction flow
	tempReg |= (pDMAHandle->pDMAConfig.DataFlowDirection << 6);//M2P
	//Write the configuration to the Control Register of the selected stream
	pDMAHandle->pDMAStreamx->CR = tempReg;

	//Peripheral to memory
	if(pDMAHandle->pDMAConfig.DataFlowDirection == 0)
	{
		//Configure the source address
		pDMAHandle->pDMAStreamx->M0AR |= (pDMAHandle->pDMAConfig.DestinationAddress << 0);
		//pDMAHanlde->pDMAx->S7M0AR |= (pDMAHanlde->pDMAConfig.SourceAddress << 0);
		//Configure the Destination address
		pDMAHandle->pDMAStreamx->PAR |= (pDMAHandle->pDMAConfig.SourceAddress << 0);
		//pDMAHanlde->pDMAx->S7PAR |= (pDMAHanlde->pDMAConfig.DestinationAddress << 0);
	}
	//Memory to peripheral
	else if(pDMAHandle->pDMAConfig.DataFlowDirection == 1)
	{
		//Configure the source address
		pDMAHandle->pDMAStreamx->M0AR |= (pDMAHandle->pDMAConfig.SourceAddress << 0);
		//pDMAHanlde->pDMAx->S7M0AR |= (pDMAHanlde->pDMAConfig.SourceAddress << 0);
		//Configure the Destination address
		pDMAHandle->pDMAStreamx->PAR |= (pDMAHandle->pDMAConfig.DestinationAddress << 0);
		//pDMAHanlde->pDMAx->S7PAR |= (pDMAHanlde->pDMAConfig.DestinationAddress << 0);
	}
#if 0
	//Configure the source address
	pDMAHandle->pDMAStreamx->M0AR |= (pDMAHandle->pDMAConfig.DestinationAddress << 0);
	//pDMAHanlde->pDMAx->S7M0AR |= (pDMAHanlde->pDMAConfig.SourceAddress << 0);
	//Configure the Destination address
	pDMAHandle->pDMAStreamx->PAR |= (pDMAHandle->pDMAConfig.SourceAddress << 0);
	//pDMAHanlde->pDMAx->S7PAR |= (pDMAHanlde->pDMAConfig.DestinationAddress << 0);
#endif
	//Configure the number of transaction
	pDMAHandle->pDMAStreamx->NDTR |= (pDMAHandle->pDMAConfig.NumberOfTransaction << 0);
	//pDMAHanlde->pDMAx->S7NDTR |= (pDMAHanlde->pDMAConfig.NumberOfTransaction << 0);
	//Configure the FIFO configuration register
	pDMAHandle->pDMAStreamx->FCR = 0;
	//pDMAHanlde->pDMAx->S7FCR = 0;

	//Enable direct mode interrupt
	//1.TCIE - Transfer complete interrupt enable
	//2. DMEIE - Direct mode interrupt enable
	pDMAHandle->pDMAStreamx->CR |= 0x10;//0x1E;
	//pDMAHanlde->pDMAx->S7CR |= 0x1E;
}

void DMAEnable(DMA_TypeDef *pDMAx,uint8_t ENOrDi)
{
	if(ENOrDi == ENABLE)
	{
		if(pDMAx == DMA1)
		{
			//DMA1_PCLK_EN();
			RCC->AHB1ENR |= (1<<21);
		}
		if(pDMAx == DMA2)
		{
			//DMA2_PCLK_EN();
			RCC->AHB1ENR |= (1<<22);
		}
	}else
	{
		if(pDMAx == DMA1)
		{
			//DMA1_PCLK_DI();
			RCC->AHB1ENR &= ~(1<<21);
		}
		if(pDMAx == DMA2)
		{
			//DMA2_PCLK_DI();
			RCC->AHB1ENR &= ~(1<<22);
		}
	}
}

void DMAPeripheralEnable(DMA_Stream_TypeDef *pDMAStreamx,uint8_t ENOrDi)
{
	if(ENOrDi == ENABLE)
	{
		pDMAStreamx->CR |= (1<<0);
	}else
	{
		pDMAStreamx->CR &= ~(1<<0);
	}
}

void DMA_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/*
	 * Processor side configuration.
	 * Refer CORTEX M4 generic user guide.
	 * STM32F4 family has up to 82 IRQ numbers thus limited up to ISER2
	 */
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0
			*NVIC_ISER0 |= (1<< IRQNumber); //Inerrupt set-enable register
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			// program ISER1
			*NVIC_ISER1 |= (1<< (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96)
		{
			// program ISER2
			*NVIC_ISER2 |= (1<< (IRQNumber % 64));
		}
	}
	else//CLEAR THE BIT / DISABLE THEM
	{
		if(IRQNumber <= 31)
		{
			// program ISER0
			*NVIC_ICER0 |= (1<< IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			// program ISER1
			*NVIC_ICER1 |= (1<< (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber <96)
		{
			// program ISER2
			*NVIC_ICER1 |= (1<< (IRQNumber % 64));
		}
	}
}

void DMAInterruptHandle(DMA_Handle_t *pDMAHandle)
{
	uint32_t DMA_LISR_reg;
	uint32_t DMA_HISR_reg;

#if 0
	pDMAHandle->pDMAx->LIFCR = 0xffffffff;
	pDMAHandle->pDMAx->HIFCR = 0xffffffff;
#endif

#if 1
	/*Get the register data from DMA low interrupt flag clear register*/
	DMA_LISR_reg = pDMAHandle->pDMAx->LISR;

	/*Get the register data from DMA high interrupt flag clear register*/
	DMA_HISR_reg = pDMAHandle->pDMAx->HISR;

	//UART 6 RX
	if(DMA_LISR_reg & DMA_LISR_TCIF2)
	{
		/*Reset the Stream 2 Transfer complete flag*/
		pDMAHandle->pDMAx->LIFCR |= DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
	}

	//UART 1 RX
	if(DMA_HISR_reg & DMA_HISR_TCIF5)
	{
		/*Reset the Stream 6 Transfer complete flag*/
		pDMAHandle->pDMAx->HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
	}

	//UART 6 TX
	if(DMA_HISR_reg & DMA_HISR_TCIF6)
	{
		/*Reset the Stream 6 Transfer complete flag*/
		pDMAHandle->pDMAx->HIFCR |= DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
	}

	//UART 1 TX
	if(DMA_HISR_reg & DMA_HISR_TCIF7)
	{
		/*Reset the Stream 6 Transfer complete flag*/
		pDMAHandle->pDMAx->HIFCR |= DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
	}

#endif
}

