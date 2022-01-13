/*
 * ADC_12138.c
 *
 *  Created on: Apr 27, 2020
 *      Author: ppingale
 */

#include "stm32f4xx_hal.h"
#include "main.h"


//uint16_t ADCbuff[100], dataptr = 0;

SPI_HandleTypeDef hspi3;


uint16_t CardReadADC(uint8_t CardType,uint16_t CHNum);
void CardADCInit(uint8_t CardType);
uint16_t CArdSPIReadByte(uint16_t TxData,uint8_t CardID);

uint16_t ReadADC(uint16_t CHNum);
void ADCInit(void);
uint16_t SPIReadByte(uint16_t TxData);
uint16_t dummy = 0;
//void MX_SPI3_Init(void);

/*void MX_SPI3_Init(void)
{

  // USER CODE BEGIN SPI3_Init 0

  // USER CODE END SPI3_Init 0

  // USER CODE BEGIN SPI3_Init 1

  // USER CODE END SPI3_Init 1
  // SPI3 parameter configuration
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    //Error_Handler();
  }
  // USER CODE BEGIN SPI3_Init 2 //

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_3, GPIO_PIN_SET); // chip select high
  HAL_Delay(10);
  // USER CODE END SPI3_Init 2 //

}*/

uint16_t SPIReadByte(uint16_t TxData)
{
	uint16_t err = 0;

	while((hspi3.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE)
	{
		(err < 1000)?(err++):(err = 2000);
		if(err == 2000)
			break;
	}

	// Select the chip
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13/*|GPIO_PIN_3*/, GPIO_PIN_RESET); // chip select low
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); // chip select low
	hspi3.Instance->DR = TxData;
	err = 0;
	while((hspi3.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
	{
		(err < 1000)?(err++):(err = 2000);
		if(err == 2000)
			break;
	}
	dummy = hspi3.Instance->DR;
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13/*|GPIO_PIN_3*/, GPIO_PIN_SET); // chip select high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET); // chip select high


	return dummy;
}

uint16_t ReadADC(uint16_t CHNum)
{
	uint16_t dummyAdCnt;
	 SPIReadByte(CHNum); // conversion
	 //HAL_Delay(100);
	 dummyAdCnt =  SPIReadByte(0x0C00); // read status
	 //HAL_Delay(100);
	 return dummyAdCnt;
}

void ADCInit(void)
{
	uint16_t err = 0;

	if ((hspi3.Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
	// Enable SPI peripheral
	 hspi3.Instance->CR1 |= SPI_CR1_SPE;
	}
	SPIReadByte(0x0008);// Auto cal
	HAL_Delay(1);
	SPIReadByte(0x0C00);// read status
	HAL_Delay(1);
	dummy = SPIReadByte(0x0C00);// read status
	HAL_Delay(1);
	// check if auto calibration is complete by cal bit in (bit 6) status register
	while((dummy & 0x2000) == 0x2000)
	{
		(err < 1000)?(err++):(err = 2000);
		if(err == 2000)
			break;
	}

	SPIReadByte(0xF100); // conversion
	HAL_Delay(100);
	SPIReadByte(0x0D00);// conversion without sign
	HAL_Delay(1);

}
//=====================================================================================
uint16_t CardSPIReadByte(uint16_t TxData,uint8_t CardID)
{
	uint16_t err = 0;

	while((hspi3.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE)
	{
		(err < 1000)?(err++):(err = 2000);
		if(err == 2000)
			break;
	}

	// Select the chip

	switch(CardID)// for Select chip-select
	{
	case MPH:
		break;

	case MVUX:
		// selest the chip CS of SPI for MVUX card
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13/*|GPIO_PIN_3*/, GPIO_PIN_RESET); // chip select low
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); // chip select low

		break;

	case MIL:
		break;

	case MI420:
		break;

	case MO420:
		break;
	default:
		break;


	}

	hspi3.Instance->DR = TxData;
	err = 0;
	while((hspi3.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
	{
		(err < 1000)?(err++):(err = 2000);
		if(err == 2000)
			break;
	}
	dummy = hspi3.Instance->DR;

	switch(CardID)// for deselect chip-select
	{
	case MPH:
		break;

	case MVUX:
		// selest the chip CS of SPI for MVUX card
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13/*|GPIO_PIN_3*/, GPIO_PIN_SET); // chip select high
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET); // chip select high

		break;

	case MIL:
		break;

	case MI420:
		break;

	case MO420:
		break;
	default:
		break;


	}


	return dummy;
}
//=====================================================================================
void CardADCInit(uint8_t CardType)
{
	uint16_t err = 0;

	if ((hspi3.Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
	// Enable SPI peripheral
	 hspi3.Instance->CR1 |= SPI_CR1_SPE;
	}
	CardSPIReadByte(0x0008, CardType);// Auto cal
	HAL_Delay(1);
	CardSPIReadByte(0x0C00, CardType);// read status
	HAL_Delay(1);
	dummy = SPIReadByte(0x0C00);// read status
	HAL_Delay(1);
	// check if auto calibration is complete by cal bit in (bit 6) status register
	while((dummy & 0x2000) == 0x2000)
	{
		(err < 1000)?(err++):(err = 2000);
		if(err == 2000)
			break;
	}

	CardSPIReadByte(0xF100, CardType); // conversion
	HAL_Delay(100);
	CardSPIReadByte(0x0D00, CardType);// conversion without sign
	HAL_Delay(1);

}

//=====================================================================================
uint16_t CardReadADC(uint8_t CardType,uint16_t CHNum)
{
	uint16_t dummyAdCnt;
	CardSPIReadByte(CHNum, CardType); // conversion
	//HAL_Delay(100);
	dummyAdCnt =  CardSPIReadByte(0x0C00, CardType); // read status
	//HAL_Delay(100);
	return dummyAdCnt;
}
//=====================================================================================
