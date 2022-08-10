/*
 * AD7682.c
 *
 *  Created on: Sep 24, 2021
 *      Author: atiqueshaikh
 */

#include "AD7682.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;
uint8_t codSlotNO = CARD_SLOT_1;
void SPI_SendData(void)
{
//	SS_LOW();
	CARD_1_SPI_SS_OFF();
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&AD7682_Config_t.ByteData[0], 1, 1000);
//	SS_HIGH();
	CARD_1_SPI_SS_ON();
}
void AD7682Init(uint8_t channel)
{
	AD7682_Config_t.CFG = 0x01;//overwrite configuration
	AD7682_Config_t.INCC = 0x07;// Single ended INx to ground
	AD7682_Config_t.INx = channel;//0x01;//Channel 1
	AD7682_Config_t.BW = 0x01;//Full BandWidth
	AD7682_Config_t.REF = 0x01;//vref = 4.096V
	AD7682_Config_t.SEQ = 0x00;//Disable
	AD7682_Config_t.RB = 0x01;// Dont read the configuration back
	AD7682_Config_t.RESERVED = 0x00;

	AD7682_ConfigHanldle_t read_st;
	uint16_t data_ = 0;
	uint16_t data = 0;
	memset(&read_st,'\0',sizeof(AD7682_ConfigHanldle_t));

	COD_SPI_SS_OFF(codSlotNO);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&AD7682_Config_t.ByteData[0], (uint8_t*)&data, 1, 1000);
	//SPI_reg_Transmit((uint16_t*)&AD7682_Config_t.RegisterData);

	COD_SPI_SS_ON(codSlotNO);
	//for(int h = 0;h<40;h++);

}

uint16_t SPI_ReadData(void)
{
	uint16_t data[200];
	uint16_t data_ = 0;
	uint64_t avgData = 0;
	uint8_t samples = 80;
	//memset(&data,'\0',sizeof(uint16_t)*100);
	//AD7682_Config_t.CFG = 0x00;//dont overwrite configuration
//	SS_HIGH();
//	SS_LOW();
	SPI_SPE_bit(SET);
	//CARD_7_SEL1_ON();
	GPIOC->ODR |= (1<<14);
	for(int i = 0;i<samples;i++)
	{
		//COD_SPI_SS_OFF(codSlotNO);
		GPIOF->ODR ^= (1<<12);
		//HAL_SPI_TransmitReceive(&hspi3,(uint8_t*)&data_ , (uint8_t*)&data[i], 1, 1000);
		SPI_reg_Receive((uint16_t*)&data[i]);
		//COD_SPI_SS_ON(codSlotNO);
		GPIOF->ODR |= (1<<12);
		for(int g = 0;g<15;g++);//4us delay
	}
	SPI_SPE_bit(RESET);
	//CARD_7_SEL1_OFF();
	GPIOC->ODR ^= (1<<14);
	for(int i = 1;i<samples;i++)//Skip the first , dummy read
	{
		avgData += data[i];
	}
	avgData = avgData/(samples-1); //Skip the first , dummy read
	return ((uint16_t)avgData);
}

void SPI_reg_Transmit(uint16_t *pData)
{
	//dummy read
	uint16_t Dummy_buff = 0;

	//Enable the SPI
	SPI_SPE_bit(SET);
	//get the status flag from the status register TXE (BIT 1)
	//uint32_t TXE_status = hspi3.Instance->SR & (1 << 1);
	//uint8_t TXE_status = getStatus(1);

	//wait while the transmission is over
	while(!getStatus(1));

	//start the transmission
	hspi3.Instance->DR = *pData;

	//get the flag from status register for RXNE (BIT 0)
//	uint32_t RXNE_status = getStatus(0);

	//wait for the data to be received from the slave
	while(!getStatus(0));

	//get the received data
	Dummy_buff = hspi3.Instance->DR;

	SPI_SPE_bit(RESET);
}

void SPI_reg_Receive(uint16_t *pData)
{
	//dummy read
	uint16_t Dummy_buff = 0;

	//get the status flag from the status register TXE (BIT 1)
	//uint32_t TXE_status = hspi3.Instance->SR & (1 << 1);
	//uint8_t TXE_status = getStatus(1);

	//wait while the transmission is over
	while(!getStatus(1));

	//start the transmission
	hspi3.Instance->DR = Dummy_buff;

	//get the flag from status register for RXNE (BIT 0)
	//uint32_t RXNE_status = hspi3.Instance->SR & (1 << 0);
	//uint32_t RXNE_status = getStatus(0);

	//wait for the data to be received from the slave
	while(!getStatus(0));

	//get the received data
	*pData = hspi3.Instance->DR;
}

uint8_t getStatus(uint8_t bit)
{
	uint32_t status = hspi3.Instance->SR & (1 << bit);

	if(status)
		return 1;
	else
		return 0;
}

void SPI_SPE_bit(uint8_t ENorDI)
{
	if(ENorDI)
		hspi3.Instance->CR1 |= (1<<6);
	else
		hspi3.Instance->CR1 ^= (1<<6);
}
