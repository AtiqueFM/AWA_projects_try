/*
 * AD7682.H
 *
 *  Created on: Sep 24, 2021
 *      Author: atiqueshaikh
 */

#ifndef INC_AD7682_H_
#define INC_AD7682_H_

#include "main.h"

uint16_t SPI_ReadData(void);
void SPI_SendData(void);
void AD7682Init(uint8_t channel);

//SPI register based function
void SPI_reg_Transmit(uint16_t *pData);
void SPI_reg_Receive(uint16_t *pData);
uint8_t getStatus(uint8_t bit);

void SPI_SPE_bit(uint8_t ENorDI);

typedef union{
	uint16_t RegisterData;
	uint8_t ByteData[2];
	struct{
		unsigned RESERVED	: 2;
		unsigned RB			: 1;
		unsigned SEQ		: 2;
		unsigned REF		: 3;
		unsigned BW			: 1;
		unsigned INx		: 3;
		unsigned INCC		: 3;
		unsigned CFG		: 1;
//		unsigned RESERVED	: 2;
	};
}AD7682_ConfigHanldle_t;
AD7682_ConfigHanldle_t AD7682_Config_t;

#endif /* INC_AD7682_H_ */
