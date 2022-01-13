/*
 * FM24CL64B.c
 *
 *  Created on: Jun 17, 2021
 *      Author: atiqueshaikh
 */

#include "FM24CL64B.h"

extern I2C_HandleTypeDef hi2c1;

void FRAM_OperationWrite(uint16_t u16_address,uint8_t *u8_pData,uint16_t u16_noByte)
{
	uint8_t address_low = (uint8_t)u16_address;
	uint8_t address_high = (uint8_t)(u16_address >> 8);
//	uint8_t *pData = NULL;
//	pData = (uint8_t*)calloc(sizeof(uint8_t),u16_noByte + 2);
//	*(pData++) = address_high;
//	*(pData++) = address_low;
	uint8_t pData[u16_noByte+2];
	pData[0] = address_high;
	pData[1] = address_low;
	for(uint16_t i = 0; i < u16_noByte; i++)
	{
		pData[i+2] = u8_pData[i];
	}
	HAL_I2C_Master_Transmit(&hi2c1, FRAM_SLAVE_ADDRESS, (uint8_t*)&pData, u16_noByte + 2, 1000);
//	free(pData);
}

void FRAM_OperationRead(uint16_t u16_address,uint8_t *u8_pData,uint16_t u16_noByte)
{
	uint8_t address_low = (uint8_t)u16_address;
	uint8_t address_high = (uint8_t)(u16_address >> 8);
	uint8_t address_len = 2;
	uint8_t pData_addr[2];
	pData_addr[0] = address_high;
	pData_addr[1] = address_low;
	HAL_I2C_Master_Transmit(&hi2c1, FRAM_SLAVE_ADDRESS, (uint8_t*)&pData_addr, address_len, 1000);
//	HAL_Delay(100);
	HAL_I2C_Master_Receive(&hi2c1, FRAM_SLAVE_ADDRESS, (uint8_t*)u8_pData,u16_noByte, 1000);
}

