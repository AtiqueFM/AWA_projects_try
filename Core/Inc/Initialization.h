/*
 * Initialization.h
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */

#ifndef INC_INITIALIZATION_H_
#define INC_INITIALIZATION_H_

#include <stdlib.h>
#include <stdint.h>
#include "Application.h"

#define DMA_FIRST_TRANSACTION_NO						( ( uint16_t ) 6 )
#define DMA_SECOND_HOLDINGREAD_TRANSACTION_NO			( ( uint16_t ) 2 )
#define DMA_SECOND_INPUTREAD_TRANSACTION_NO				( ( uint16_t ) 2 )
#define DMA_SECOND_SINGLEPRESETHOLDING_TRANSACTION_NO	( ( uint16_t ) 2 )
#define DMA_SECOND_INPUTCOILREAD_TRANSACTION_NO			( ( uint16_t ) 2 )
#define DMA_SECOND_SINGLEPRESETINPUTCOIL_TRANSACTION_NO	( ( uint16_t ) 2 )
#define DMA_SECOND_MULTIPRESETINPUTCOIL_TRANSACTION_NO	( ( uint16_t ) 2 )
#define DMA_SECOND_STATUSCOILREAD_TRANSACTION_NO		( ( uint16_t ) 2 )


#define MODBUS_FUNCODE_READ_HOLDING				( ( uint8_t ) 0x03 )
#define MODBUS_FUNCODE_READ_INPUT				( ( uint8_t ) 0x04 )
#define MODBUS_FUNCODE_READ_HOLDING				( ( uint8_t ) 0x03 )
#define MODBUS_FUNCODE_SINGLE_PRESET_HOLDING	( ( uint8_t ) 0x06 )
#define MODBUS_FUNCODE_MULTI_PRESET_HOLDING		( ( uint8_t ) 0x10 )
#define MODBUS_FUNCODE_READ_INPUTCOIL			( ( uint8_t ) 0x01 )
#define MODBUS_FUNCODE_SINGLE_PRESET_INPUTCOIL	( ( uint8_t ) 0x05 )
#define MODBUS_FUNCODE_MULTI_PRESET_INPUTCOIL	( ( uint8_t ) 0x0F )
#define MODBUS_FUNCODE_READ_STATUSCOIL			( ( uint8_t ) 0x02 )


void ReadSlotID(void);

void PWM_setup(void);

void HoldingRegisterdefaultData(void);

/*Initialization of UART6 Reception*/
void DMA_UART6_RX_Init(void);

/*Initialization of UART6 Transmission*/
void DMA_UART6_TX_Init(void);

/*Initialization of UART1 Reception*/
void DMA_UART1_RX_Init_(void);

/*Initialization of UART1 Reception with variable transactions*/
void DMA_UART1_RX_Init(uint32_t *pDestinationBuff, uint16_t noTrasaction);

/*Initialization of UART1 Transmission*/
void DMA_UART1_TX_Init(void);

/*Internal ADC read*/
uint16_t InternalADCRead(void);

/*Internal ADC start conversion*/
void InternalADCStartConversion(void);

/*Default calibration values*/
void CalibrationDefaultValue(uint8_t AnalyzerRange);

/*RTU Data*/
void setRTUData(void);

/*Initialization on MODBUS port*/
void MODBUS_Port_Initialization(void);
#endif /* INC_INITIALIZATION_H_ */
