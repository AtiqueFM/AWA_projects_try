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

#define DMA_FIRST_TRANSACTION_NO						( ( uint16_t ) 6 )
#define DMA_SECOND_HOLDINGREAD_TRANSACTION_NO			( ( uint16_t ) 2 )
#define DMA_SECOND_INPUTREAD_TRANSACTION_NO				( ( uint16_t ) 2 )
#define DMA_SECOND_SINGLEPRESETHOLDING_TRANSACTION_NO	( ( uint16_t ) 2 )
#define DMA_SECOND_INPUTCOILREAD_TRANSACTION_NO			( ( uint16_t ) 2 )
#define DMA_SECOND_SINGLEPRESETINPUTCOIL_TRANSACTION_NO	( ( uint16_t ) 2 )
#define DMA_SECOND_STATUSCOILREAD_TRANSACTION_NO		( ( uint16_t ) 2 )


#define MODBUS_FUNCODE_READ_HOLDING				( ( uint8_t ) 0x3 )
#define MODBUS_FUNCODE_READ_INPUT				( ( uint8_t ) 0x4 )
#define MODBUS_FUNCODE_READ_HOLDING				( ( uint8_t ) 0x3 )
#define MODBUS_FUNCODE_SINGLE_PRESET_HOLDING	( ( uint8_t ) 0x6 )
#define MODBUS_FUNCODE_MULTI_PRESET_HOLDING		( ( uint8_t ) 0x10 )
#define MODBUS_FUNCODE_READ_INPUTCOIL			( ( uint8_t ) 0x1 )
#define MODBUS_FUNCODE_SINGLE_PRESET_INPUTCOIL	( ( uint8_t ) 0x5 )
#define MODBUS_FUNCODE_READ_STATUSCOIL			( ( uint8_t ) 0x2 )

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

#endif /* INC_INITIALIZATION_H_ */
