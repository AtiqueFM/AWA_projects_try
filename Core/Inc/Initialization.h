/*
 * Initialization.h
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */

#ifndef INC_INITIALIZATION_H_
#define INC_INITIALIZATION_H_

void ReadSlotID(void);

void PWM_setup(void);

void HoldingRegisterdefaultData(void);

/*Initialization of UART6 Reception*/
void DMA_UART6_RX_Init(void);

/*Initialization of UART6 Transmission*/
void DMA_UART6_TX_Init(void);

/*Initialization of UART1 Reception*/
void DMA_UART1_RX_Init(void);

/*Initialization of UART1 Transmission*/
void DMA_UART1_TX_Init(void);

#endif /* INC_INITIALIZATION_H_ */
