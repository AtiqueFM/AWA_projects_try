/*
 * ADS1115.h
 *
 *  Created on: Jun 25, 2021
 *      Author: atiqueshaikh
 */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

#include "main.h"

#define ADS1115_SLAVE_ADDRESS ADS_SLAVE_ADDRESS_GND

/*ADS1115 I2C slave address depending on ADDR pin*/
#define ADS_SLAVE_ADDRESS_GND	0b1001000
#define ADS_SLAVE_ADDRESS_VDD	0b1001001
#define ADS_SLAVE_ADDRESS_SDA	0b1001010
#define ADS_SLAVE_ADDRESS_SCL	0b1001011

/*ADS1115 register address*/
#define ADS_REGISTER_CONVERSION	0
#define ADS_REGISTER_CONFIG		1
#define ADS_REGISTER_LO_THRESH	2
#define ADS_REGISTER_HI_THRESH	3

/*MODE bit configuration*/
#define ADS_MODE_SINGLE			1
#define ADS_MODE_CONT			0

/*DR (Data Rate) bit(s) configuration*/
#define ADS_DR_8SPS				0
#define ADS_DR_16SPS			1
#define ADS_DR_32SPS			2
#define ADS_DR_64SPS			3
#define ADS_DR_128SPS			4 /*Default Value*/
#define ADS_DR_250SPS			5
#define ADS_DR_475SPS			6
#define ADS_DR_860SPS			7

/*MUX (Multiplexer) bit(s) configuration - Single Ended*/
#define ADS_MUX_CHANNEL_AINP0_AINN1	0 /*pH*/
#define ADS_MUX_CHANNEL_AINP0_AINN2	1
#define ADS_MUX_CHANNEL_AINP1_AINN3	2
#define ADS_MUX_CHANNEL_AINP2_AINN3	3
#define ADS_MUX_CHANNEL_AIN0		4
#define ADS_MUX_CHANNEL_AIN1		5
#define ADS_MUX_CHANNEL_AIN2		6/*Temperature*/
#define ADS_MUX_CHANNEL_AIN3		7

/*COMP_POL (Comparator Polarity) bit configuration
 *Determines the ALERT/RDY polarity*/
#define ADS_COMP_POL_ACTIVELOW	0 /*Default Value*/
#define ADS_COMP_POL_ACTIVEHIGH	1 /*Prefered*/

/*Programmable Gate Amplifier bit(s)*/
#define ADS_PGA_6_144			0
#define ADS_PGA_4_096			1
#define ADS_PGA_2_048			2 /*Default Value*/
#define ADS_PGA_1_024			3
#define ADS_PGA_0_512			4
#define ADS_PGA_0_256			5

/* Start Conversation*/
#define ADS_OS_START_CONV		1

/* Compare Queue */
#define ADS_COMP_QUE_1			0
#define ADS_COMP_QUE_2			1
#define ADS_COMP_QUE_4			2
#define ADS_COMP_QUE_DIS		3 /*Default Value*/
/*
 * Functions for initialization and ADC read
 */
void ADS1115_OperationInit(void);
void ADS1115_OperationInit_Temperature(void);
void ADS1115_OperationStartConversion(void);
uint16_t ADS1115_OperationADCRead(void);
void ADS1115_OperationADCWrite(uint8_t *pData,uint8_t len);
//uint16_t ADS1115_OperationpHMeasurement(void);
uint16_t ADS1115_OperationpHMeasurement(uint8_t measure);

uint16_t ADS1115_OperationReadConfiguration(void);

/*
 * Configuration register bit(s)
 */
typedef union{
	uint16_t WordData;
	uint8_t ByteData[2];
	struct
	{
		uint8_t COMP_QUE	: 2;	//BIT0
		uint8_t COMP_LAT	: 1;
		uint8_t COP_POL		: 1;
		uint8_t COMP_MODE 	: 1;
		uint8_t DR	 		: 3;
		uint8_t MODE 		: 1;
		uint8_t PGA  		: 3;
		uint8_t MUX  		: 3;
		uint8_t OS 	 		: 1;	//BIT15
	};
}ADS1115_Config;

#endif /* INC_ADS1115_H_ */
