/*
 * ADS1115.c
 *
 *  Created on: Jun 25, 2021
 *      Author: atiqueshaikh
 */


#include "ADS1115.h"

extern I2C_HandleTypeDef hi2c1;
ADS1115_Config ADS1115_Config_t;
ADS1115_Config ADS1115_ReadConfigutationHandle_t;

void ADS1115_OperationInit(void)
{
	/*Configuration register data
	 * 1. Single ended configuration
	 * 2. Single Shot ADC conversion
	 * 3. Data rate 860 SPS
	 * 4. PGA to +- 2.048V
	 * 5. Comparator Polarity to Active Low
	 * 6. Channel AIN0 selected*/
	ADS1115_Config_t.DR = ADS_DR_250SPS;
	ADS1115_Config_t.MUX = ADS_MUX_CHANNEL_AINP0_AINN1;
	ADS1115_Config_t.MODE = ADS_MODE_SINGLE;
	ADS1115_Config_t.COP_POL = ADS_COMP_POL_ACTIVEHIGH;
	ADS1115_Config_t.PGA = ADS_PGA_2_048;
	ADS1115_Config_t.COMP_QUE = ADS_COMP_QUE_DIS;

#if 0
	uint8_t Data_Tx[3];
	Data_Tx[0] = ADS_REGISTER_CONFIG;
	Data_Tx[1] = ADS1115_Config_t.ByteData[1];
	Data_Tx[2] = ADS1115_Config_t.ByteData[0];
	ADS1115_OperationADCWrite((uint8_t*)&Data_Tx,3);


	/*ALERT/RDY pin configuration in HI_THRESH and LOW_THRESH register
	 * To use ALERT/RDY pin as Conversion ready pin*/
	Data_Tx[0] = ADS_REGISTER_HI_THRESH;
	Data_Tx[1] = 0x80;
	Data_Tx[2] = 0x00;
	ADS1115_OperationADCWrite((uint8_t*)&Data_Tx,3);

	Data_Tx[0] = ADS_REGISTER_LO_THRESH;
	Data_Tx[1] = 0x00;
	Data_Tx[2] = 0x00;
//	HAL_I2C_Master_Transmit(&hi2c1, ADS_SLAVE_ADDRESS_VDD,(uint8_t*)&Data_Tx, 3, 1000);
	ADS1115_OperationADCWrite((uint8_t*)&Data_Tx,3);

#endif
}

void ADS1115_OperationInit_Temperature(void)
{
	memset(&ADS1115_Config_t,'\0',sizeof(ADS1115_Config_t));
	/*Configuration register data
	 * 1. Single ended configuration
	 * 2. Single Shot ADC conversion
	 * 3. Data rate 250 SPS
	 * 4. PGA to +- 2.048V
	 * 5. Comparator Polarity to Active Low
	 * 6. Channel AIN2 selected*/
	ADS1115_Config_t.DR = ADS_DR_250SPS;
	ADS1115_Config_t.MUX = ADS_MUX_CHANNEL_AIN2;
	ADS1115_Config_t.MODE = ADS_MODE_SINGLE;
	ADS1115_Config_t.COP_POL = ADS_COMP_POL_ACTIVEHIGH;
	ADS1115_Config_t.PGA = ADS_PGA_2_048;
	ADS1115_Config_t.COMP_QUE = ADS_COMP_QUE_DIS;
}

void ADS1115_OperationStartConversion(void)
{
	ADS1115_Config StartConveration_config;
	uint8_t Data_Tx[3];
	StartConveration_config = ADS1115_Config_t;
	StartConveration_config.OS = ADS_OS_START_CONV;
	Data_Tx[0] = ADS_REGISTER_CONFIG;
	Data_Tx[1] = StartConveration_config.ByteData[1];
	Data_Tx[2] = StartConveration_config.ByteData[0];
	ADS1115_OperationADCWrite((uint8_t*)&Data_Tx,3);
}
uint16_t ADS1115_OperationADCRead(void)
{
	uint8_t Data_Tx;
	union{
		uint8_t byte[2];
		uint16_t Word;
	}ADC_Data;
	Data_Tx = ADS_REGISTER_CONVERSION;
	ADS1115_OperationADCWrite((uint8_t*)&Data_Tx,1);
	uint8_t address = (ADS1115_SLAVE_ADDRESS << 1) | 0x01;
	HAL_I2C_Master_Receive(&hi2c1, address, (uint8_t*)&ADC_Data.Word, 2, 1000);
	return ADC_Data.Word;

}
void ADS1115_OperationADCWrite(uint8_t *pData,uint8_t len)
{
	uint8_t address = (ADS1115_SLAVE_ADDRESS << 1);
	HAL_I2C_Master_Transmit(&hi2c1, address,pData, len, 1000);
}

void ADS1115_OperationADCconversionregister()
{
	uint8_t address = (ADS1115_SLAVE_ADDRESS << 1);
	HAL_I2C_Master_Transmit(&hi2c1, address,ADS_REGISTER_CONVERSION, 1, 1000);
}
#if 0
/*1/9/2021*/
uint16_t ADS1115_OperationpHMeasurement(void)
{
	uint16_t sample_no = 50;
//	AWAOperationStatus_t.FactoryMode = 0x01;//change to setting mode
	uint64_t adc_temp = 0;
	uint16_t adc_val = 0;
	union{
		uint8_t raw_data[2];
		uint16_t data;
	}raw_adc;
	ADS1115_OperationInit();
	for(int i = 0;i<sample_no;i++)
	{
		ADS1115_OperationStartConversion();
		HAL_Delay(2);
		adc_val = ADS1115_OperationADCRead();
		raw_adc.raw_data[0] = (adc_val >> 8);
		raw_adc.raw_data[1] = adc_val;
		adc_temp += raw_adc.data;
	}
	return ((float)adc_temp / (float)sample_no);
}
#endif
/*2/9/2021*/
/*
 * BUG :- 0 apearing while averaging (SOLVED) // 6/9/2021
 */
uint16_t ADS1115_OperationpHMeasurement(uint8_t measure)
{
	uint16_t sample_no = 50;
	uint16_t count = 0;
	uint64_t adc_temp = 0;
	uint16_t adc_val = 0;
	uint16_t arr[50];
	uint16_t avg_val = 0;
	union{
		uint8_t raw_data[2];
		uint16_t data;
	}raw_adc;
	if(measure == 0x01)
		ADS1115_OperationInit();
	else if(measure == 0x02)
		ADS1115_OperationInit_Temperature();
	for(int i = 0;i<sample_no;i++)
	{
		ADS1115_OperationStartConversion();
		HAL_Delay(2);
		adc_val = ADS1115_OperationADCRead();
//		HAL_Delay(2);
		raw_adc.raw_data[0] = (adc_val >> 8);
		raw_adc.raw_data[1] = adc_val;
		if(raw_adc.data != 0)
		{
			arr[count] = raw_adc.data;/*Change Yet to test*/
//			adc_temp += raw_adc.data;
			count += 1;
		}
	}
	for(int i = 1;i<count;i++)
	{
		adc_temp +=arr[i]/(count-1);
		//avg_val +=arr[i]/(count-1);
	}

	return adc_temp;//((float)adc_temp / (float)count);
}

uint16_t ADS1115_OperationReadConfiguration(void)
{
	uint8_t Data_Tx;
	union{
		uint8_t byte[2];
		uint16_t Word;
	}ADC_Data;
	Data_Tx = ADS_REGISTER_CONFIG;
	ADS1115_OperationADCWrite((uint8_t*)&Data_Tx,1);
	uint8_t address = (ADS1115_SLAVE_ADDRESS << 1) | 0x01;
	HAL_I2C_Master_Receive(&hi2c1, address, (uint8_t*)&ADC_Data.Word, 2, 1000);
	return ADC_Data.Word;
}
