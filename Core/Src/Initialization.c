/*
 * Initialization.c
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */

#include "Initialization.h"
#include "main.h"
#include "math.h"
#include "AD7682.h"
#include <stdlib.h>
#include "stm32f4xx_dma.h"

TIM_OC_InitTypeDef sConfigOC_PWM1 = {0};//moved
TIM_OC_InitTypeDef sConfigOC_PWM2 = {0};//moved
TIM_OC_InitTypeDef sConfigOC_PWM3 = {0};//moved
TIM_OC_InitTypeDef sConfigOC_PWM4 = {0};//moved

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern float COD_UpperLimit,BOD_UpperLimit,TSS_UpperLimit;

DMA_Handle_t DMA_UART6_RX_handle_t;
DMA_Handle_t DMA_UART6_TX_handle_t;
DMA_Handle_t DMA_UART1_RX_handle_t;
DMA_Handle_t DMA_UART1_TX_handle_t;

void ReadSlotID(void)
{
	//PG[0:1] - ID[0:1],PE7 - ID3
	CardID_Info.CARD_1 = ((GPIOE->IDR & 0x80)>>5) | (GPIOG->IDR & 0x03);//BUG //COD
	//ID[1:3] - PG[5:7]
	CardID_Info.CARD_2 = ((GPIOG->IDR & 0xE0) >> 5);
	//ID[1:2] - PA[11:12], ID3 - PG9
	CardID_Info.CARD_3 = ((GPIOA->IDR & 0xC00) >> 10) | ((GPIOG->IDR & 0x200)>>7);//BUG
	//ID[1:3] - PA[5:7]
	CardID_Info.CARD_5 = (GPIOA->IDR & 0xE0);
	//ID[1:3] - PF[0:2]
	CardID_Info.CARD_6 = (GPIOF->IDR & 0x07); //pH
	//ID[1:3] - PE[3:5]
	CardID_Info.CARD_7 = ((GPIOE->IDR & 0x38) >> 3);
	//ID[1:3] - PE[0:2]
	CardID_Info.CARD_8 = GPIOE->IDR & 0x07;
}

void PWM_setup(void)
{
	sConfigOC_PWM1.OCMode = TIM_OCMODE_PWM1;
	sConfigOC_PWM1.Pulse = 4000;
	sConfigOC_PWM1.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_PWM1.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC_PWM1.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC_PWM1.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC_PWM1.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	sConfigOC_PWM2 = sConfigOC_PWM1;
	sConfigOC_PWM3 = sConfigOC_PWM1;
	sConfigOC_PWM4 = sConfigOC_PWM1;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM1, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC_PWM2.Pulse = 16;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM2, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC_PWM3.Pulse = 46;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM3, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC_PWM4.Pulse = 6;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_PWM4, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
}

void HoldingRegisterdefaultData(void)
{
	//Device will start from RUN->Batch mode
	HoldingRegister_t.ModeCommand_t.ModeCommand_H = 0x22;
	HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x01;
//	HoldingRegister_t.ModeCommand_t.PUMP1_ONTIME = 1;
	//depricated code
	g_u8PUMP_action = 0x01;//pump-on time
	//default value for the pump action, this will start from the pump on and off then it will enter to the de-gas time (pump delay)
	PUMPControlHandle_t.u8PUMP_action = 0x01;
	SEL1_PT100();
	//COD Sensor calibration, should start with capturing x1 and y1
	COD_SensorCalibration_t.point_flag = 0;
	//When the system is rebooted or booted for the first time user is supposed to measure the PD1 and PD2 zero values
	COD_MeasurementStatus.COD_ZERO = 0x00;
	//Set the COD Auto Zero flag as we have default data
	COD_MeasurementStatus.COD_ZERO = 0x01;

	//testing purpose
	InputRegister_t.PV_info.CODValue = 0;//12.99;
	InputRegister_t.PV_info.BODValue = 0;//111.23;
	InputRegister_t.PV_info.TSSValue = 0;//56.88;

}

void DMA_UART6_RX_Init(void)
{
	//UART RX
	//Select DMA 2
	DMA_UART6_RX_handle_t.pDMAx = DMA2;
	//Select Stream 5 for UART6_RX
	DMA_UART6_RX_handle_t.pDMAStreamx = DMA2_Stream2;

	//Configuration
	DMA_UART6_RX_handle_t.pDMAConfig.ChannelSelect = 5;
	DMA_UART6_RX_handle_t.pDMAConfig.DataFlowDirection = 0;//Peripheral to memory
	DMA_UART6_RX_handle_t.pDMAConfig.DirectModeEnable = 1;

	DMA_UART6_RX_handle_t.pDMAConfig.MemoryIncrementEnable = 1;
	DMA_UART6_RX_handle_t.pDMAConfig.NumberOfTransaction = 8;//data
	DMA_UART6_RX_handle_t.pDMAConfig.PeripheralDataSize = 0;//8 bit
	DMA_UART6_RX_handle_t.pDMAConfig.MemoryDataSize = 0;//8 bit
	DMA_UART6_RX_handle_t.pDMAConfig.PeripheralIncrementEnable = 0;
	DMA_UART6_RX_handle_t.pDMAConfig.SelectPriority = 0x11;//priority very high
	DMA_UART6_RX_handle_t.pDMAConfig.StreamSelect = 1;
	//source destination address
	DMA_UART6_RX_handle_t.pDMAConfig.SourceAddress= USART6_BASE + 0x04;//UART6_DR register address
	//Destination destination address
	DMA_UART6_RX_handle_t.pDMAConfig.DestinationAddress = (uint32_t)(&MOD2_Rxbuff);//UART_6_RX_DESTINATION_ADDR;//0x40005410;//SRAM address
	//enable interrupt at its default priority
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 1);
	//DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,ENABLE);//RX
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	DMAEnable(DMA2,ENABLE);//RCC CLOCK
	DMAInit(&DMA_UART6_RX_handle_t);//configuration


}


void DMA_UART6_TX_Init(void)
{
	//UART TX
	//Select DMA 2
	DMA_UART6_TX_handle_t.pDMAx = DMA2;
	//Select Stream 5 for UART6_RX
	DMA_UART6_TX_handle_t.pDMAStreamx = DMA2_Stream6;

	//Configuration
	DMA_UART6_TX_handle_t.pDMAConfig.ChannelSelect = 5;
	DMA_UART6_TX_handle_t.pDMAConfig.DataFlowDirection = 1;//Memory to Peripheral
	DMA_UART6_TX_handle_t.pDMAConfig.DirectModeEnable = 1;

	DMA_UART6_TX_handle_t.pDMAConfig.MemoryIncrementEnable = 1;
	DMA_UART6_TX_handle_t.pDMAConfig.NumberOfTransaction = DMA_Transaction_no_Tx_uart6 + 5;//SET GLOBAL VARIABLE
	DMA_UART6_TX_handle_t.pDMAConfig.PeripheralDataSize = 0;//8 bit
	DMA_UART6_TX_handle_t.pDMAConfig.MemoryDataSize = 0;//8 bit
	DMA_UART6_TX_handle_t.pDMAConfig.PeripheralIncrementEnable = 0;
	DMA_UART6_TX_handle_t.pDMAConfig.SelectPriority = 0x11;//priority very high
	DMA_UART6_TX_handle_t.pDMAConfig.StreamSelect = 6;
	//source destination address
	DMA_UART6_TX_handle_t.pDMAConfig.SourceAddress= (uint32_t)(&MOD2_Txbuff);//USART6_BASE + 0x04;//UART6_DR register address
	//Destination destination address
	DMA_UART6_TX_handle_t.pDMAConfig.DestinationAddress = USART6_BASE + 0x04;//(uint32_t)(&MOD2_Rxbuff);//UART_6_RX_DESTINATION_ADDR;//0x40005410;//SRAM address
	//enable interrupt at its default priority

	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 1);

	//DMA_IRQITConfig(IRQ_NO_DMA2_Stream6,ENABLE);//TX
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

	DMAInit(&DMA_UART6_TX_handle_t);//configuration
}


/*Initialization of UART1 Reception*/
void DMA_UART1_RX_Init_(void)
{
	//UART RX
	//Select DMA 2
	DMA_UART1_RX_handle_t.pDMAx = DMA2;
	//Select Stream 5 for UART6_RX
	DMA_UART1_RX_handle_t.pDMAStreamx = DMA2_Stream5;

	//Configuration
	DMA_UART1_RX_handle_t.pDMAConfig.ChannelSelect = 4;
	DMA_UART1_RX_handle_t.pDMAConfig.DataFlowDirection = 0;//Peripheral to memory
	DMA_UART1_RX_handle_t.pDMAConfig.DirectModeEnable = 1;

	DMA_UART1_RX_handle_t.pDMAConfig.MemoryIncrementEnable = 1;
	DMA_UART1_RX_handle_t.pDMAConfig.NumberOfTransaction = 8; /*<Must be changed according to function code*/
	DMA_UART1_RX_handle_t.pDMAConfig.PeripheralDataSize = 0;//8 bit
	DMA_UART1_RX_handle_t.pDMAConfig.MemoryDataSize = 0;//8 bit
	DMA_UART1_RX_handle_t.pDMAConfig.PeripheralIncrementEnable = 0;
	DMA_UART1_RX_handle_t.pDMAConfig.SelectPriority = 0x11;//priority very high
	DMA_UART1_RX_handle_t.pDMAConfig.StreamSelect = 5;
	//source destination address
	DMA_UART1_RX_handle_t.pDMAConfig.SourceAddress= USART1_BASE + 0x04;//UART6_DR register address
	//Destination destination address
	DMA_UART1_RX_handle_t.pDMAConfig.DestinationAddress = (uint32_t)(&Rxbuff);//UART_6_RX_DESTINATION_ADDR;//0x40005410;//SRAM address
	//enable interrupt at its default priority
	HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
	//DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,ENABLE);//RX
	HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	//DMAEnable(DMA2,ENABLE);//RCC CLOCK
	DMAInit(&DMA_UART1_RX_handle_t);//configuration
}

/*Initialization of UART1 Reception with variable transactions*/
void DMA_UART1_RX_Init(uint32_t *pDestinationBuff, uint16_t noTrasaction)
{
	//UART RX
	//Select DMA 2
	DMA_UART1_RX_handle_t.pDMAx = DMA2;
	//Select Stream 5 for UART6_RX
	DMA_UART1_RX_handle_t.pDMAStreamx = DMA2_Stream5;

	//Configuration
	DMA_UART1_RX_handle_t.pDMAConfig.ChannelSelect = 4;
	DMA_UART1_RX_handle_t.pDMAConfig.DataFlowDirection = 0;//Peripheral to memory
	DMA_UART1_RX_handle_t.pDMAConfig.DirectModeEnable = 1;

	DMA_UART1_RX_handle_t.pDMAConfig.MemoryIncrementEnable = 1;
	DMA_UART1_RX_handle_t.pDMAConfig.NumberOfTransaction = noTrasaction;//8; /*<Must be changed according to function code*/
	DMA_UART1_RX_handle_t.pDMAConfig.PeripheralDataSize = 0;//8 bit
	DMA_UART1_RX_handle_t.pDMAConfig.MemoryDataSize = 0;//8 bit
	DMA_UART1_RX_handle_t.pDMAConfig.PeripheralIncrementEnable = 0;
	DMA_UART1_RX_handle_t.pDMAConfig.SelectPriority = 0x11;//priority very high
	DMA_UART1_RX_handle_t.pDMAConfig.StreamSelect = 5;
	//source destination address
	DMA_UART1_RX_handle_t.pDMAConfig.SourceAddress= USART1_BASE + 0x04;//UART6_DR register address
	//Destination destination address
	DMA_UART1_RX_handle_t.pDMAConfig.DestinationAddress = (uint32_t)pDestinationBuff;//UART_6_RX_DESTINATION_ADDR;//0x40005410;//SRAM address
	//enable interrupt at its default priority
	HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
	//DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,ENABLE);//RX
	HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	//DMAEnable(DMA2,ENABLE);//RCC CLOCK
	DMAInit(&DMA_UART1_RX_handle_t);//configuration
}
/*Initialization of UART1 Transmission*/
void DMA_UART1_TX_Init(void)
{
	//UART TX
	//Select DMA 2
	DMA_UART1_TX_handle_t.pDMAx = DMA2;
	//Select Stream 5 for UART6_RX
	DMA_UART1_TX_handle_t.pDMAStreamx = DMA2_Stream7;

	//Configuration
	DMA_UART1_TX_handle_t.pDMAConfig.ChannelSelect = 4;
	DMA_UART1_TX_handle_t.pDMAConfig.DataFlowDirection = 1;//Memory to Peripheral
	DMA_UART1_TX_handle_t.pDMAConfig.DirectModeEnable = 1;

	DMA_UART1_TX_handle_t.pDMAConfig.MemoryIncrementEnable = 1;
	DMA_UART1_TX_handle_t.pDMAConfig.NumberOfTransaction = DMA_Transaction_no_Tx_uart1;// + 5;//Change the variable
	DMA_UART1_TX_handle_t.pDMAConfig.PeripheralDataSize = 0;//8 bit
	DMA_UART1_TX_handle_t.pDMAConfig.MemoryDataSize = 0;//8 bit
	DMA_UART1_TX_handle_t.pDMAConfig.PeripheralIncrementEnable = 0;
	DMA_UART1_TX_handle_t.pDMAConfig.SelectPriority = 0x11;//priority very high
	DMA_UART1_TX_handle_t.pDMAConfig.StreamSelect = 7;
	//source destination address
	DMA_UART1_TX_handle_t.pDMAConfig.SourceAddress= (uint32_t)(&Txbuff);//USART6_BASE + 0x04;//UART6_DR register address
	//Destination destination address
	DMA_UART1_TX_handle_t.pDMAConfig.DestinationAddress = USART1_BASE + 0x04;//(uint32_t)(&MOD2_Rxbuff);//UART_6_RX_DESTINATION_ADDR;//0x40005410;//SRAM address
	//enable interrupt at its default priority

	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);

	//DMA_IRQITConfig(IRQ_NO_DMA2_Stream6,ENABLE);//TX
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	DMAInit(&DMA_UART1_TX_handle_t);//configuration
}

uint16_t InternalADCRead(void)
{
	uint16_t readingADCcounts = 0;

	/*Poll for the ADC conversion*/
	//HAL_ADC_PollForConversion(&hadc1, 1000);
	/*Get the ADC converted data*/
	readingADCcounts = HAL_ADC_GetValue(&hadc1);
	/*Stop the ADC*/
	//HAL_ADC_Stop(&hadc1);

	return(readingADCcounts);
}

void InternalADCStartConversion(void)
{
	/*Start the ADC*/
	HAL_ADC_Start(&hadc1);
}

///**
//  * @brief  Regular conversion complete callback in non blocking mode
//  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
//  *         the configuration information for the specified ADC.
//  * @retval None
//  */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	uint16_t readingADCcounts = 0;
//
//	/*Get the ADC converted data*/
//	readingADCcounts = HAL_ADC_GetValue(&hadc1);
//
//	/*Start the ADC interrupt*/
//	//HAL_ADC_Stop_IT(hadc);
//
//	/*Start the ADC interrupt*/
//	//HAL_ADC_Start_IT(hadc);
//
//	InputRegister_t.PV_info.TOC = readingADCcounts;
//
//}

void transfer_CODdatatoLiveBuffer(void)
{
	for(int i =0;i<10;i++)
	{
		COD_10ptFactoryCalibrationHandle_t.x[i] = HoldingRegister_t.ModeCommand_t.COD_X[i];
		COD_10ptFactoryCalibrationHandle_t.y[i] = HoldingRegister_t.ModeCommand_t.COD_Y[i];
	}

	for(int i = 0;i<4;i++)
		COD_10ptFactoryCalibrationHandle_t.c[i] = HoldingRegister_t.ModeCommand_t.C[i];

	COD_SensorCalibration_t.x1 = HoldingRegister_t.SensorCalibration_t.COD_X1;
	COD_SensorCalibration_t.x2 = HoldingRegister_t.SensorCalibration_t.COD_X2;
	COD_SensorCalibration_t.y1 = HoldingRegister_t.SensorCalibration_t.COD_Y1;
	COD_SensorCalibration_t.y2 = HoldingRegister_t.SensorCalibration_t.COD_Y2;

	COD_SensorCalibration_t.slope = HoldingRegister_t.SensorCalibration_t.COD_CF;
	COD_SensorCalibration_t.intercept = HoldingRegister_t.SensorCalibration_t.COD_Intercept;

	COD_10ptFactoryCalibrationHandle_t.SF = HoldingRegister_t.ModeCommand_t.COD_SF;
}

void transfer_TSSdatatoLiveBuffer(void)
{

	for(int i = 0;i<10;i++)
	{
		TSS_10ptFactoryCalibrationHandle_t.x[i] = HoldingRegister_t.ModeCommand_t.TSS_F[i];
		TSS_10ptFactoryCalibrationHandle_t.y[i] = HoldingRegister_t.ModeCommand_t.TSS_G[i];
	}

	for(int i = 0;i<3;i++)
		TSS_10ptFactoryCalibrationHandle_t.c[i] = HoldingRegister_t.ModeCommand_t.TSS_K[i];

	TSS_SensorCalibration_t.x1 = HoldingRegister_t.SensorCalibration_t.TSS_X1;
	TSS_SensorCalibration_t.x2 = HoldingRegister_t.SensorCalibration_t.TSS_X2;
	TSS_SensorCalibration_t.y1 = HoldingRegister_t.SensorCalibration_t.TSS_Y1;
	TSS_SensorCalibration_t.y2 = HoldingRegister_t.SensorCalibration_t.TSS_Y2;

	TSS_SensorCalibration_t.slope = HoldingRegister_t.SensorCalibration_t.TSS_CF;
	TSS_SensorCalibration_t.intercept = HoldingRegister_t.SensorCalibration_t.TSS_Intercept;

	TSS_10ptFactoryCalibrationHandle_t.SF = HoldingRegister_t.ModeCommand_t.TSS_SF;
}

void setRelayCofigurationData(uint8_t model)
{
	switch(model)
	{
	case MODEL_3021_3022: // COD	800 mg/l
		//Default data for Alarm output
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Parameter = 1; //COD
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Threshold = 500;//ppm
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Parameter = 1; //COD
		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Threshold = 500;//ppm
		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Parameter = 2; //BOD
		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Threshold = 250;//ppm
		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Parameter = 2; //BOD
		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Threshold = 250;//ppm
		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Parameter = 3; //TSS
		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Threshold = 300;//ppm
		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Parameter = 3; //TSS
		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Threshold = 300;//ppm
		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Parameter = 4; //pH
		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Hysterisis = 20;//ph
		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Threshold = 10;//pH
		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Parameter = 4; //pH
		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Hysterisis = 20;//pH
		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Threshold = 5;//pH
		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2
		break;
	case MODEL_3011_3012: // COD	300 mg/l
		//Default data for Alarm output
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Parameter = 1; //COD
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Threshold = 250;//ppm
		HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Parameter = 1; //COD
		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Threshold = 100;//ppm
		HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Parameter = 2; //BOD
		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Threshold = 125;//ppm
		HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Parameter = 2; //BOD
		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Threshold = 75;//ppm
		HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Parameter = 3; //TSS
		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Threshold = 400;//ppm
		HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Parameter = 3; //TSS
		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Hysterisis = 20;//ppm
		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Threshold = 200;//ppm
		HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Parameter = 4; //pH
		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Hysterisis = 20;//ph
		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Threshold = 10;//pH
		HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Parameter = 4; //pH
		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Hysterisis = 20;//pH
		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Threshold = 5;//pH
		HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2
		break;
	}
}

void setAOCofigurationData(uint8_t model)
{
	switch(model)
	{
	case MODEL_3021_3022: // COD	800 mg/l
		//test
		HoldingRegister_t.IOUTConfig_t[0].Current_OP_option = 1; //cod
		HoldingRegister_t.IOUTConfig_t[0].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[0].Current_OP_MAX_Value = 800.0f;

		HoldingRegister_t.IOUTConfig_t[1].Current_OP_option = 2; //BOD
		HoldingRegister_t.IOUTConfig_t[1].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[1].Current_OP_MAX_Value = 400.0f;

		HoldingRegister_t.IOUTConfig_t[2].Current_OP_option = 3; //TSS
		HoldingRegister_t.IOUTConfig_t[2].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[2].Current_OP_MAX_Value = 750.0f;

		HoldingRegister_t.IOUTConfig_t[3].Current_OP_option = 4; //pH
		HoldingRegister_t.IOUTConfig_t[3].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[3].Current_OP_MAX_Value = 14.0f;
		break;
	case MODEL_3011_3012: // COD	300 mg/l
		HoldingRegister_t.IOUTConfig_t[0].Current_OP_option = 1; //cod
		HoldingRegister_t.IOUTConfig_t[0].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[0].Current_OP_MAX_Value = 300.0f;

		HoldingRegister_t.IOUTConfig_t[1].Current_OP_option = 2; //BOD
		HoldingRegister_t.IOUTConfig_t[1].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[1].Current_OP_MAX_Value = 150.0f;

		HoldingRegister_t.IOUTConfig_t[2].Current_OP_option = 3; //TSS
		HoldingRegister_t.IOUTConfig_t[2].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[2].Current_OP_MAX_Value = 450.0f;

		HoldingRegister_t.IOUTConfig_t[3].Current_OP_option = 4; //pH
		HoldingRegister_t.IOUTConfig_t[3].Current_OP_MIN_Value = 0.0f;
		HoldingRegister_t.IOUTConfig_t[3].Current_OP_MAX_Value = 14.0f;
		break;
	}
}

void CalibrationDefaultValue(uint8_t AnalyzerRange)
{

	switch(AnalyzerRange)
	{
	/*
	 * Range	:
	 * 				COD	:	800 mg/l
	 * 				BOD	:	400 mg/l
	 * 				TSS	:	750 mg/l
	 */
	case MODEL_3021_3022:
		/*COD factory 10 points*/
		HoldingRegister_t.ModeCommand_t.COD_X[0] = 	0.25f;
		HoldingRegister_t.ModeCommand_t.COD_X[1] =	92.93f;
		HoldingRegister_t.ModeCommand_t.COD_X[2] =	168.42f;
		HoldingRegister_t.ModeCommand_t.COD_X[3] =	313.62f;
		HoldingRegister_t.ModeCommand_t.COD_X[4] =	432.86f;
		HoldingRegister_t.ModeCommand_t.COD_X[5] =	549.59f;
		HoldingRegister_t.ModeCommand_t.COD_X[6] =	658.18f;
		HoldingRegister_t.ModeCommand_t.COD_X[7] =	751.88f;
		HoldingRegister_t.ModeCommand_t.COD_X[8] =	844.06f;
		HoldingRegister_t.ModeCommand_t.COD_X[9] =	933.87f;

		HoldingRegister_t.ModeCommand_t.COD_Y[0] =	0.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[1] =	50.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[2] =	100.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[3] =	200.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[4] =	300.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[5] =	400.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[6] =	500.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[7] =	600.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[8] =	700.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[9] =	800.0f;

		/*COD coefficients*/
		HoldingRegister_t.ModeCommand_t.C[0] =	-0.88631f;
		HoldingRegister_t.ModeCommand_t.C[1] =	0.53091f;
		HoldingRegister_t.ModeCommand_t.C[2] =	0.00037f;
		HoldingRegister_t.ModeCommand_t.C[3] =	0.0f;

		/*COD 2 point calibration*/
		HoldingRegister_t.SensorCalibration_t.COD_X1 = 0.0f;
		HoldingRegister_t.SensorCalibration_t.COD_X2 = 200.0f;
		HoldingRegister_t.SensorCalibration_t.COD_Y1 = -0.4f;
		HoldingRegister_t.SensorCalibration_t.COD_Y2 = 199.153f;

		/*COD 2 point calibration data*/
		HoldingRegister_t.SensorCalibration_t.COD_CF = 1.0f;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept = 0.4f;

		/*COD Scaling factors*/
		HoldingRegister_t.ModeCommand_t.COD_SF = 100.0f;

		/*Transfer the data from MODBUS to live buffer*/
		transfer_CODdatatoLiveBuffer();


		/*TSS factory 10 point calibration*/
		HoldingRegister_t.ModeCommand_t.TSS_F[0] =	0.31f;
		HoldingRegister_t.ModeCommand_t.TSS_F[1] =	32.64f;
		HoldingRegister_t.ModeCommand_t.TSS_F[2] =	52.43f;
		HoldingRegister_t.ModeCommand_t.TSS_F[3] =	86.95f;
		HoldingRegister_t.ModeCommand_t.TSS_F[4] =	103.55f;
		HoldingRegister_t.ModeCommand_t.TSS_F[5] =	132.74f;
		HoldingRegister_t.ModeCommand_t.TSS_F[6] =	149.44f;
		HoldingRegister_t.ModeCommand_t.TSS_F[7] =	178.79f;
		HoldingRegister_t.ModeCommand_t.TSS_F[8] =	191.17f;
		HoldingRegister_t.ModeCommand_t.TSS_F[9] =	212.95f;

		HoldingRegister_t.ModeCommand_t.TSS_G[0] =	0.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[1] =	50.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[2] =	100.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[3] =	150.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[4] =	200.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[5] =	250.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[6] =	300.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[7] =	350.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[8] =	400.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[9] =	450.0f;

		/*TSS coefficients*/
		HoldingRegister_t.ModeCommand_t.TSS_K[0] =	-0.36972f;
		HoldingRegister_t.ModeCommand_t.TSS_K[1] =	1.62619f;
		HoldingRegister_t.ModeCommand_t.TSS_K[2] =	0.00226f;

		/*TSS 2 point calibration*/
		HoldingRegister_t.SensorCalibration_t.TSS_X1 = 0.0f;
		HoldingRegister_t.SensorCalibration_t.TSS_X2 = 200.0f;
		HoldingRegister_t.SensorCalibration_t.TSS_Y1 = 1.163f;
		HoldingRegister_t.SensorCalibration_t.TSS_Y2 = 203.622f;

		/*COD 2 point calibration data*/
		HoldingRegister_t.SensorCalibration_t.TSS_CF = -1.15f;
		HoldingRegister_t.SensorCalibration_t.TSS_Intercept = 0.94f;

		/*COD Scaling factors*/
		HoldingRegister_t.ModeCommand_t.TSS_SF = 100.0f;

		/*Load the data into TSS factory calibration live buffer*/
		transfer_TSSdatatoLiveBuffer();

		/*Relay configurations*/
		setRelayCofigurationData(MODEL_3021_3022);

		/*AO configuration*/
		setAOCofigurationData(MODEL_3021_3022);
		break;
	/*
	 * Range	:
	 * 				COD	:	300 mg/l
	 * 				BOD	:	150 mg/l
	 * 				TSS	:	450 mg/l
	 */
	case MODEL_3011_3012:
		/*COD factory 10 points*/
		HoldingRegister_t.ModeCommand_t.COD_X[0] = 	0.16f;
		HoldingRegister_t.ModeCommand_t.COD_X[1] =	48.66f;
		HoldingRegister_t.ModeCommand_t.COD_X[2] =	81.30f;
		HoldingRegister_t.ModeCommand_t.COD_X[3] =	119.71f;
		HoldingRegister_t.ModeCommand_t.COD_X[4] =	150.86f;
		HoldingRegister_t.ModeCommand_t.COD_X[5] =	217.84f;
		HoldingRegister_t.ModeCommand_t.COD_X[6] =	268.59f;
		HoldingRegister_t.ModeCommand_t.COD_X[7] =	291.85f;
		HoldingRegister_t.ModeCommand_t.COD_X[8] =	315.85f;
		HoldingRegister_t.ModeCommand_t.COD_X[9] =	364.45f;

		HoldingRegister_t.ModeCommand_t.COD_Y[0] =	0.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[1] =	25.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[2] =	50.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[3] =	75.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[4] =	100.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[5] =	150.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[6] =	200.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[7] =	225.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[8] =	250.0f;
		HoldingRegister_t.ModeCommand_t.COD_Y[9] =	300.0f;

		/*COD coefficients*/
		HoldingRegister_t.ModeCommand_t.C[0] =	-0.55542f;
		HoldingRegister_t.ModeCommand_t.C[1] =	0.53607f;
		HoldingRegister_t.ModeCommand_t.C[2] =	0.00077f;
		HoldingRegister_t.ModeCommand_t.C[3] =	0.0f;

		/*COD 2 point calibration*/
		HoldingRegister_t.SensorCalibration_t.COD_X1 = 0.0f;
		HoldingRegister_t.SensorCalibration_t.COD_X2 = 200.0f;
		HoldingRegister_t.SensorCalibration_t.COD_Y1 = -0.4f;
		HoldingRegister_t.SensorCalibration_t.COD_Y2 = 199.153f;

		/*COD 2 point calibration data*/
		HoldingRegister_t.SensorCalibration_t.COD_CF = 1.0f;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept = 0.4f;

		/*COD Scaling factors*/
		HoldingRegister_t.ModeCommand_t.COD_SF = 100.0f;

		/*Transfer the data from MODBUS to live buffer*/
		transfer_CODdatatoLiveBuffer();

		/*TSS factory 10 point calibration*/
		HoldingRegister_t.ModeCommand_t.TSS_F[0] =	0.31f;
		HoldingRegister_t.ModeCommand_t.TSS_F[1] =	32.64f;
		HoldingRegister_t.ModeCommand_t.TSS_F[2] =	52.43f;
		HoldingRegister_t.ModeCommand_t.TSS_F[3] =	86.95f;
		HoldingRegister_t.ModeCommand_t.TSS_F[4] =	103.55f;
		HoldingRegister_t.ModeCommand_t.TSS_F[5] =	132.74f;
		HoldingRegister_t.ModeCommand_t.TSS_F[6] =	149.44f;
		HoldingRegister_t.ModeCommand_t.TSS_F[7] =	178.79f;
		HoldingRegister_t.ModeCommand_t.TSS_F[8] =	191.17f;
		HoldingRegister_t.ModeCommand_t.TSS_F[9] =	212.95f;

		HoldingRegister_t.ModeCommand_t.TSS_G[0] =	0.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[1] =	50.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[2] =	100.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[3] =	150.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[4] =	200.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[5] =	250.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[6] =	300.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[7] =	350.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[8] =	400.0f;
		HoldingRegister_t.ModeCommand_t.TSS_G[9] =	450.0f;

		/*TSS coefficients*/
		HoldingRegister_t.ModeCommand_t.TSS_K[0] =	-0.36972f;
		HoldingRegister_t.ModeCommand_t.TSS_K[1] =	1.62619f;
		HoldingRegister_t.ModeCommand_t.TSS_K[2] =	0.00226f;

		/*TSS 2 point calibration*/
		HoldingRegister_t.SensorCalibration_t.TSS_X1 = 0.0f;
		HoldingRegister_t.SensorCalibration_t.TSS_X2 = 200.0f;
		HoldingRegister_t.SensorCalibration_t.TSS_Y1 = 1.163f;
		HoldingRegister_t.SensorCalibration_t.TSS_Y2 = 203.622f;

		/*TSS 2 point calibration data*/
		HoldingRegister_t.SensorCalibration_t.TSS_CF = -1.15f;
		HoldingRegister_t.SensorCalibration_t.TSS_Intercept = 0.94f;

		/*TSS Scaling factors*/
		HoldingRegister_t.ModeCommand_t.TSS_SF = 100.0f;

		/*Load the data into TSS factory calibration live buffer*/
		transfer_TSSdatatoLiveBuffer();

		/*Relay configurations*/
		setRelayCofigurationData(MODEL_3011_3012);

		/*AO configuration*/
		setAOCofigurationData(MODEL_3011_3012);
		break;
	}
}
