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
//	PUMPControlHandle_t.u8PUMP_action = 0x01;
	SEL1_PT100();

#if 0
	HoldingRegister_t.ModeCommand_t.PUMP1_ONTIME = 20;
	HoldingRegister_t.ModeCommand_t.PUMP1_DELAY = 1;
	HoldingRegister_t.ModeCommand_t.PUMP2_ONTIME = 20;
	HoldingRegister_t.ModeCommand_t.PUMP2_DELAY = 1;
#endif

#if 0
	//Testing COD +ve values
	HoldingRegister_t.ModeCommand_t.C[2] = 0.0002f;
	HoldingRegister_t.ModeCommand_t.C[1] = 0.27108f;
	HoldingRegister_t.ModeCommand_t.C[0] = 0.23031f;
#endif
#if 0
	HoldingRegister_t.SensorCalibration_t.COD_CF = 1.0f;
	HoldingRegister_t.SensorCalibration_t.COD_Intercept = 0.4f;

	COD_SensorCalibration_t.slope = HoldingRegister_t.SensorCalibration_t.COD_CF;
	COD_SensorCalibration_t.intercept = HoldingRegister_t.SensorCalibration_t.COD_Intercept;
#endif
	//COD Sensor calibration, should start with capturing x1 and y1
	COD_SensorCalibration_t.point_flag = 0;

	HoldingRegister_t.ModeCommand_t.BOD_CF = 0.5f;

#if 0
	double data[10] = {95,150,240,360,450,500,620,700,750,800};
	for(int i = 0;i<10;i++)
	{
		HoldingRegister_t.ModeCommand_t.COD_X[i] = (i+1)*100;
		HoldingRegister_t.ModeCommand_t.COD_Y[i] = data[i];

		HoldingRegister_t.ModeCommand_t.TSS_F[i] = (i+1)*100;
		HoldingRegister_t.ModeCommand_t.TSS_G[i] = data[i];
	}
#endif
	//PT100
	HoldingRegister_t.IOUTCalibandTest_t.CalibrationType = 0X01;
//	HoldingRegister_t.ModeCommand_t.COD_Y[] = {95,150,240,360,450,500,620,700,750,800};

	//When the system is rebooted or booted for the first time user is supposed to measure the PD1 and PD2 zero values
	COD_MeasurementStatus.COD_ZERO = 0x00;

	//COD SF default value
	HoldingRegister_t.ModeCommand_t.COD_SF = 100.0f;
	HoldingRegister_t.ModeCommand_t.TSS_SF = 500.0f;
	//Default PD1 and PD2 Zero
	//COD_MeasurementValues_t.PD1_Zero = 30988;
	//COD_MeasurementValues_t.PD2_Zero = 23617;
#if 0
	InputRegister_t.PV_info.PD1_0 = 23000;//30988;
	InputRegister_t.PV_info.PD2_0 = 30000;//23617;
#endif

	//Set the COD Auto Zero flag as we have default data
	COD_MeasurementStatus.COD_ZERO = 0x01;


	//testing purpose
	InputRegister_t.PV_info.CODValue = 0;//12.99;
	InputRegister_t.PV_info.BODValue = 0;//111.23;
	InputRegister_t.PV_info.TSSValue = 0;//56.88;

	//HoldingRegister_t.ModeCommand_t.Epoch_Timestamp = 0x12345678;

#if 0
	//test
	HoldingRegister_t.IOUTConfig_t[0].Current_OP_option = 1; //cod
	HoldingRegister_t.IOUTConfig_t[0].Current_OP_MIN_Value = 0.0f;
	HoldingRegister_t.IOUTConfig_t[0].Current_OP_MAX_Value = 160.0f;

	HoldingRegister_t.IOUTConfig_t[1].Current_OP_option = 2; //BOD
	HoldingRegister_t.IOUTConfig_t[1].Current_OP_MIN_Value = 0.0f;
	HoldingRegister_t.IOUTConfig_t[1].Current_OP_MAX_Value = 100.0f;

	HoldingRegister_t.IOUTConfig_t[2].Current_OP_option = 4;
	HoldingRegister_t.IOUTConfig_t[2].Current_OP_MIN_Value = 0.0f;
	HoldingRegister_t.IOUTConfig_t[2].Current_OP_MAX_Value = 14.0f;

	HoldingRegister_t.IOUTConfig_t[3].Current_OP_option = 4;
	HoldingRegister_t.IOUTConfig_t[3].Current_OP_MIN_Value = 0.0f;
	HoldingRegister_t.IOUTConfig_t[3].Current_OP_MAX_Value = 14.0f;
#endif

#if 0
	//Default data for Alarm output
	HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Parameter = 1; //COD
	HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Hysterisis = 50;//ppm
	HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_Threshold = 500;//ppm
	HoldingRegister_t.RelayOUTConfig_t[0].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Parameter = 1; //COD
	HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Hysterisis = 50;//ppm
	HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_Threshold = 500;//ppm
	HoldingRegister_t.RelayOUTConfig_t[1].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Parameter = 2; //BOD
	HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Hysterisis = 50;//ppm
	HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_Threshold = 500;//ppm
	HoldingRegister_t.RelayOUTConfig_t[2].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Parameter = 2; //BOD
	HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Hysterisis = 50;//ppm
	HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_Threshold = 500;//ppm
	HoldingRegister_t.RelayOUTConfig_t[3].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Parameter = 3; //TSS
	HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Hysterisis = 50;//ppm
	HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_Threshold = 500;//ppm
	HoldingRegister_t.RelayOUTConfig_t[4].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Parameter = 3; //TSS
	HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Hysterisis = 50;//ppm
	HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_Threshold = 500;//ppm
	HoldingRegister_t.RelayOUTConfig_t[5].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Parameter = 4; //pH
	HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Hysterisis = 10;//ph
	HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_Threshold = 1;//pH
	HoldingRegister_t.RelayOUTConfig_t[6].Relay_OP_HIGHLOW = 1;//HIGH = 1; LOW = 2

	HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Parameter = 4; //pH
	HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Hysterisis = 2;//pH
	HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_Threshold = 1;//pH
	HoldingRegister_t.RelayOUTConfig_t[7].Relay_OP_HIGHLOW = 2;//HIGH = 1; LOW = 2
#endif

//	HoldingRegister_t.IOUTCalibandTest_t.CalibrationType = 0x03;//Manual temp input
//	HoldingRegister_t.ModeCommand_t.Temperature_setPoint = 25.0f;//Temperature set point
	HoldingRegister_t.ModeCommand_t.FlowSensorCutoff = 2.3f;

	/*<Last Calibration Data*/
//	InputRegister_t.COD_lastCalibration[0].C[0] = 10.2f;
//	InputRegister_t.COD_lastCalibration[0].C[1] = 20.2f;
//	InputRegister_t.COD_lastCalibration[0].C[2] = 30.2f;
//	InputRegister_t.COD_lastCalibration[0].timestamp = 0x12345689;
//
//	InputRegister_t.TSS_lastCalibration[9].C[0] = 10.2f;
//	InputRegister_t.TSS_lastCalibration[9].C[1] = 20.2f;
//	InputRegister_t.TSS_lastCalibration[9].C[2] = 30.2f;
//	InputRegister_t.TSS_lastCalibration[9].timestamp = 0x12345689;


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
