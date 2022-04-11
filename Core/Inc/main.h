/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*SOftware Version for HMI and Firmware*/
#define HMI_VER_MAJOR		2
#define HMI_VER_MINOR		0
#define HMI_VER_BUGFIX		0
#define FW_VER_MAJOR		2
#define FW_VER_MINOR		0
#define FW_VER_BUGFIX		0

#define MODBUS_1000_BYTES					1
#define HOLDING_REGISTER_BYTE_SIZE			(uint16_t)1500
#define UART_6_RX_DESTINATION_ADDR			(uint32_t)0x20005000
uint8_t DMA_TX_FLAG;
uint8_t DMA_TX_FLAG_HMI;
//If you want to skip the auto zero process for the HMI testing
//#define HMI_TEST
// Card IDs
#define MPH  	0x01
#define MI420  	0x02
#define MIL  	0x03
#define MVUX  	0x04
#define MO420  	0x0C
#define RS485  	0x0E
#define COD		0x0F


// ADC channel data for 16 bit MSB first
#define CH0  0x8100
#define CH1  0xC100
#define CH2  0x9100
#define CH3  0xD100
#define CH4  0xA100
#define CH5  0xE100
#define CH6  0xB100
#define CH7  0xF100

//#define MINMAX_NO
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern uint16_t CardReadADC(uint8_t CardType,uint16_t CHNum);
extern void CardADCInit(uint8_t CardType);
extern uint16_t CArdSPIReadByte(uint16_t TxData,uint8_t CardID);

extern uint16_t ReadADC(uint16_t CHNum);
extern void ADCInit(void);
extern void ProcessModbusQuery(void);
extern void ProcessMOD2_ModbusQuery(void);
extern void ProcessMOD2_ModbusQuery_DMA(void);
//extern void MX_SPI3_Init(void);
//extern void MX_USART3_UART_Init(void);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define TIME_46MS 0
#define TIME_6MS 1
#define TIME_1MS 2
#define TIME_8MS 3

#define NO_OF_SAMPLES 	5


#define TIMER_TICKS_46MS 460 //chnaged th 46ms wait to 100ms wait
#define TIMER_TICKS_6MS 60
#define TIMER_TICKS_1MS 10
#define TIMER_TICKS_8MS 80

// Modbus address - 30000 - 300399

struct ADCReg
{
	// Modbus address - 30000 - 300399
	uint16_t Det1Noise_Ch1;
	uint16_t Det1Signal_Ch1;
	uint16_t Det2Noise_Ch2;
	uint16_t Det2Signal_Ch2;
};

struct InputReg
{
	// Modbus address - 30000 - 300399
	/*uint16_t Det1Noise_Ch1[100];
	uint16_t Det1Signal_Ch1[100];
	uint16_t Det2Noise_Ch2[100];
	uint16_t Det2Signal_Ch2[100];*/
	struct ADCReg SADCReg[110];//100
	// Modbus address - 30400 - 300499 - future use
	uint16_t FutureUse[100];
	// Modbus address - 30500 - 300502
	uint16_t TemperatureCh5;
	uint16_t WDetectCh6;
	uint16_t TestCh7;

};

union InputRegister
{
	struct InputReg SInputReg;
	uint16_t  WordArr[600];
	uint8_t  ByteArr[1200];
};
extern union InputRegister UInputReg;

// Modbus address - 0 - 15
struct CoilDt
{
	uint8_t P[8];
	uint8_t Relay[8];
};

union CoilDtata
{
	struct CoilDt SCoil;
	uint8_t  ByteArr[16];
};
extern union CoilDtata UCoil;

// Modbus address - 40000 - 40004
struct HoldingReg
{
   uint16_t PWMcntCOD;
   uint16_t PWMcntBOD;
   uint16_t PWMcntTSS;
   uint16_t PWMcntPH;
   uint16_t ADCReadStart;
};

union HoldReg
{
	struct HoldingReg SHoldReg;
	uint8_t  ByteArr[10];
};
extern union HoldReg UHoldReg;

/*****************************New MODBUS MAP*************************************/
/*
 *  Input Registers
 *  MODBUS address : 30000 - 30059
 *  MOSCAN address : 30001 - 30060
 */
typedef struct{
	float CODValue;		/*<This value is for the Customer display, display only positive value.*/
	float BODValue;		/*<This value is for the Customer display, display only positive value.*/
	float TSSValue;		/*<This value is for the Customer display, display only positive value.*/
	float TOC;			/*<This value is for the Customer display, display only positive value.*/
	float PD1_MEAN;
	float PD2_MEAN;
	float PD3_MEAN;
	float PD1_0;
	float PD2_0;
	float PD3_0;
	float RSD1;
	float RSD2;
	float RSD3;
	float COD_RAW;
	float TSS_RAW;
	float temp_OIW;
	float pH_value;
	float pH_mV;
	float temp_pH;
	float OIW_value;
	float OIW_raw;
	float TSS_PD2_0;
	float SPARE[3];
//	float Iout_1_value;
//	float Iout_2_value;
//	float Iout_3_value;
//	float Iout_4_value;
//	float Iout_5_value;
//	float Iout_6_value;
//	float Iout_7_value;
//	float Iout_8_value;
	float Iout_value[8];
	float SPARE_A[10];
	float CODValueUser;		/*<This value is for the Developer only*/
	float TSSValueUser;		/*<This value is for the Developer only*/
	float BODValueUser;		/*<This value is for the Developer only*/
}PVhandle_t;

/*Generic handle for COD and TSS factory calibration*/
typedef struct{
	float C0[10];
	float C1[10];
	float C2[10];
	uint32_t epochtimestamp[10];
	uint32_t overflowFlag;
}LastCalibrationFactoryHanlde_t;

/*Generic handle for COD, TSS and pH sensor calibration*/
typedef struct{
	float intercept[10];
	float slope[10];
	uint32_t epochtimestamp[10];
	uint32_t overflowFlag;
}LastCalibrationSensorHandle_t;

typedef struct{
	float BODCalibSlope;
}LastCalibrationBODHanlde_t;

typedef struct{
	float TSSCalibSlope;
	float TSSCalibIntercept;
}LastCalibrationTSSHanlde_t;

typedef struct{
	float OIWCalibSlope;
	float OIWCalibIntercept;
}LastCalibrationOIWHanlde_t;

typedef struct{
	float PHCalibSlope;
	float PHCalibIntercept;
}LastCalibrationPHHanlde_t;

typedef struct{
	float AI1CalibSlope;
	float AI1CalibIntercept;
}LastCalibrationAI1Hanlde_t;

typedef struct{
	float AI2CalibSlope;
	float AI2CalibIntercept;
}LastCalibrationAI2Hanlde_t;

typedef struct{
	uint16_t Slot_1_card_ID[7];
	uint16_t slaveID;
	uint16_t BaudRate;
	uint16_t Parity;
	uint16_t Stopbit;
	/*23/8/2021*/
	uint16_t Status;//not fixed
	uint16_t pH_ADC_Counts;
	uint16_t pH_Temperature_ADC_Counts;
	uint16_t PD1_ADC_Counts;
	uint16_t PD2_ADC_Counts;
	//test
	uint16_t PD1_mean;
	//test
	uint16_t PD2_mean;
	uint16_t PD1_Zero;
	uint16_t PD2_Zero;
	uint16_t PD1_SIGNAL_MIN;
	uint16_t PD1_SIGNAL_MAX;
	uint16_t PD2_SIGNAL_MIN;
	uint16_t PDW_SIGNAL_MAX;
	float pH_Live_Volatge;
	float pH_Temperature_Live_Voltage;
	float PD1_Voltage;/*14/10/2021*/
	float PD2_Voltage;/*19/10/2021*/
	float FlowSensorVolatge;			/*<Sensor voltage output; For diagnostics*/
}SlotParametersHandle_t;


typedef union{
#if(MODBUS_1000_BYTES)
	uint8_t bytes[sizeof(PVhandle_t)
				  + (sizeof(LastCalibrationFactoryHanlde_t))	/*<COD Factory calibration*/
				  + (sizeof(LastCalibrationFactoryHanlde_t))	/*<TSS Factory calibration*/
				  + (sizeof(LastCalibrationSensorHandle_t))		/*<COD Sensor calibration*/
				  + (sizeof(LastCalibrationSensorHandle_t))		/*<TSS Sensor calibration*/
				  + (sizeof(LastCalibrationSensorHandle_t))		/*<pH Sensor calibration*/
				  + (10 * sizeof(LastCalibrationAI1Hanlde_t))
				  + (10 * sizeof(LastCalibrationAI2Hanlde_t))
				  + sizeof(SlotParametersHandle_t)];
#else
	uint8_t bytes[1000];
#endif
	struct{
		//30000
		PVhandle_t PV_info;
		//31000
		LastCalibrationFactoryHanlde_t COD_lastCalibration;
		LastCalibrationFactoryHanlde_t TSS_lastCalibration;
		LastCalibrationSensorHandle_t COD_lastSensorCalibration;
		LastCalibrationSensorHandle_t TSS_lastSensorCalibration;
		LastCalibrationSensorHandle_t pH_lastSensorCalibration;
		LastCalibrationAI1Hanlde_t AI1_lastCalibration[10];
		LastCalibrationAI2Hanlde_t AI2_lastCalibration[10];
		//32000
		SlotParametersHandle_t SlotParameter;
	};

}UInput_t;

UInput_t InputRegister_t;

typedef struct{
//	uint16_t ModeCommand;
	uint8_t ModeCommand_L;
	uint8_t ModeCommand_H;
	uint16_t CommonCommand;
	uint16_t ADC_Samples_for_COD;
	uint16_t PUMP1_ONTIME;
	uint16_t PUMP1_DELAY;
	uint16_t PUMP2_ONTIME;
	uint16_t PUMP2_DELAY;
	uint16_t RANGESELECT; 			/*<Range select for the Analyzer; @ref RANGE_SELECT_MACRO*/
	uint16_t AI1_unit;
	uint16_t AI2_unit;
	float AI1_SF;
	float AI1_MAX_Value;
	float AI1_MIN_Value;
	float AI2_SF;
	float AI2_MAX_Value;
	float AI2_MIN_Value;
	float COD_X[10];
	float COD_Y[10];
	float C[4];//added new//one more varibale added for 3rd degree polynomial
	float COD_SF;
	float BOD_CF;
	float TSS_F[10];
	float TSS_G[10];
	float TSS_K[3];//updated new
	float TSS_SF;
	float PH_SF;
//	float Temperature_setPoint;
	float OIW_H[10];
	float OIW_I[10];
	float M[3];//added new
	float OIW_SF;
	//uint16_t Procedure_State;
	uint16_t treand_time;
	uint16_t HMI_onUpdate;
	uint16_t SlaveID;
	uint16_t Baud;
	uint16_t Stopbit;
	uint16_t Parity;
	float Firmware_Version;
	float GUI_Version;
	float Hardware_Version;
	//test purpose
	float Temperature_setPoint;
	//uint16_t PWM2;
//	uint16_t PWM3;
//	uint16_t PWM4;
	//uint16_t PWM1_offset;
	//uint16_t PWM2_offset;
	uint32_t Epoch_Timestamp;		/*<Epoch time stamp from HMI*/
	float FlowSensorCutoff;			/*<HMI access for cutoff setting*/
	//uint16_t PWM3_offset;
	//uint16_t PWM4_offset;
	//uint16_t relay[8];//address : 187
	uint8_t ModeCommandHMI_L;		/*<Request command from HMI*/
	uint8_t ModeCommandHMI_H;		/*<Request command from HMI*/
	uint16_t CommonCommandHMI;		/*<Request command from HMI*/
	//uint32_t RES[10];				/*<Reserve memory 40 bytes*/
	float CS_PUMP1_ONTIME;			/*<Cleaning pump on time for Check screen*/
	float CS_PUMP1_DELAY;			/*<Cleaning pump de-gas time for check screen*/
	float CS_PUMP2_ONTIME;			/*<Sample pump on time for check screen*/
	float CS_PUMP2_DELAY;			/*<Cleaning pump de-gas time for check screen*/
	float CS_FLASH;					/*<IF 1 : 100 FLASHES; IF 2 : 500 flashes*/
	float CS_PD_1_MIN;				/*<Gives the minimum values from the array of flashes*/
	float CS_PD_1_MAX;				/*<Gives the maximum values from the array of flashes*/
	float CS_PD_2_MIN;				/*<Gives the minimum values from the array of flashes*/
	float CS_PD_2_MAX;				/*<Gives the maximum values from the array of flashes*/
	float AUTO_ZERO;				/*<Auto Zero, if SET : Perform zeroing else : only perform flashing*/
	float HMI_Ver_Major;
	float HMI_Ver_Minor;
	float HMI_Ver_BugFix;
	float FW_Ver_Major;
	float FW_Ver_Minor;
	float FW_Ver_BugFix;
	float RL[8];					/*<Relay 1 - 8*/
}ModeCommandHandle_t;

typedef struct{
	uint16_t Current_OP_option;
	uint16_t Current_OP_Error_Type;
	float Current_OP_MAX_Value;
	float Current_OP_MIN_Value;
}IOUTConfigHanldle_t;

typedef struct{
	float Current_OP_Calib_Point1;
	float Current_OP_Calib_Point2;
}CurrentOPCalibPointsHandle_t;

/* Output Current Calibration */
typedef struct{
	uint16_t Current_OP_Calib_Command;
	uint16_t Current_OP_Common_Command;
	uint16_t Current_OP_Test_Command;
	uint16_t CalibrationType;/*30/8/2021*/
	CurrentOPCalibPointsHandle_t CurrentOPPoints_t[8];
	/*30/8/2021*/
	float AO_5mA_temp_value;
	float AO_19mA_temp_value;
	uint16_t pH_ADC_Counts_p414mA;
	uint16_t pH_ADC_Counts_n414mA;
	/*1/9/2021*/
	uint16_t PT100_ADC_Counts_100;
	uint16_t PT100_ADC_Counts_150;
	uint16_t PT1000_ADC_Counts_1000;
	uint16_t PT1000_ADC_Counts_1500;
	float temperatureSensorCable_resistance; /*<Temperature sensor cable resistance, to be subtracted from calculated resistance*/

}IOUTCalibandTestHandle_t;

typedef struct{
	uint16_t Relay_OP_Parameter;
	uint16_t Relay_OP_HIGHLOW;
	float Relay_OP_Threshold;
	float Relay_OP_Hysterisis;
}RelayOUTConfigHandle_t;

/*13-08-2021*/
typedef struct{
//	float X1;
	float Y1;
//	float X2;
	float Y2;
}Two_pt_Cal;
/*13-08-2021*/
typedef struct{
	uint16_t Sensor_Calib_Command;
	uint16_t Calibration_Type;//0: None, 1: 1pt calib, 2: 2pt calib
	uint16_t AI1_UNIT;
	uint16_t AI2_UNIT;
	//Two_pt_Cal BOD_PT;
	//Two_pt_Cal TSS_PT;
	float TSS_X1;
	float TSS_Y1;
	float TSS_X2;
	float TSS_Y2;
	Two_pt_Cal pH_PT;
	float pH_1pt_calib_point1;
	uint16_t pH_Cal_Point_1_count;
	uint16_t pH_Cal_Point_2_count;
	uint16_t pH_1pt_Cal_point_1_count;
	uint16_t SPARE;
	float COD_X1;//POINT 1
	float COD_Y1;//POINT 1
	float COD_X2;//POINT 2
	float COD_Y2;//POINT 2
	float COD_X3;//POINT 3
	float COD_Y3;//POINT 3
	float COD_CF;
	float COD_Intercept;
	//Two_pt_Cal AI1_PT;
	float TSS_CF;
	float TSS_Intercept;
	float pH_Cal_Point_1_value; 			/*<Caluclated pH value*/
	float pH_Cal_Point_2_value; 			/*<Caluclated pH value*/
	float pH_1pt_Cal_point_1_value; 		/*<Caluclated pH value*/
	float pH_slope;					/*<For HMI display*/
	float pH_intercept;				/*<For HMI display*/
//	float pH_1_pt_slope;					/*<For HMI display*/
//	float pH_1_pt_intercept;				/*<For HMI display*/
	uint16_t NextProcessTime_Hr;	/*<From HMI, Batch mode hour*/
	uint16_t NextProcessTime_Min;	/*<From HMI, Batch mode min*/
	uint16_t NextProcessTime_Sec;	/*<From HMI, Batch mode sec*/
	uint32_t LastBatchTime;			/*<Epoch time from HMI*/
	uint32_t NextProcessTime;		/*<Epoch time for next process time*/
	uint32_t pHElecMeaages;			/*1. p4141, 2. n414, 3. calibrated*/
}SensoralibrationHandle_t;

typedef union{
#if(!MODBUS_1000_BYTES)
	uint8_t bytes[sizeof(ModeCommandHandle_t)\
				  +(8*sizeof(IOUTConfigHanldle_t))\
				  +sizeof(IOUTCalibandTestHandle_t)\
				  +(8*sizeof(RelayOUTConfigHandle_t))\
				  +sizeof(SensoralibrationHandle_t)];/*13-08-2021*/
#else
	uint8_t bytes[HOLDING_REGISTER_BYTE_SIZE];
#endif
	struct{
		//40000
		ModeCommandHandle_t ModeCommand_t;
		//uint8_t RES_1[100 - 4];					/*<Reserve for additional MODBUS register*/
		uint8_t RES_1[100 - 36 - 4 - 28 - 32];						/*<Reserve for additional MODBUS register*/
		//41000
		IOUTConfigHanldle_t IOUTConfig_t[8];
		uint8_t RES_2[100];						/*<Reserve for additional MODBUS register*/
		//42000
		IOUTCalibandTestHandle_t IOUTCalibandTest_t;
		uint8_t RES_3[100 - 4];						/*<Reserve for additional MODBUS register*/
		//43000
		RelayOUTConfigHandle_t RelayOUTConfig_t[8];
		uint8_t RES_4[100];						/*<Reserve for additional MODBUS register*/
		/*13/8/2021*/
		//44000
		SensoralibrationHandle_t SensorCalibration_t;
	};
}Uholding_t;

Uholding_t HoldingRegister_t;


typedef struct{
	uint8_t p[8]; //p5-p12
	uint8_t relay[8];//relay 1 - relay 8
	uint8_t spare[2];
	uint8_t NEW_COMMAND_FLAG;	/*<Set by HMI is there is new command, Reset by uC after receiving the command.*/
	uint8_t TRED_TRIGGER;		/*<NA*/
	uint8_t RES;			/*<NA*/ //21
	uint8_t NoProcess;		/*<NA*/
	uint8_t pHelecCalibrate;	/*<NA*/
	uint8_t AUTOZERO;			/*<If set; Perform zeroing of PD1 and PD2 Zero*/
	uint8_t MeasureReady;		/*<NA*/
	uint8_t CleanReady;			/*<NA*/
	uint8_t ADCReadStart;		/*<NA*/
	uint8_t CleaningTankEmpty;	/*<Bit Set if Cleaning tank is empty*/
	uint8_t MILSwitchState;		/*<State - Yet to be defined.*/
	uint8_t measure;			/*<NA*/
	uint8_t read_acid;			/*<NA*/
	uint8_t read_sample;		/*<NA*/
	uint8_t button_press;		/*<Button inter-locking*/
}CoilStatusHandle_t;

typedef union{
	uint8_t bytes[sizeof(CoilStatusHandle_t)];
	CoilStatusHandle_t CoilStatus_t;
}UCoilStatus_t;

UCoilStatus_t CoilStatusRegister_t;

typedef struct{
	uint8_t WaterLevel;
	uint8_t Error;
	uint8_t P[6];
	uint8_t Relay[8];
	uint8_t Errors1_8[8];
}CoilInputHandle_t;

typedef union{
	uint8_t bytes[sizeof(CoilInputHandle_t)];
	CoilInputHandle_t CoilInput_t;
}UCoilInput_t;

UCoilInput_t CoilInputRegister_t;


typedef union{
	uint8_t bytes[100];
	struct{
		float COD_Value;
		float TSS_Value;
		float BOD_Value;
		float TOC_Value;
		float pH;
		float Temperature;
		float FlowSensorVoltage;
		float FlowSensorStatus;
		float MILSwitchStatus;
		/*2-pt calibration*/
		float COD_slope;
		float COD_intercept;
		float TSS_slope;
		float TSS_intercept;
		float pH_slope;
		float pH_intercept;
		float PD_1;
		float PD_2;
		float PD_1_Zero;
		float PD_2_Zero;
		float PD_2_TSS_Zero;
		float rsd_1;
		float rsd_2;
		float COD_raw;
		float TSS_raw;
		float pH_mV;
		float temp_resistance;
		float RES[50];
		uint16_t main_cmd;
		uint16_t common_cmd;
		float RES_1[10];
		LastCalibrationSensorHandle_t pH_lastSensorCalibration;
		float auto_zero;//test
		float SchedulerTime;				//<modbus address : 239
		float measure;
		float read_acid;
		float read_sample;
		float NoProcess;
	};
}UHolding_Modbus_2_t;
UHolding_Modbus_2_t UHolding_Modbus_2;
/*****************/
extern UART_HandleTypeDef huart3;
extern volatile uint8_t Rxbuff[310], RxFlag;
extern volatile uint16_t Rxptr, Txptr, RxBytes, TxBytes;
extern volatile uint8_t Txbuff[310] , ReadADCFlag;
extern volatile uint8_t Tim2CtrFlag;
extern volatile uint16_t TIM2State;
extern uint8_t TimerStart;

//For RTU MODBUS
extern volatile uint8_t MOD2_RxFlag;
extern volatile uint8_t MOD2_Rxbuff[310];
extern volatile uint8_t MOD2_Txbuff[310];
extern volatile uint16_t MOD2_Rxptr, MOD2_Txptr, MOD2_RxBytes, MOD2_TxBytes;
extern volatile uint16_t DMA_Transaction_no_Tx_uart6;
extern volatile uint16_t DMA_Transaction_no_Tx_uart1;//HMI


//extern uint16_t ADCbuff[100], dataptr;
extern uint16_t crc_calc(char* input_str, int len );

uint16_t AdCounts_pH;
uint16_t AdCounts_MIL;

uint8_t CycleStart;
uint16_t TempDetNoise;
uint16_t TempDetSignal;

uint16_t TempADCounts[10];
//uint16_t TempDet1Signal_Ch1[100];

uint16_t TempDet1Noise_Ch1[500];
uint16_t TempDet1Signal_Ch1[500];
uint16_t TempDet2Noise_Ch2[500];
uint16_t TempDet2Signal_Ch2[500];

//buffers required for the the averaging of PD1 and PD2 noise and signal
uint16_t SignalNoiseDiff_Ch1[500];//19/10/2021
uint16_t SignalNoiseDiff_Ch2[500];//19/10/2021
uint64_t TempMeanPd1,TempMeanPd2;//19/10/2021
uint16_t PD1_new, PD2_new;
uint16_t PD1_Zero, PD2_Zero;
double sd[500];
double sd2[500];

//LPF Array
//uint16_t Y_filter_new;
//uint16_t X_new;
//uint16_t Y_filter_old;
////float sampling_time_noise = 0.059;
////float sampling_time_noise = 0.051;
//uint16_t averaging_time = 1;
uint16_t filter_data_PD1[500];
uint16_t filter_data_PD2[500];
uint64_t filter_data_PD1_mean;
uint64_t filter_data_PD2_mean;

/*2-point Current calibration*/
typedef struct{
	float AO_slope;
	float AO_intercept;
}CurrentOutputCalibrationHandle_t;

CurrentOutputCalibrationHandle_t CurrentOutputCalibration_t[8];

uint8_t AO_19mA_flag;

typedef struct{
	uint8_t AO_19mA_flag;
	float AO_5mA_value;
	float AO_19mA_value;
}AOCalibHandle_t;
AOCalibHandle_t AO_ElectronicHanlde_t;
/*PUMP Action variables and Handles*/
volatile uint16_t g_u16TIM6_count;
volatile uint16_t g_u16TIM6_limit;
volatile uint8_t g_u8PUMP_action;
volatile uint8_t g_u8PUMP_no;//1 - pump1, 2 - pump2

typedef struct
{
	uint16_t u16TIM6_count;
	uint16_t u16TIM6_limit;//0x01 - on time, 0x02 - delay - time
	uint8_t u8PUMP_action;//control the pump 1 on/off
	uint8_t u8PUMP_no;//control the pump 2 on/off
	uint8_t u8PUMP_delayaction;
	uint8_t u8Flag_measurement;//1 : dont allow; 0 : allow ADC measurement //21/10/2021
}PUMPHandle_t;
PUMPHandle_t PUMPControlHandle_t;

volatile uint8_t common_command_flag;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CARD_8_ID3_Pin GPIO_PIN_2
#define CARD_8_ID3_GPIO_Port GPIOE
#define CARD_7_ID1_Pin GPIO_PIN_3
#define CARD_7_ID1_GPIO_Port GPIOE
#define CARD_7_ID2_Pin GPIO_PIN_4
#define CARD_7_ID2_GPIO_Port GPIOE
#define CARD_7_ID3_Pin GPIO_PIN_5
#define CARD_7_ID3_GPIO_Port GPIOE
#define CARD_7_SPI_SS_Pin GPIO_PIN_6
#define CARD_7_SPI_SS_GPIO_Port GPIOE
#define CARD_7_PWR_Pin GPIO_PIN_13
#define CARD_7_PWR_GPIO_Port GPIOC
#define CARD_7_SEL1_Pin GPIO_PIN_14
#define CARD_7_SEL1_GPIO_Port GPIOC
#define CARD_7_SEL2_Pin GPIO_PIN_15
#define CARD_7_SEL2_GPIO_Port GPIOC
#define CARD_6_ID1_Pin GPIO_PIN_0
#define CARD_6_ID1_GPIO_Port GPIOF
#define CARD_6_ID2_Pin GPIO_PIN_1
#define CARD_6_ID2_GPIO_Port GPIOF
#define CARD_6_ID3_Pin GPIO_PIN_2
#define CARD_6_ID3_GPIO_Port GPIOF
#define CARD_6_SPI_SS_Pin GPIO_PIN_3
#define CARD_6_SPI_SS_GPIO_Port GPIOF
#define CARD_6_PWR_Pin GPIO_PIN_4
#define CARD_6_PWR_GPIO_Port GPIOF
#define CARD_6_SEL1_Pin GPIO_PIN_5
#define CARD_6_SEL1_GPIO_Port GPIOF
#define CARD_6_SEL2_Pin GPIO_PIN_6
#define CARD_6_SEL2_GPIO_Port GPIOF
#define DO_LV_CH3_Pin GPIO_PIN_7
#define DO_LV_CH3_GPIO_Port GPIOF
#define DO_LV_CH4_Pin GPIO_PIN_8
#define DO_LV_CH4_GPIO_Port GPIOF
#define DO_LV_CH6_Pin GPIO_PIN_10
#define DO_LV_CH6_GPIO_Port GPIOF
#define RELAY_3_Pin GPIO_PIN_2
#define RELAY_3_GPIO_Port GPIOC
#define RELAY_4_Pin GPIO_PIN_3
#define RELAY_4_GPIO_Port GPIOC
#define MIL_SWITCH_Pin GPIO_PIN_0
#define MIL_SWITCH_GPIO_Port GPIOA
#define RELAY_5_Pin GPIO_PIN_4
#define RELAY_5_GPIO_Port GPIOA
#define CARD_5_ID1_Pin GPIO_PIN_5
#define CARD_5_ID1_GPIO_Port GPIOA
#define CARD_5_ID2_Pin GPIO_PIN_6
#define CARD_5_ID2_GPIO_Port GPIOA
#define CARD_5_ID3_Pin GPIO_PIN_7
#define CARD_5_ID3_GPIO_Port GPIOA
#define CARD_5_SEL2_Pin GPIO_PIN_4
#define CARD_5_SEL2_GPIO_Port GPIOC
#define CARD_5_SEL1_Pin GPIO_PIN_5
#define CARD_5_SEL1_GPIO_Port GPIOC
#define CARD_5_PWR_Pin GPIO_PIN_0
#define CARD_5_PWR_GPIO_Port GPIOB
#define CARD_5_SPI_SS_Pin GPIO_PIN_1
#define CARD_5_SPI_SS_GPIO_Port GPIOB
#define SYS_LED_STATUS_Pin GPIO_PIN_2
#define SYS_LED_STATUS_GPIO_Port GPIOB
#define SYS_LED_ERROR_Pin GPIO_PIN_11
#define SYS_LED_ERROR_GPIO_Port GPIOF
#define CARD_1_SPI_SS_Pin GPIO_PIN_12
#define CARD_1_SPI_SS_GPIO_Port GPIOF
#define CARD_1_PWR_Pin GPIO_PIN_13
#define CARD_1_PWR_GPIO_Port GPIOF
#define CARD_1_SEL1_Pin GPIO_PIN_14
#define CARD_1_SEL1_GPIO_Port GPIOF
#define CARD_1_SEL2_Pin GPIO_PIN_15
#define CARD_1_SEL2_GPIO_Port GPIOF
#define CARD_1_ID1_Pin GPIO_PIN_0
#define CARD_1_ID1_GPIO_Port GPIOG
#define CARD_1_ID2_Pin GPIO_PIN_1
#define CARD_1_ID2_GPIO_Port GPIOG
#define CARD_1_ID3_Pin GPIO_PIN_7
#define CARD_1_ID3_GPIO_Port GPIOE
#define RELAY_6_Pin GPIO_PIN_8
#define RELAY_6_GPIO_Port GPIOE
#define PWM_1_Pin GPIO_PIN_9
#define PWM_1_GPIO_Port GPIOE
#define RELAY_7_Pin GPIO_PIN_10
#define RELAY_7_GPIO_Port GPIOE
#define PWM_2_Pin GPIO_PIN_11
#define PWM_2_GPIO_Port GPIOE
#define RELAY_8_Pin GPIO_PIN_12
#define RELAY_8_GPIO_Port GPIOE
#define PWM_3_Pin GPIO_PIN_13
#define PWM_3_GPIO_Port GPIOE
#define PWM_4_Pin GPIO_PIN_14
#define PWM_4_GPIO_Port GPIOE
#define RESERVE_PE15_Pin GPIO_PIN_15
#define RESERVE_PE15_GPIO_Port GPIOE
#define RELAY_1_Pin GPIO_PIN_10
#define RELAY_1_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_11
#define RELAY_2_GPIO_Port GPIOB
#define RESERVE_PD11_Pin GPIO_PIN_11
#define RESERVE_PD11_GPIO_Port GPIOD
#define RESERVE_PD12_Pin GPIO_PIN_12
#define RESERVE_PD12_GPIO_Port GPIOD
#define RESERVE_PD13_Pin GPIO_PIN_13
#define RESERVE_PD13_GPIO_Port GPIOD
#define RESERVE_PD14_Pin GPIO_PIN_14
#define RESERVE_PD14_GPIO_Port GPIOD
#define CARD_2_SPI_SS_Pin GPIO_PIN_15
#define CARD_2_SPI_SS_GPIO_Port GPIOD
#define CARD_2_PWR_Pin GPIO_PIN_2
#define CARD_2_PWR_GPIO_Port GPIOG
#define CARD_2_SEL1_Pin GPIO_PIN_3
#define CARD_2_SEL1_GPIO_Port GPIOG
#define CARD_2_SEL2_Pin GPIO_PIN_4
#define CARD_2_SEL2_GPIO_Port GPIOG
#define CARD_2_ID1_Pin GPIO_PIN_5
#define CARD_2_ID1_GPIO_Port GPIOG
#define CARD_2_ID2_Pin GPIO_PIN_6
#define CARD_2_ID2_GPIO_Port GPIOG
#define CARD_2_ID3_Pin GPIO_PIN_7
#define CARD_2_ID3_GPIO_Port GPIOG
#define RESERVE_PG8_Pin GPIO_PIN_8
#define RESERVE_PG8_GPIO_Port GPIOG
#define ADM_2_CLTR_Pin GPIO_PIN_8
#define ADM_2_CLTR_GPIO_Port GPIOC
#define CARD_3_SPI_SS_Pin GPIO_PIN_9
#define CARD_3_SPI_SS_GPIO_Port GPIOC
#define ADM_CLTR_Pin GPIO_PIN_8
#define ADM_CLTR_GPIO_Port GPIOA
#define CARD_3_ID1_Pin GPIO_PIN_11
#define CARD_3_ID1_GPIO_Port GPIOA
#define CARD_3_ID2_Pin GPIO_PIN_12
#define CARD_3_ID2_GPIO_Port GPIOA
#define CARD_3_SEL2_Pin GPIO_PIN_15
#define CARD_3_SEL2_GPIO_Port GPIOA
#define CARD_3_PWR_Pin GPIO_PIN_6
#define CARD_3_PWR_GPIO_Port GPIOD
#define CARD_3_ID3_Pin GPIO_PIN_9
#define CARD_3_ID3_GPIO_Port GPIOG
#define CARD_3_SEL1_Pin GPIO_PIN_10
#define CARD_3_SEL1_GPIO_Port GPIOG
#define RESERVE_PG11_Pin GPIO_PIN_11
#define RESERVE_PG11_GPIO_Port GPIOG
#define RESERVE_PG12_Pin GPIO_PIN_12
#define RESERVE_PG12_GPIO_Port GPIOG
#define RESERVE_PG13_Pin GPIO_PIN_13
#define RESERVE_PG13_GPIO_Port GPIOG
#define RESERVE_PG14_Pin GPIO_PIN_14
#define RESERVE_PG14_GPIO_Port GPIOG
#define RESERVE_PG15_Pin GPIO_PIN_15
#define RESERVE_PG15_GPIO_Port GPIOG
#define CARD_8_PWR_Pin GPIO_PIN_4
#define CARD_8_PWR_GPIO_Port GPIOB
#define CARD_8_SEL1_Pin GPIO_PIN_5
#define CARD_8_SEL1_GPIO_Port GPIOB
#define CARD_8_SEL2_Pin GPIO_PIN_8
#define CARD_8_SEL2_GPIO_Port GPIOB
#define CARD_8_SPI_SS_Pin GPIO_PIN_9
#define CARD_8_SPI_SS_GPIO_Port GPIOB
#define CARD_8_ID1_Pin GPIO_PIN_0
#define CARD_8_ID1_GPIO_Port GPIOE
#define CARD_8_ID2_Pin GPIO_PIN_1
#define CARD_8_ID2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
//Starting addresses
#define INPUT_REGISTER_ADDRESS_31000	(uint16_t)(sizeof(PVhandle_t))
#define INPUT_REGISTER_ADDRESS_32000	(uint16_t)(INPUT_REGISTER_ADDRESS_31000\
													+ (sizeof(LastCalibrationFactoryHanlde_t))\
													+ (sizeof(LastCalibrationFactoryHanlde_t))\
													+ (sizeof(LastCalibrationSensorHandle_t))\
													+ (sizeof(LastCalibrationSensorHandle_t))\
													+ (sizeof(LastCalibrationSensorHandle_t))\
													+ (10 * sizeof(LastCalibrationAI1Hanlde_t))\
													+ (10 * sizeof(LastCalibrationAI2Hanlde_t)))


//#define HOLDING_REGISTER_ADDRESS_41000	(uint16_t)(sizeof(ModeCommandHandle_t) + 100 - 4)
#define HOLDING_REGISTER_ADDRESS_41000	(uint16_t)(sizeof(ModeCommandHandle_t) + 100 - 36 - 4 - 28 - 32)
#define HOLDING_REGISTER_ADDRESS_42000	(uint16_t)(HOLDING_REGISTER_ADDRESS_41000\
													+ (8*sizeof(IOUTConfigHanldle_t))\
													+ 100)
#define HOLDING_REGISTER_ADDRESS_43000	(uint16_t)(HOLDING_REGISTER_ADDRESS_42000\
													+ sizeof(IOUTCalibandTestHandle_t)\
													+96)
/*13/8/2021*/
#define HOLDING_REGISTER_ADDRESS_44000	(uint16_t)(HOLDING_REGISTER_ADDRESS_43000\
													+ (8*sizeof(RelayOUTConfigHandle_t))\
													+ 100)

#if 0
//Commands
//0x220 RUN MODE
#define Batch							0x2201
#define External						0x2202
#define Stopped							0x2203
//0x2300 Setting/config mode
#define Default							0x2311
#define Edit							0x2312
#define Save							0x2314
#define Calibrate_COD					0x2321
#define Calibrate_TSS					0x2322
#define Calibrate_PH					0x2323
#define Calibrate_A1					0x2324
#define Calibrate_A2					0x2325
//0x2400 Factory Setting
#define Electronics_Calibration_I_Out	0x2411
#define Electronics_Calibration_mV		0x2412
#define Electronics_Calibration_I_input	0x2413
#define I_Out_tes						0x2421
#define Relay_test						0x2422
#define CODCoeffs						0x2431
#define CalibrateCOD					0x2432
#define TSSCoeffs						0x2433
#define CalibrateTSS					0x2434
#endif

#define RUN_MODE						0x22
#define Batch							0x01
#define External						0x02
#define Stopped							0x03
#define COD_Measure						0x11//21/10/2021
#define TSS_Measure						COD_Measure
#define PUMP1_ACTION					0x12//21/10/2021
#define PUMP2_ACTION					0x13//21/10/2021
#define AUTO_COD_MEASURE				0X14//21/10/2021
#define AUTO_COD_ZERO					0x15//15/11/2021
#define COD_ZERO_MEAS					0x16
#define COD_FACTORY_SETASZERO			0x17
#define TSS_FACTORY_SETASZERO			COD_FACTORY_SETASZERO
#define COD_SENSOR_MEASURE_pt1			0x18
#define COD_SENSOR_MEASURE_pt2			0x19
#define COD_SENSOR_MEASURE_pt3			0x20
#define STOP_RUNNING_PUMP				0x99

#define SETTING_CONFIG_MODE				0x23
#define Default							0x11
#define Edit							0x12
#define Save							0x14
#define SchedulerSave					0x15
/*Sensor Calibration*/
#define Calibrate_COD					0x21
#define Calibrate_TSS					0x22
#define Calibrate_PH					0x23
#define Calibrate_A1					0x24
#define Calibrate_A2					0x25
/*19/7/2021*/
#define Relay_Config_page				0x30 /*Alarm Limit*/
#define Relay_Config					0x31 /*Save the alarm Limits*/
#define Current_Config_page				0x40 /*Parameter Range*/
#define Current_Config					0x41 /*Save the Parameter Range*/



#define FACTORY_MODE							0x24
#define Electronics_Calibration_I_Out			0x11
#define Electronics_Calibration_mV				0x12 /*pH Calibrate page (Start ADC command)*/
//#define Electronics_Calibration_mV_Calibrate	0x13 /*Calibrate Command*/
#define Electronics_Calibration_I_input			0x14
#define Electronics_Calibration_PT_100			0x16/*1/9/2021*/
#define Electronics_Calibration_PT_1000 		0x17/*1/9/2021*/
#define I_Out_test								0x21
#define Relay_test								0x22
#define CODCoeffs								0x31
#define CalibrateCOD							0x32
#define TSSCoeffs								0x33
#define CalibrateTSS							0x34
#define Select_Range							0x41 /*<Set button*/


/*CHECK*/
#define CHECK_MODE								0x25
#define COD_Check								0x10
#define TSS_Check								0x20
#define Read_acid								0x01
#define Read_sample								0x02

/*SOFTWARE REST*/
#define SOFTWARE_RESET							0x5A

/*COMMON COMMANDS*/
#define Sample_pump								0x01
#define Clean_pump								0x02
#define Measure_sample							0x05
#define Electronics_Calibration_mV_Calibrate	0x06 /*Calibrate Command*/
#define Sensor_Calibration_pH					0x07
#define Electronic_Calibration_PT				0x08/*1/9/2021*/
#define Factory_Calibrate_COD					0x09/*25/10/2021*/
#define Sensor_Calibrate_COD					0x0A//HMI
#define Factory_Calibrate_TSS					0x10/*25/10/2021*/
#define Sensor_Calibrate_TSS					0x0B//HMI

/*Current output calibration commands*/
#define AO1_GENERATE_5mA				1
#define AO1_GENERATE_19mA				2
#define AO1_CALIBRATE					3
#define AO2_GENERATE_5mA				4
#define AO2_GENERATE_19mA				5
#define AO2_CALIBRATE					6
#define AO3_GENERATE_5mA				7
#define AO3_GENERATE_19mA				8
#define AO3_CALIBRATE					9
#define AO4_GENERATE_5mA				10
#define AO4_GENERATE_19mA				11
#define AO4_CALIBRATE					12
#define AO5_GENERATE_5mA				13
#define AO5_GENERATE_19mA				14
#define AO5_CALIBRATE					15
#define AO6_GENERATE_5mA				16
#define AO6_GENERATE_19mA				17
#define AO6_CALIBRATE					18
#define AO7_GENERATE_5mA				19
#define AO7_GENERATE_19mA				20
#define AO7_CALIBRATE					21
#define AO8_GENERATE_5mA				22
#define AO8_GENERATE_19mA				23
#define AO8_CALIBRATE					24



#define AO1								1
#define AO2								2
#define AO3								3
#define AO4								4
#define AO5								5
#define AO6								6
#define AO7								7
#define AO8								8
#define pH_ADC_Value_Save				9
#define Temp_ADC_Value_Save				10

#define PWM_5mA_COUNTS					6469UL//2070.0f
#define PWM_19mA_COUNTS					21855UL//6993.0f
#define PWM_TIM_FS						24999UL//7999UL



//RELAY control
#define RELAY_1_ON()					HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET)
#define RELAY_1_OFF()					HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_RESET)
#define RELAY_2_ON()					HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET)
#define RELAY_2_OFF()					HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET)
#define RELAY_3_ON()					HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, GPIO_PIN_SET)
#define RELAY_3_OFF()					HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, GPIO_PIN_RESET)
#define RELAY_4_ON()					HAL_GPIO_WritePin(RELAY_4_GPIO_Port, RELAY_4_Pin, GPIO_PIN_SET)
#define RELAY_4_OFF()					HAL_GPIO_WritePin(RELAY_4_GPIO_Port, RELAY_4_Pin, GPIO_PIN_RESET)
#define RELAY_5_ON()					HAL_GPIO_WritePin(RELAY_5_GPIO_Port, RELAY_5_Pin, GPIO_PIN_SET)
#define RELAY_5_OFF()					HAL_GPIO_WritePin(RELAY_5_GPIO_Port, RELAY_5_Pin, GPIO_PIN_RESET)
#define RELAY_6_ON()					HAL_GPIO_WritePin(RELAY_6_GPIO_Port, RELAY_6_Pin, GPIO_PIN_SET)
#define RELAY_6_OFF()					HAL_GPIO_WritePin(RELAY_6_GPIO_Port, RELAY_6_Pin, GPIO_PIN_RESET)
#define RELAY_7_ON()					HAL_GPIO_WritePin(RELAY_7_GPIO_Port, RELAY_7_Pin, GPIO_PIN_SET)
#define RELAY_7_OFF()					HAL_GPIO_WritePin(RELAY_7_GPIO_Port, RELAY_7_Pin, GPIO_PIN_RESET)
#define RELAY_8_ON()					HAL_GPIO_WritePin(RELAY_8_GPIO_Port, RELAY_8_Pin, GPIO_PIN_SET)
#define RELAY_8_OFF()					HAL_GPIO_WritePin(RELAY_8_GPIO_Port, RELAY_8_Pin, GPIO_PIN_RESET)

//pump control definition - GPIO toggle action
#define PUMP1_ON()						HAL_GPIO_WritePin(DO_LV_CH3_GPIO_Port, DO_LV_CH3_Pin, GPIO_PIN_SET)
#define PUMP1_OFF()						HAL_GPIO_WritePin(DO_LV_CH3_GPIO_Port, DO_LV_CH3_Pin, GPIO_PIN_RESET)
#define PUMP2_ON()						HAL_GPIO_WritePin(DO_LV_CH4_GPIO_Port, DO_LV_CH4_Pin, GPIO_PIN_SET)
#define PUMP2_OFF()						HAL_GPIO_WritePin(DO_LV_CH4_GPIO_Port, DO_LV_CH4_Pin, GPIO_PIN_RESET)
#define VALVE_ON()						HAL_GPIO_WritePin(DO_LV_CH6_GPIO_Port, DO_LV_CH6_Pin, GPIO_PIN_SET)
#define VALVE_OFF()						HAL_GPIO_WritePin(DO_LV_CH6_GPIO_Port, DO_LV_CH6_Pin, GPIO_PIN_RESET)

// ADM MODBUS controls
#define ADM_CLTR_LOW()					HAL_GPIO_WritePin(ADM_CLTR_GPIO_Port, ADM_CLTR_Pin, GPIO_PIN_RESET)
#define ADM_CLTR_HIGH()					HAL_GPIO_WritePin(ADM_CLTR_GPIO_Port, ADM_CLTR_Pin, GPIO_PIN_SET)
#define ADM_2_CLTR_LOW()				HAL_GPIO_WritePin(ADM_2_CLTR_GPIO_Port, ADM_2_CLTR_Pin, GPIO_PIN_RESET)
#define ADM_2_CLTR_HIGH()				HAL_GPIO_WritePin(ADM_2_CLTR_GPIO_Port, ADM_2_CLTR_Pin, GPIO_PIN_SET)


//pH Card
#define SEL2_pH_REDOX_SET()					HAL_GPIO_WritePin(CARD_6_SEL2_GPIO_Port,CARD_6_SEL2_Pin,GPIO_PIN_SET)
#define SEL2_pH_REDOX_RESET()				HAL_GPIO_WritePin(CARD_6_SEL2_GPIO_Port,CARD_6_SEL2_Pin,GPIO_PIN_RESET)
#define SEL1_PT100()						HAL_GPIO_WritePin(CARD_6_SEL1_GPIO_Port,CARD_6_SEL1_Pin,GPIO_PIN_SET)
#define SEL1_PT1000()						HAL_GPIO_WritePin(CARD_6_SEL1_GPIO_Port,CARD_6_SEL1_Pin,GPIO_PIN_RESET)

//COD card
#define CARD_1_LAMP_ON()					HAL_GPIO_WritePin(CARD_1_PWR_GPIO_Port,CARD_1_PWR_Pin,GPIO_PIN_SET)
#define CARD_1_LAMP_OFF()					HAL_GPIO_WritePin(CARD_1_PWR_GPIO_Port,CARD_1_PWR_Pin,GPIO_PIN_RESET)
#define CARD_1_SEL1_ON()					HAL_GPIO_WritePin(CARD_1_SEL1_GPIO_Port,CARD_1_SEL1_Pin,GPIO_PIN_SET)
#define CARD_1_SEL1_OFF()					HAL_GPIO_WritePin(CARD_1_SEL1_GPIO_Port,CARD_1_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_1_SEL2_ON()					HAL_GPIO_WritePin(CARD_1_SEL2_GPIO_Port,CARD_1_SEL2_Pin,GPIO_PIN_SET)
#define CARD_1_SEL2_OFF()					HAL_GPIO_WritePin(CARD_1_SEL2_GPIO_Port,CARD_1_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_1_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_1_SPI_SS_GPIO_Port,CARD_1_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_1_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_1_SPI_SS_GPIO_Port,CARD_1_SPI_SS_Pin,GPIO_PIN_RESET)

//CARD 2
#define CARD_2_LAMP_ON()					HAL_GPIO_WritePin(CARD_2_PWR_GPIO_Port,CARD_2_PWR_Pin,GPIO_PIN_SET)
#define CARD_2_LAMP_OFF()					HAL_GPIO_WritePin(CARD_2_PWR_GPIO_Port,CARD_2_PWR_Pin,GPIO_PIN_RESET)
#define CARD_2_SEL1_ON()					HAL_GPIO_WritePin(CARD_2_SEL1_GPIO_Port,CARD_2_SEL1_Pin,GPIO_PIN_SET)
#define CARD_2_SEL1_OFF()					HAL_GPIO_WritePin(CARD_2_SEL1_GPIO_Port,CARD_2_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_2_SEL2_ON()					HAL_GPIO_WritePin(CARD_2_SEL2_GPIO_Port,CARD_2_SEL2_Pin,GPIO_PIN_SET)
#define CARD_2_SEL2_OFF()					HAL_GPIO_WritePin(CARD_2_SEL2_GPIO_Port,CARD_2_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_2_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_2_SPI_SS_GPIO_Port,CARD_2_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_2_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_2_SPI_SS_GPIO_Port,CARD_2_SPI_SS_Pin,GPIO_PIN_RESET)

//CARD3
#define CARD_3_LAMP_ON()					HAL_GPIO_WritePin(CARD_3_PWR_GPIO_Port,CARD_3_PWR_Pin,GPIO_PIN_SET)
#define CARD_3_LAMP_OFF()					HAL_GPIO_WritePin(CARD_3_PWR_GPIO_Port,CARD_3_PWR_Pin,GPIO_PIN_RESET)
#define CARD_3_SEL1_ON()					HAL_GPIO_WritePin(CARD_3_SEL1_GPIO_Port,CARD_3_SEL1_Pin,GPIO_PIN_SET)
#define CARD_3_SEL1_OFF()					HAL_GPIO_WritePin(CARD_3_SEL1_GPIO_Port,CARD_3_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_3_SEL2_ON()					HAL_GPIO_WritePin(CARD_3_SEL2_GPIO_Port,CARD_3_SEL2_Pin,GPIO_PIN_SET)
#define CARD_3_SEL2_OFF()					HAL_GPIO_WritePin(CARD_3_SEL2_GPIO_Port,CARD_3_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_3_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_3_SPI_SS_GPIO_Port,CARD_3_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_3_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_3_SPI_SS_GPIO_Port,CARD_3_SPI_SS_Pin,GPIO_PIN_RESET)

//CARD 5
#define CARD_5_LAMP_ON()					HAL_GPIO_WritePin(CARD_5_PWR_GPIO_Port,CARD_5_PWR_Pin,GPIO_PIN_SET)
#define CARD_5_LAMP_OFF()					HAL_GPIO_WritePin(CARD_5_PWR_GPIO_Port,CARD_5_PWR_Pin,GPIO_PIN_RESET)
#define CARD_5_SEL1_ON()					HAL_GPIO_WritePin(CARD_5_SEL1_GPIO_Port,CARD_5_SEL1_Pin,GPIO_PIN_SET)
#define CARD_5_SEL1_OFF()					HAL_GPIO_WritePin(CARD_5_SEL1_GPIO_Port,CARD_5_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_5_SEL2_ON()					HAL_GPIO_WritePin(CARD_5_SEL2_GPIO_Port,CARD_5_SEL2_Pin,GPIO_PIN_SET)
#define CARD_5_SEL2_OFF()					HAL_GPIO_WritePin(CARD_5_SEL2_GPIO_Port,CARD_5_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_5_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_5_SPI_SS_GPIO_Port,CARD_5_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_5_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_5_SPI_SS_GPIO_Port,CARD_5_SPI_SS_Pin,GPIO_PIN_RESET)

//pH CARD
#define CARD_6_LAMP_ON()					HAL_GPIO_WritePin(CARD_6_PWR_GPIO_Port,CARD_6_PWR_Pin,GPIO_PIN_SET)
#define CARD_6_LAMP_OFF()					HAL_GPIO_WritePin(CARD_6_PWR_GPIO_Port,CARD_6_PWR_Pin,GPIO_PIN_RESET)
#define CARD_6_SEL1_ON()					HAL_GPIO_WritePin(CARD_6_SEL1_GPIO_Port,CARD_6_SEL1_Pin,GPIO_PIN_SET)
#define CARD_6_SEL1_OFF()					HAL_GPIO_WritePin(CARD_6_SEL1_GPIO_Port,CARD_6_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_6_SEL2_ON()					HAL_GPIO_WritePin(CARD_6_SEL2_GPIO_Port,CARD_6_SEL2_Pin,GPIO_PIN_SET)
#define CARD_6_SEL2_OFF()					HAL_GPIO_WritePin(CARD_6_SEL2_GPIO_Port,CARD_6_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_6_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_6_SPI_SS_GPIO_Port,CARD_6_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_6_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_6_SPI_SS_GPIO_Port,CARD_6_SPI_SS_Pin,GPIO_PIN_RESET)

//CARD 7
#define CARD_7_LAMP_ON()					HAL_GPIO_WritePin(CARD_7_PWR_GPIO_Port,CARD_7_PWR_Pin,GPIO_PIN_SET)
#define CARD_7_LAMP_OFF()					HAL_GPIO_WritePin(CARD_7_PWR_GPIO_Port,CARD_7_PWR_Pin,GPIO_PIN_RESET)
#define CARD_7_SEL1_ON()					HAL_GPIO_WritePin(CARD_7_SEL1_GPIO_Port,CARD_7_SEL1_Pin,GPIO_PIN_SET)
#define CARD_7_SEL1_OFF()					HAL_GPIO_WritePin(CARD_7_SEL1_GPIO_Port,CARD_7_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_7_SEL2_ON()					HAL_GPIO_WritePin(CARD_7_SEL2_GPIO_Port,CARD_7_SEL2_Pin,GPIO_PIN_SET)
#define CARD_7_SEL2_OFF()					HAL_GPIO_WritePin(CARD_7_SEL2_GPIO_Port,CARD_7_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_7_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_7_SPI_SS_GPIO_Port,CARD_7_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_7_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_7_SPI_SS_GPIO_Port,CARD_7_SPI_SS_Pin,GPIO_PIN_RESET)

//CARD 8
#define CARD_8_LAMP_ON()					HAL_GPIO_WritePin(CARD_8_PWR_GPIO_Port,CARD_8_PWR_Pin,GPIO_PIN_SET)
#define CARD_8_LAMP_OFF()					HAL_GPIO_WritePin(CARD_8_PWR_GPIO_Port,CARD_8_PWR_Pin,GPIO_PIN_RESET)
#define CARD_8_SEL1_ON()					HAL_GPIO_WritePin(CARD_8_SEL1_GPIO_Port,CARD_8_SEL1_Pin,GPIO_PIN_SET)
#define CARD_8_SEL1_OFF()					HAL_GPIO_WritePin(CARD_8_SEL1_GPIO_Port,CARD_8_SEL1_Pin,GPIO_PIN_RESET)
#define CARD_8_SEL2_ON()					HAL_GPIO_WritePin(CARD_8_SEL2_GPIO_Port,CARD_8_SEL2_Pin,GPIO_PIN_SET)
#define CARD_8_SEL2_OFF()					HAL_GPIO_WritePin(CARD_8_SEL2_GPIO_Port,CARD_8_SEL2_Pin,GPIO_PIN_RESET)
#define CARD_8_SPI_SS_ON()					HAL_GPIO_WritePin(CARD_8_SPI_SS_GPIO_Port,CARD_8_SPI_SS_Pin,GPIO_PIN_SET)
#define CARD_8_SPI_SS_OFF()					HAL_GPIO_WritePin(CARD_8_SPI_SS_GPIO_Port,CARD_8_SPI_SS_Pin,GPIO_PIN_RESET)

//Physical slots on the MOTHERBOARD
#define CARD_SLOT_1							0x01
#define CARD_SLOT_2							0x02
#define CARD_SLOT_3							0x03
#define CARD_SLOT_4							0x04
#define CARD_SLOT_5							0x05
#define CARD_SLOT_6							0x06
#define CARD_SLOT_7							0x07
#define CARD_SLOT_8							0x08



// Standard CARD GPIO definition
//COD CARD PIN DEFINATION
#define COD_LAMP_ON(x)						((x == CARD_SLOT_1) ? CARD_1_LAMP_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_LAMP_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_LAMP_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_LAMP_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_LAMP_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_LAMP_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_LAMP_ON():0)

#define COD_LAMP_OFF(x)						((x == CARD_SLOT_1) ? CARD_1_LAMP_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_LAMP_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_LAMP_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_LAMP_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_LAMP_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_LAMP_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_LAMP_OFF():0)

#define COD_SEL1_ON(x)						((x == CARD_SLOT_1) ? CARD_1_SEL1_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL1_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL1_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL1_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL1_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL1_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL1_ON():0)

#define COD_SEL1_OFF(x)						((x == CARD_SLOT_1) ? CARD_1_SEL1_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL1_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL1_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL1_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL1_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL1_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL1_OFF():0)

#define COD_SEL2_ON(x)						((x == CARD_SLOT_1) ? CARD_1_SEL2_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL2_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL2_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL2_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL2_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL2_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL2_ON():0)

#define COD_SEL2_OFF(x)						((x == CARD_SLOT_1) ? CARD_1_SEL2_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL2_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL2_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL2_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL2_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL2_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL2_OFF():0)

#define COD_SPI_SS_ON(x)					((x == CARD_SLOT_1) ? CARD_1_SPI_SS_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_SPI_SS_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_SPI_SS_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_SPI_SS_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_SPI_SS_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_SPI_SS_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_SPI_SS_ON():0)

#define COD_SPI_SS_OFF(x)					((x == CARD_SLOT_1) ? CARD_1_SPI_SS_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_SPI_SS_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_SPI_SS_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_SPI_SS_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_SPI_SS_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_SPI_SS_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_SPI_SS_OFF():0)

//PH CARD PIN DEFINATION
#define PH_LAMP_ON(x)						((x == CARD_SLOT_1) ? CARD_1_LAMP_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_LAMP_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_LAMP_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_LAMP_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_LAMP_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_LAMP_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_LAMP_ON():0)

#define PH_LAMP_OFF(x)						((x == CARD_SLOT_1) ? CARD_1_LAMP_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_LAMP_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_LAMP_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_LAMP_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_LAMP_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_LAMP_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_LAMP_OFF():0)

#define PH_SEL1_ON(x)						((x == CARD_SLOT_1) ? CARD_1_SEL1_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL1_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL1_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL1_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL1_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL1_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL1_ON():0)

#define PH_SEL1_OFF(x)						((x == CARD_SLOT_1) ? CARD_1_SEL1_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL1_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL1_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL1_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL1_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL1_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL1_OFF():0)

#define PH_SEL2_ON(x)						((x == CARD_SLOT_1) ? CARD_1_SEL2_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL2_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL2_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL2_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL2_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL2_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL2_ON():0)

#define PH_SEL2_OFF(x)						((x == CARD_SLOT_1) ? CARD_1_SEL2_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_SEL2_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_SEL2_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_SEL2_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_SEL2_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_SEL2_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_SEL2_OFF():0)

#define PH_SPI_SS_ON(x)					    ((x == CARD_SLOT_1) ? CARD_1_SPI_SS_ON():\
											 (x == CARD_SLOT_2) ? CARD_2_SPI_SS_ON():\
											 (x == CARD_SLOT_3) ? CARD_3_SPI_SS_ON():\
											 (x == CARD_SLOT_5) ? CARD_5_SPI_SS_ON():\
											 (x == CARD_SLOT_6) ? CARD_6_SPI_SS_ON():\
											 (x == CARD_SLOT_7) ? CARD_7_SPI_SS_ON():\
											 (x == CARD_SLOT_8) ? CARD_8_SPI_SS_ON():0)

#define PH_SPI_SS_OFF(x)					((x == CARD_SLOT_1) ? CARD_1_SPI_SS_OFF():\
											 (x == CARD_SLOT_2) ? CARD_2_SPI_SS_OFF():\
											 (x == CARD_SLOT_3) ? CARD_3_SPI_SS_OFF():\
											 (x == CARD_SLOT_5) ? CARD_5_SPI_SS_OFF():\
											 (x == CARD_SLOT_6) ? CARD_6_SPI_SS_OFF():\
											 (x == CARD_SLOT_7) ? CARD_7_SPI_SS_OFF():\
											 (x == CARD_SLOT_8) ? CARD_8_SPI_SS_OFF():0)




/*To be used in 1-pt sensor calibration*/
#define ZERO_pH							0.0f

/*1/9/2021*/
#define TEMP_TYPE_PT100					0x01
#define TEMP_TYPE_PT1000				0x02
#define TEMP_TYPE_MANUAL				0x03

//PD1 and PD2 ADC Channel
#ifdef MINMAX_NO
#define PD1_CHANNEL						0x02//0x01//0x00
#define PD2_CHANNEL						0x03//0x03//0x01
#else
#define PD1_CHANNEL						0x00//0x01//0x00
#define PD2_CHANNEL						0x03//0x03//0x01
#endif

//COD filtering
#define COD_OUT_FILTER_MIN				20
#define COD_OUT_FILTER_MAX				80
//#define COD_OUT_FILTER_RANGE			COD_OUT_FILTER_MAX - COD_OUT_FILTER_MIN//BUG
#define COD_OUT_FILTER_RANGE			((COD_OUT_FILTER_MAX == COD_OUT_FILTER_MIN)? 100 - (COD_OUT_FILTER_MAX + COD_OUT_FILTER_MIN) : \
																						COD_OUT_FILTER_MAX - COD_OUT_FILTER_MIN)//BUG

//Pump Operation macros
#define PUMP1							0x01
#define PUMP2							0x02
#define OPERATION_PUMP_ON				0x01
#define OPERATION_PUMP_OFF				0x02

#define PUMP1_TURN_OFF					0x05
#define PUMP2_TURN_OFF					0x06
#define PUMP1_TURN_ON					0x01
#define PUMP2_TURN_ON					0x02

#define SAVE_CONFIGURATION_DATA			0x01
#define SAVE_CALIBRATION_DATA			0x02
#define SAVE_PROCESSPARAMETER_DATA		0x03

//HMI inter-locking
#define HMI_INTERLOCK_SAMPLE_PUMP				1
#define HMI_INTERLOCK_ACID_PUMP					2
#define HMI_INTERLOCK_MEASURE					3
#define HMI_INTERLOCK_SENSOR_SAMPLE_PUMP		4
#define HMI_INTERLOCK_SENSOR_ACID_PUMP			5
#define HMI_INTERLOCK_SENSOR_MEASURE			6
#define HMI_INTERLOCK_SENSOR_READACID			7	/*<For acid pump and flashing*/
#define HMI_INTERLOCK_SENSOR_READSAMPLE			8	/*<For sample pump and flashing*/
#define HMI_INTERLOCKING_RESETALL				9	/*<For resetting all the buttons to orignal state*/


/*
 * @ref RANGE_SELECT_MACRO
 */
#define MODEL_3041_3042			4	/*<COD : 20000 mg/l*/
#define MODEL_3051_3052			5	/*<COD : 05000 mg/l*/
#define MODEL_3031_3032			3	/*<COD : 02000 mg/l*/
#define MODEL_3021_3022			2	/*<COD : 00800 mg/l*/
#define MODEL_3011_3012			1	/*<COD : 00300 mg/l*/

/*Enum for parameter limit*/
typedef enum{
	COD_20000_UPPER	= 20000,
	COD_5000_UPPER	= 5000,
	COD_2000_UPPER	= 2000,
	COD_800_UPPER	= 800,
	COD_300_UPPER	= 300,
	TSS_750_UPPER	= 750,
	TSS_450_UPPER	= 450
}AWA_RangeSelect_t;
/*Control status bits*/
typedef union{
	uint16_t status;
	struct{
		unsigned PUMP1_Cleaning_OPERATION 	: 1;
		unsigned PUMP2_Sample_OPERATION 	: 1;
		unsigned VALVE_OPERATION 			: 1;
		unsigned RELAY1_OPERATION			: 1;
		unsigned RELAY2_OPERATION			: 1;
		unsigned RELAY3_OPERATION			: 1;
		unsigned RELAY4_OPERATION			: 1;
		unsigned RELAY5_OPERATION			: 1;
		unsigned RELAY6_OPERATION			: 1;
		unsigned RELAY7_OPERATION			: 1;
		unsigned RELAY8_OPERATION			: 1;
		unsigned SPARE						: 7;
	};
}AWADOControlHandle_t;
AWADOControlHandle_t AWADOControlStatus_t;

typedef union{
	uint16_t status;
	struct{
		unsigned BatchMode 					: 1;
		unsigned ExternMode 				: 1;
		unsigned ContinuousMode 			: 1;
		unsigned MeasureSample				: 1;
		unsigned Stopped_Mode 				: 1;
		unsigned SensorCalibration_COD 		: 1;
		unsigned SensorCalibration_pH 		: 1;
		unsigned SensorCalibration_TSS 		: 1;
		unsigned SensorCalibration_BOD 		: 1;
		unsigned ElectronicCal_AO			: 1;
		unsigned ElectronicCal_pH			: 1;
		unsigned ElectronicCal_Temp			: 1;
		unsigned FactoryMode				: 1;
		unsigned CalibrationMode			: 1;/*2/9/2021*/
		unsigned CODFlashOperation			: 1;
		unsigned AWADataSave_Calibration	: 1;//if not saved '1', else '0'
		unsigned AWADataSave_ProcessValues	: 1;
		unsigned CleaningTankEmpty			: 1;	/*<If SET then Set As Zero won't be accessible
		 	 	 	 	 	 	 	 	 	 	 	 	 Warning on the HMI*/
		unsigned MILSwitchState				: 1;
	};
}AWAOperationHandle_t;
AWAOperationHandle_t AWAOperationStatus_t;

/*Data to save in FRAM*/
typedef struct{
	unsigned factoryCOD 		: 1;	/*<SET after calibration and RESET after COD factory calibration stored in last calibration FRAM*/
	unsigned factoryTSS 		: 1;	/*<SET after calibration and RESET after TSS factory calibration stored in last calibration FRAM*/
	unsigned sensorCOD			: 1;	/*<SET after calibration and RESET after COD sensor calibration stored in last calibration FRAM*/
	unsigned sensorTSS			: 1;	/*<SET after calibration and RESET after TSS sensor calibration stored in last calibration FRAM*/
	unsigned sensorpH			: 1;	/*<SET after calibration and RESET after pH sensor calibration stored in last calibration FRAM*/
	unsigned electronicpH		: 1;	/*<SET after calibration and RESET after pH electronic calibration stored in last calibration FRAM*/
	unsigned electronicPT		: 1;	/*<SET after calibration and RESET after PT electronic calibration stored in last calibration FRAM*/
	unsigned electronicAO		: 1;	/*<SET after calibration and RESET after AO electronic calibration stored in last calibration FRAM*/
	unsigned factoryCOD_setzero	: 1;	/*<SET after setting zero and RESET after storing in FRAM*/
	unsigned factoryTSS_setzero	: 1;	/*<SET after setting zero and RESET after storing in FRAM*/
	unsigned analyzerRangeSelect: 1;	/*<SEt after selecting the analyzer range from the factory menu, will be reset for the first time.*/
	unsigned analyzerPocessvalue: 1;	/*<Set after calculating new process values ; COD,BOD,TSS,TOC*/
}AWADataStoreState_t;
AWADataStoreState_t	AWADataStoreState;

typedef struct{
	uint8_t factoryCOD_count;			/*<Stores the count of COD factory calibration last calibration stored in the FRAM*/
	uint8_t factoryTSS_count;			/*<Stores the count of TSS factory calibration last calibration stored in the FRAM*/
	uint8_t sensorCOD_count;			/*<Stores the count of COD sensor calibration last calibration stored in the FRAM*/
	uint8_t sensorTSS_count;			/*<Stores the count of TSS sensor calibration last calibration stored in the FRAM*/
	uint8_t sensorpH_count;				/*<Stores the count of pH sensor calibration last calibration stored in the FRAM*/
	uint8_t electronicpH_count;			/*<Stores the count of pH electronic calibration last calibration stored in the FRAM*/
	uint8_t electronicPT_count;			/*<Stores the count of PT electronic calibration last calibration stored in the FRAM*/
	uint8_t electronicAO_count;			/*<Stores the count of AO electronic calibration last calibration stored in the FRAM*/
	unsigned factoryCOD_overflowflag;	/*<Flag will be set to 1 when count exceeds 10*/
	unsigned factoryTSS_overflowflag;	/*<Flag will be set to 1 when count exceeds 10*/
	unsigned sensorCOD_overflowflag;	/*<Flag will be set to 1 when count exceeds 10*/
	unsigned sensorTSS_overflowflag;	/*<Flag will be set to 1 when count exceeds 10*/
	unsigned sensorpH_overflowflag;		/*<Flag will be set to 1 when count exceeds 10*/
	unsigned analyzerRangeSelectflag;	/*<SET when Range is selected from Factory menu, RESET when the analyzer is programmed for the first time*/
}AWALastCalibrationCount_t;
AWALastCalibrationCount_t AWALastCalibrationCount;

//typedef struct{
//	uint16_t pH_ADCCounts;
//	uint16_t pH_ElectronicCalib_p414V_count;
//	uint16_t pH_ElectronicCalib_n414V_count;
//	uint16_t flag_p414;
//	float pH_Slope;
//	float pH_Intercept;
//}ElectronicCalibpHHandle_t;
typedef union{
	uint8_t bytes[16];
	struct{
		uint16_t pH_ADCCounts;
		uint16_t pH_ElectronicCalib_p414V_count;
		uint16_t pH_ElectronicCalib_n414V_count;
		uint16_t flag_p414;
		float pH_Slope;//8
		float pH_Intercept;
	};
}ElectronicCalibpHHandle_t;
ElectronicCalibpHHandle_t pH_ElectronicCalibpoints_t;

typedef union{
	uint8_t byte[32];
	struct{
		//Stored in FRAM
		float pH_Solpe;					/*<Slope*/
		float pH_Intercept;				/*<Intercept*/
		float pH_Solution_1_y1;			/*<Simulation solution*/
		float pH_Solution_2_y2;			/*<Simulation solution*/
		int pH_counts_1_x1;				/*<Corresponding ADC counts*/
		int pH_counts_2_x2;				/*<Corresponding ADC counts*/
		float pH_Calculated_1_x1;		/*<Calculated pH value,(Display only)*/
		float pH_Calculated_2_x2;		/*<Calculated pH value,(Display only)*/
		//Not stored in FRAM
		uint16_t pH_1ptCalib_sample;	/*<NA*/
		uint16_t pH_1ptCalib_count;		/*<NA*/
		uint16_t flag_sample_set;		/*<From HMI, for storing the simulation pH solution value*/
		uint16_t flag_ADC_count_set;	/*<From HMI, for storing the pH solution corresponding ADC counts*/
		uint16_t flag_display_count;	/*<From HMI, for displaying the current raw value of the pH*/
		/*ADC counts*/
		uint16_t pH_ADCCounts;
	};
}SensorCalibrationHande_t;
SensorCalibrationHande_t pH_SensorCalibpoints_t;

/*For AO output*/
typedef union{
	float value[4];
	struct{
		float COD_Value;
		float BOD_Value;
		float TSS_Value;
		float pH_Value;
	};

}ParameterValueHandle_t;
ParameterValueHandle_t ParameterValues_t;

/*PT-100/PT-1000 Electronic Calibration handle*/
typedef union{
	uint8_t bytes[16];
	struct{
		float PT100_slope;
		float PT100_intercept;
		float PT1000_slope;
		float PT1000_intercept;
		float slope;
		float intercept;
		uint16_t ADCCounts;
		uint16_t PT_100_Counts;
		uint16_t PT_150_Counts;
		uint16_t PT_1000_Counts;
		uint16_t PT_1500_Counts;
		uint8_t flag_15;
	};
}ElectronicCalibration_PT_Handle_t;
ElectronicCalibration_PT_Handle_t PT_ElectronicCalibration_t;

typedef union{
	float byte[2];
	struct{
		float slope;
		float intercept;
		float zero_value; /*For the "set as zero" storage variable*/
		float x1;
		float y1;
		float x2;
		float y2;
		uint8_t point_flag;
	};
}COD_SensorCalibrationHandle_t;
COD_SensorCalibrationHandle_t COD_SensorCalibration_t;
COD_SensorCalibrationHandle_t TSS_SensorCalibration_t;

typedef struct{
	float RAW_Value;
	float Cal_Value;
	float PD1_Zero;
	float PD2_Zero;
	float PD1_New;
	float PD2_New;
	float PD1_Mean;
	float PD2_Mean;
	float RSD1;
	float RSD2;
}AWA_MeasurementHandle_t;
AWA_MeasurementHandle_t COD_MeasurementValues_t;
AWA_MeasurementHandle_t TSS_MeasurementValues_t;

typedef union{
	uint8_t bytes[10];
	struct{
		unsigned COD_ZERO	: 1;//1:if the Autozero or zero is done else 0
	};
}CODMeasurementStatusHandle_t;
CODMeasurementStatusHandle_t COD_MeasurementStatus;

typedef union{
	uint16_t Status;
	struct{
		unsigned AO_MotherBoardElectronicCalibrationStatus 	: 1;
		unsigned PH_ElectronicCalibrationStatus 			: 1;
		unsigned COD_ElectronicCalibrationStatus			: 1;
		unsigned PH_SensorCalibrationStatus					: 1;
		unsigned COD_SensorCalibrationStatus				: 1;
	};
}AWACalibrationStatusHandler_t;
AWACalibrationStatusHandler_t AWA_CalibrationStatus_t;

typedef union{
	uint8_t bytes[100];
	struct{
		float c[4];
		float x[10];
		float y[10];
		float SF;
	};
}FactoryCalibration10ptHandle_t;
FactoryCalibration10ptHandle_t COD_10ptFactoryCalibrationHandle_t;
FactoryCalibration10ptHandle_t TSS_10ptFactoryCalibrationHandle_t;

typedef union{
	uint8_t SlotIDarray[8];
	struct{
		uint8_t CARD_1;
		uint8_t CARD_2;
		uint8_t CARD_3;
		uint8_t CARD_4;
		uint8_t CARD_5;
		uint8_t CARD_6;
		uint8_t CARD_7;
		uint8_t CARD_8;
	};

}CardIDInfo_Handler_t;
CardIDInfo_Handler_t CardID_Info;
// Date-Time structure, used to store date-time values such as seconds, minutes etc...
typedef struct
{
    unsigned char second; // 0-59
    unsigned char minute; // 0-59
    unsigned char hour;   // 0-23
    unsigned char day;    // 1-31
    unsigned char month;  // 1-12
    unsigned char year;   // 0-99 (representing 2000-2099)
}
date_time_t;

// Days array
static unsigned short days[4][12] =
{
    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
    { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
    { 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
    {1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
