/*
 * Application.c
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */


#include "Application.h"
#include "main.h"
#include "math.h"
#include "AD7682.h"
#include "ADS1115.h"
#include "Calibration.h"
#include <stdlib.h>
#include <string.h>
#include "FM24CL64B.h"
#include <stdio.h>
#include "Initialization.h"
#include <time.h>

uint16_t CurrentTestValues_mA[5] = {4,8,12,16,20};

union fram_data
{
	float fData[2];
	uint8_t bytes[8];

};
union fram_data framdata;

uint8_t point = 0;

union{
	uint8_t raw_data[2];
	uint16_t data;
}raw_adc;

float volatge;

float adc;

/*External variables*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern uint8_t performAUTOZERO;
extern uint8_t performBatchCycle;
TIM_HandleTypeDef htim12;

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart6;
extern uint8_t uart_rx_buffer;						/*< Will receive single byte from UART and will transfer it to the RX buffer array.*/
extern volatile uint8_t uart_rx_bytes; 			/*< Will contain the received byte count.*/
extern volatile uint8_t uart_rcv_bytes;			/*< Flag will be set when the first byte will be received, will be served in the timer ISR*/
extern volatile uint16_t uart_rx_timeout_counter;
extern volatile uint8_t uart_rx_process_query;		/*< This flag will be set when the slave ID is correct and the query needs to be processed.*/

extern uint32_t budrate_9600;//19200;//9600;		/*<For testing, lateron will be replaced by the moodbus register.*/
extern uint32_t baudrate_uart1;		/*<For testing, lateron will be replaced by the moodbus register.*/
extern uint32_t baudrate_uart3;		/*<For testing, lateron will be replaced by the moodbus register.*/
extern float middle_point;
extern volatile uint8_t uart_rx_bytes_uart1; 		/*< Will contain the received byte count.*/
extern volatile uint16_t uart_rx_timeout_counter_uart1;

uint8_t cod_flash_operation;
uint16_t dataptr1;
uint16_t dataptr;
uint16_t ArrayPtr;
uint16_t noOfFlashes;
uint16_t flash_limit_max;
uint16_t flash_limit_min;

uint8_t COD_sensor_calibration_point;
uint8_t TSS_sensor_calibration_point;

struct Sensornode_3pt *CODSensorhead = '\0'; 	/*<Head for COD Sensor Calibration*/
struct Sensornode_3pt *TSSSensorhead = '\0'; 	/*<Head for TSS Sensor Calibration*/
struct Sensornode_3pt *pHSensorhead = '\0'; 	/*<Head for pH Sensor Calibration*/
struct Factorynode *CODFactoryhead = '\0'; 	/*<Head for COD Factory Calibration*/
struct Factorynode *TSSFactoryhead = '\0'; 	/*<Head for TSS Factory Calibration*/

QueueHandle_t *queue_;
/*
 * Analyzer range:- Parameter limit select
 */
float COD_UpperLimit;
float BOD_UpperLimit;
float TSS_UpperLimit;

MODBUS_config_t PORT_1;
MODBUS_config_t PORT_2;
MODBUS_config_t PORT_3;

uint8_t inProgress = RESET;
void ProcessModesCommands(void)
{
	if(HoldingRegister_t.ModeCommand_t.ModeCommand_H == RUN_MODE)
	{
		switch(HoldingRegister_t.ModeCommand_t.ModeCommand_L)
		{
			//Will change the commands, this just for testing
			case Batch:
			{
				/*RESET MESSAGES*/
				HoldingRegister_t.SensorCalibration_t.pHElectronicCalibMessages = RESET;
				/**/
				AWAOperationStatus_t.ElectronicCal_AO = 0;
				AWAOperationStatus_t.CalibrationMode = 0;
				AWAOperationStatus_t.FactoryMode = 0;
				//Auto measurement, background process
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == AUTO_COD_MEASURE
						&& AWAOperationStatus_t.MILSwitchState == SET
						&& (inProgress == SET || CoilStatusRegister_t.CoilStatus_t.Scheduler_interlock == SET))
				{
					inProgress = SET;
					//Change the next process time
					calculate_Next_ProcessTime();
					//First operate the sample pump
					PumpOperation(0x02);

					//perform the ADC measurement action when the pump action is completed
					if(!PUMPControlHandle_t.u8Flag_measurement)
						HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == AUTO_COD_MEASURE && AWAOperationStatus_t.MILSwitchState == RESET)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					pumpOperationStop();
				}
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CODADCCapture(COD_Measure);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && PUMPControlHandle_t.u8Flag_measurement)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				//Perform the Auto Zero process
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == AUTO_COD_ZERO
						&& (inProgress == SET || CoilStatusRegister_t.CoilStatus_t.Scheduler_interlock == SET))
				{
					inProgress = SET;
					//First operate the sample pump
					PumpOperation(0x01);

					//perform the ADC measurement action when the pump action is completed
					if(!PUMPControlHandle_t.u8Flag_measurement)
						HoldingRegister_t.ModeCommand_t.CommonCommand = COD_ZERO_MEAS;
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_ZERO_MEAS && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CODADCCapture(AUTO_COD_ZERO);
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == 0x97)
				{
					//Store the manual COD calibration factor

					//Get the sensor calibration factor from modus
					COD_SensorCalibration_t.slope = HoldingRegister_t.SensorCalibration_t.COD_CF;
					COD_SensorCalibration_t.intercept = HoldingRegister_t.SensorCalibration_t.COD_Intercept;


					/*<TODO: Push the slope and intercept to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.sensorCOD = SET;/*<TODO: to RESET after storing the data in FRAM*/
					Application_LastCaldataToModbus();
					/*----------------------------------------------------------------------*/

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == 0x96)
				{
					//Store the manual TSS calibration factor
					TSS_SensorCalibration_t.slope = HoldingRegister_t.SensorCalibration_t.TSS_CF;
					TSS_SensorCalibration_t.intercept = HoldingRegister_t.SensorCalibration_t.TSS_Intercept;

					/*<TODO: Push the slope and intercept to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.sensorTSS = SET;/*<TODO: to RESET after storing the data in FRAM*/
					Application_LastCaldataToModbus();

					/*----------------------------------------------------------------------*/

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == 0x95)
				{
					//Store the manual pH calibration factor
					pH_SensorCalibpoints_t.pH_Solpe = HoldingRegister_t.SensorCalibration_t.pH_slope;
					pH_SensorCalibpoints_t.pH_Intercept = HoldingRegister_t.SensorCalibration_t.pH_intercept;
					pH_SensorCalibpoints_t.pH_slope_range_2 = HoldingRegister_t.SensorCalibration_t.pH_slope_range_2;
					pH_SensorCalibpoints_t.pH_Intercept_range_2 = HoldingRegister_t.SensorCalibration_t.pH_intercept_range_2;

					/*<TODO: Push the slope and intercept to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.sensorpH = SET;/*<TODO: to RESET after storing the data in FRAM*/
					Application_LastCaldataToModbus();
					/*----------------------------------------------------------------------*/

					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
					//AWAOperationStatus_t.CalibrationMode = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

				}
				else{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
					CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
					//HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
				}
				break;
			}
			case External:
			{
				break;
			}
			case Stopped:
			{
				break;
			}
		}
	}
	//SETTINGS MODE
	else if(HoldingRegister_t.ModeCommand_t.ModeCommand_H == SETTING_CONFIG_MODE)
	{
		switch(HoldingRegister_t.ModeCommand_t.ModeCommand_L)
		{
			case Save:
			{
				ModbusSaveConfiguration(SAVE_CONFIGURATION_DATA);
#if 0
				uint32_t epoch = 1648645808;//HoldingRegister_t.ModeCommand_t.Epoch_Timestamp;
				epoch_to_date_time(&systemTime,epoch);

				//Next process time
				int next_batch_hr = HoldingRegister_t.SensorCalibration_t.NextProcessTime_Hr;
				int next_batch_min = HoldingRegister_t.SensorCalibration_t.NextProcessTime_Min;
				int next_batch_sec = HoldingRegister_t.SensorCalibration_t.NextProcessTime_Sec;

				next_batch_hr *= 3600;
				next_batch_min *= 60;
				epoch += (next_batch_hr + next_batch_min + next_batch_sec);

				if(next_batch_hr + systemTime.hour > 24)
				{
					int pre24_delta = 24 - systemTime.hour;
					int post24_delta = abs(next_batch_hr - pre24_delta);
					next_batch_hr = post24_delta;
//					systemTime.day += 2;//not required

				}else if(next_batch_hr + systemTime.hour == 24)
				{
					next_batch_hr = 24;
				}
				else{
					next_batch_hr += systemTime.hour;
				}

				if(next_batch_min + systemTime.minute > 60)
				{
					int pre24_delta = 60 - systemTime.minute;
					int post24_delta = abs(next_batch_min - pre24_delta);
					next_batch_min = post24_delta;
					next_batch_hr += 1;

				}else{
					next_batch_min += systemTime.minute;
				}

				if(next_batch_sec + systemTime.second > 60)
				{
					int pre24_delta = 60 - systemTime.second;
					int post24_delta = abs(next_batch_sec - pre24_delta);
					next_batch_sec = post24_delta;
					next_batch_sec += 1;

				}else{
					next_batch_sec += systemTime.second;
				}


				date_time_t ti;
				memset(&ti,'\0',sizeof(ti));
				ti.hour = next_batch_hr;//HoldingRegister_t.SensorCalibration_t.NextProcessTime_Hr;
				ti.second = next_batch_sec;//HoldingRegister_t.SensorCalibration_t.NextProcessTime_Sec;
				ti.minute = next_batch_min;//HoldingRegister_t.SensorCalibration_t.NextProcessTime_Min;
				ti.day = systemTime.day;
				ti.month = systemTime.month;
				ti.year = systemTime.year;

				uint32_t next_cal_time = date_time_to_epoch(&ti);

				HoldingRegister_t.SensorCalibration_t.NextProcessTime = epoch;//next_cal_time;// + 1648629116;//HoldingRegister_t.ModeCommand_t.Epoch_Timestamp;
#endif
				HoldingRegister_t.ModeCommand_t.ModeCommand_H = 0x23;
				HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x10;

				HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
				break;
			}
			case SchedulerSave:
			{
				//ModbusSaveConfiguration(SAVE_CONFIGURATION_DATA);

				calculate_Next_ProcessTime();

				HoldingRegister_t.ModeCommand_t.ModeCommand_H = 0x23;
				HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x00;

				HoldingRegister_t.ModeCommand_t.ModeCommandHMI_H = 0x23;
				HoldingRegister_t.ModeCommand_t.ModeCommandHMI_L = 0x00;

				HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
				break;
			}
			case Calibrate_PH:/*31/8/2021*/
			{

				AWAOperationStatus_t.CalibrationMode = 0x01;//change to setting mode
				PH_SEL2_ON(CARD_SLOT_6);
#if 0
				uint16_t sample_no = 50;
				uint64_t adc_temp = 0;
				uint16_t adc_val = 0;
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
				pH_SensorCalibpoints_t.pH_ADCCounts = (float)adc_temp / (float)sample_no;
#endif
				pH_SensorCalibpoints_t.pH_ADCCounts = ADS1115_OperationpHMeasurement(0x01);
				InputRegister_t.SlotParameter.pH_ADC_Counts = pH_SensorCalibpoints_t.pH_ADCCounts;
				if(pH_SensorCalibpoints_t.pH_ADCCounts > 0x7fff)
					InputRegister_t.SlotParameter.pH_Live_Volatge = pH_ElectronicCalibpoints_t.pH_Slope * (pH_SensorCalibpoints_t.pH_ADCCounts - 65535.0f) + pH_ElectronicCalibpoints_t.pH_Intercept;
				else
					InputRegister_t.SlotParameter.pH_Live_Volatge = pH_ElectronicCalibpoints_t.pH_Slope * pH_SensorCalibpoints_t.pH_ADCCounts + pH_ElectronicCalibpoints_t.pH_Intercept;
				pH_SensorCalibrationGetValues();
#if 0
				//-------------------------1 point calibration--------------------------------------//
				if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x01)
				{
					if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x01)//set sample
					{
						pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_1pt_calib_point1;
						pH_SensorCalibpoints_t.flag_display_count = 0x01;
						HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
					}
					if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x02)//set ADC counts
					{
						pH_SensorCalibpoints_t.pH_counts_1_x1 = pH_SensorCalibpoints_t.pH_ADCCounts;
						HoldingRegister_t.SensorCalibration_t.pH_1pt_Cal_point_1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;
						pH_SensorCalibpoints_t.flag_display_count = 0x03;//no action on display value
						HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
					}
					/*For HMI display*/
					if(pH_SensorCalibpoints_t.flag_display_count == 0x01)
					{
						//Read by HMI
						HoldingRegister_t.SensorCalibration_t.pH_1pt_Cal_point_1_count = pH_SensorCalibpoints_t.pH_ADCCounts;

					}
				}
				//----------------------------------------------------------------------------------//
				//------------------------ 2 point calibration--------------------------------------//
				if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x02)
				{
					if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x01)//set sample
					{
						if(!pH_SensorCalibpoints_t.flag_sample_set)
						{
							pH_SensorCalibpoints_t.flag_sample_set = 0x01;
							pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y1;
							pH_SensorCalibpoints_t.flag_display_count = 0x01;
						}else if(pH_SensorCalibpoints_t.flag_sample_set == 0x01)
						{
							pH_SensorCalibpoints_t.flag_sample_set = 0x0;
							pH_SensorCalibpoints_t.pH_Solution_2_y2 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y2;
							pH_SensorCalibpoints_t.flag_display_count = 0x02;
						}
						HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
					}
					if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x02)//set ADC counts
					{
						if(!pH_SensorCalibpoints_t.flag_ADC_count_set)
						{
							pH_SensorCalibpoints_t.flag_ADC_count_set = 0x01;
							pH_SensorCalibpoints_t.pH_counts_1_x1 = pH_SensorCalibpoints_t.pH_ADCCounts;
							HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;
							pH_SensorCalibpoints_t.flag_display_count = 0x03;//no action on display value
						}else if(pH_SensorCalibpoints_t.flag_ADC_count_set == 0x01)
						{
							pH_SensorCalibpoints_t.flag_ADC_count_set = 0x0;
							pH_SensorCalibpoints_t.pH_counts_2_x2 = pH_SensorCalibpoints_t.pH_ADCCounts;
							HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_counts_2_x2;
							pH_SensorCalibpoints_t.flag_display_count = 0x03;//no action on display value
						}
						HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
					}
					/*For HMI display*/
					if(pH_SensorCalibpoints_t.flag_display_count == 0x01)
					{
						//Read by HMI
						HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_ADCCounts;
						HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;
					}else if(pH_SensorCalibpoints_t.flag_display_count == 0x02)
					{
						//Read by HMI
						HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_ADCCounts;
					}
				}
				//---------------------------- 2 point calibration end--------------------------------------------//
#endif

				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Sensor_Calibration_pH)
				{
					pHSensorCalibrationmV();

					/*<TODO: Push the slope and intercept to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.sensorpH = SET;/*<TODO: to RESET after storing the data in FRAM*/
					Application_LastCaldataToModbus();
					/*----------------------------------------------------------------------*/

					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
					AWAOperationStatus_t.CalibrationMode = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

					HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x50;
				}else{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				break;
			}
			case Calibrate_COD:
			{
				//Measure Zero
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt1)
				{
					COD_SensorCalib_ypoint(1);
					COD_sensor_calibration_point = 1;
				}
				//Measure Sample 1
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt2)
				{
					COD_SensorCalib_ypoint(2);
					COD_sensor_calibration_point = 2;
				}
				//Measure sample 2
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt3)
				{
					COD_SensorCalib_ypoint(3);
					COD_sensor_calibration_point = 3;
				}

				/*Stop sampling pump, if MIL state is RESET*/
				else if((HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt1
					|| HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt2
					|| HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt3)
					&& AWAOperationStatus_t.MILSwitchState == RESET)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					pumpOperationStop();
				}

				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					//CODADCCapture(COD_Measure);
					CODADCCapture_Sensor(COD_Measure,point);
#endif
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == STOP_RUNNING_PUMP)
				{
					pumpOperationStop();
					//Delay
					HAL_Delay(500);
					//perform the ADC measurement action when the pump action is completed
					if(!PUMPControlHandle_t.u8Flag_measurement)
					{
						HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
						point = COD_sensor_calibration_point;
					}
				}
				//For Two point calibration operation
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == Sensor_Calibrate_COD)
				{
					COD_SensorCalibration();
					/*Re-calculate the values of COD from the new 2-pt calibration*/
					CODCalculateValue();
					/*<TODO: Push the slope and intercept to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.sensorCOD = SET;/*<TODO: to RESET after storing the data in FRAM*/
					Application_LastCaldataToModbus();
					/*----------------------------------------------------------------------*/

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

				}else{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				break;
			}
			case Calibrate_TSS:
			{
				//Measure Zero
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt1)
				{
					COD_SensorCalib_ypoint(4);
					TSS_sensor_calibration_point = 4;

				}
				//Measure Sample 1
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt2)
				{
					COD_SensorCalib_ypoint(5);
					TSS_sensor_calibration_point = 5;
				}
				//Measure sample 2
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt3)
				{
					COD_SensorCalib_ypoint(6);
					TSS_sensor_calibration_point = 6;
				}

				/*Stop sampling pump, if MIL state is RESET*/
				else if((HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt1
					|| HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt2
					|| HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt3)
					&& AWAOperationStatus_t.MILSwitchState == RESET)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					pumpOperationStop();
				}

				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					//CODADCCapture(COD_Measure);
					CODADCCapture_Sensor(COD_Measure,point);
#endif
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == STOP_RUNNING_PUMP)
				{
					pumpOperationStop();
					//Delay
					HAL_Delay(500);
					//perform the ADC measurement action when the pump action is completed
					if(!PUMPControlHandle_t.u8Flag_measurement)
					{
						HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
						point = TSS_sensor_calibration_point;
					}
				}
				//For Two point calibration operation
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == Sensor_Calibrate_TSS)
				{
					TSS_SensorCalibration();
					/*Calculate the new TSS value from the new 2-pt calibration*/
					TSScalculateValue();
					/*<TODO: Push the slope and intercept to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.sensorTSS = SET;/*<TODO: to RESET after storing the data in FRAM*/
					Application_LastCaldataToModbus();

					/*----------------------------------------------------------------------*/

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

				}else{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				break;
			}
			case MODBUS_Config:
			{
				//HMI
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == configure_mosbus_port1)
				{
					MODBUS_config_t temp_config;
					temp_config.baudrate = HoldingRegister_t.MODBUS_PORTConfig_t.PORT_1.baudrate;
					temp_config.data_length = HoldingRegister_t.MODBUS_PORTConfig_t.PORT_1.data_length;
					temp_config.parity_bit = HoldingRegister_t.MODBUS_PORTConfig_t.PORT_1.parity_bit;
					temp_config.stop_bits = HoldingRegister_t.MODBUS_PORTConfig_t.PORT_1.stop_bits;
					temp_config.slave_ID = HoldingRegister_t.MODBUS_PORTConfig_t.PORT_1.slave_ID;
					MODBUSConfiguration(&huart6, temp_config);
				}
				//RTU
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == configure_mosbus_port2)
				{
					//Copy the data from Holding register to live buffer
					memcpy(&PORT_2,&HoldingRegister_t.MODBUS_PORTConfig_t.PORT_2,sizeof(HoldingRegister_t.MODBUS_PORTConfig_t.PORT_2));

					//Select the baudrate according to the selection
					baudrateSelection(&PORT_2);

					//Publish the selected baudrate value on the MODBUS
					HoldingRegister_t.MODBUS_PORTConfig_t.PORT_2.baudrate = PORT_2.baudrate;

					//Reset the common command
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;

					//Stop the MODBUS communication
					resetMODBUSComm(&huart1, &htim5);

					//Configure the new MODBUS configuration
					MODBUSConfiguration(&huart1, PORT_2);

					//Start the MODBUS communication
					beginMODBUSComm(&htim5, &uart_rx_timeout_counter_uart1, &uart_rx_bytes_uart1, &RxFlag,2);

					AWADataStoreState.modbusConfigport2 = SET;

					AWAOperationStatus_t.AWADataSave_Calibration = SET;
				}
				//CPCB
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == configure_mosbus_port3)
				{

					//Copy the data from Holding register to live buffer
					memcpy(&PORT_3,&HoldingRegister_t.MODBUS_PORTConfig_t.PORT_3,sizeof(HoldingRegister_t.MODBUS_PORTConfig_t.PORT_3));

					//Select the baudrate according to the selection
					baudrateSelection(&PORT_3);

					//Publish the selected baudrate value on the MODBUS
					HoldingRegister_t.MODBUS_PORTConfig_t.PORT_3.baudrate = PORT_3.baudrate;

					//Reset the common command
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;

					//Stop the MODBUS communication
					resetMODBUSComm(&huart3, &htim12);

					//Configure the new MODBUS configuration
					MODBUSConfiguration(&huart3, PORT_3);

					//Start the MODBUS communication
					beginMODBUSComm(&htim12, &uart_rx_timeout_counter_uart3, &uart_rx_bytes_uart3, &MOD3_RxFlag,3);

					AWADataStoreState.modbusConfigport3 = SET;

					AWAOperationStatus_t.AWADataSave_Calibration = SET;
				}
			}
#if 0
			case Calibrate_COD:
			{
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					CODADCCapture(COD_Measure);
#endif
				}
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION)
				{
					//operate the cleaning pump
					PumpOperation(0x01);
				}
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION)
				{
					//operate the sample pump
					PumpOperation(0x02);
				}
				//For Two point calibration operation
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Sensor_Calibrate_COD)
				{
					COD_SensorCalibration();
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}
				// for x1 and y1
				if(cod_flash_operation == 2 && COD_SensorCalibration_t.point_flag == 0)
				{
					//Get the use entry point from the HMI
					HoldingRegister_t.SensorCalibration_t.COD_X1 = HoldingRegister_t.SensorCalibration_t.Set_Point_buff;
					//Put the X1 value
					COD_SensorCalibration_t.x1 = HoldingRegister_t.SensorCalibration_t.COD_X1;
					//Y1 value will be the COD_RAW
					COD_SensorCalibration_t.y1 = InputRegister_t.PV_info.COD_RAW;
					//sent the flag to 1 for getting the 2nd point
					COD_SensorCalibration_t.point_flag = 1;
					//reset the flash operation flag
					cod_flash_operation = 1;
					//Reset the set point buffer
					HoldingRegister_t.SensorCalibration_t.Set_Point_buff = 0;
				}
				//for x2 and y2
				if(cod_flash_operation == 2 && COD_SensorCalibration_t.point_flag == 1)
				{
					//Get the use entry point
					HoldingRegister_t.SensorCalibration_t.COD_X2 = HoldingRegister_t.SensorCalibration_t.Set_Point_buff;
					//Put the x2 value
					COD_SensorCalibration_t.x2 = HoldingRegister_t.SensorCalibration_t.COD_X2;
					//y2 value will the COD RAW
					COD_SensorCalibration_t.y1 = InputRegister_t.PV_info.COD_RAW;
					//sent the flag to 1 to reset the value getting process
					COD_SensorCalibration_t.point_flag = 1;
					//reset the flash operation flag
					cod_flash_operation = 1;
					//Reset the set point buffer
					HoldingRegister_t.SensorCalibration_t.Set_Point_buff = 0;
				}
				//Capture the offset COD_RAW value "Set as zero"
				if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x03)
				{
					//Store the zero value offset which will be then subtracted from the COD_RAW
					COD_SensorCalibration_t.zero_value = COD_MeasurementValues_t.RAW_Value;

					//Remove the offset from the cod raw value
					COD_MeasurementValues_t.RAW_Value = COD_MeasurementValues_t.RAW_Value - COD_SensorCalibration_t.zero_value;

					//publish to the MODBUS

					//Reset the common command flag
					HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
				}
				break;
			}
#endif
		}
	}
	//FACTORY MODE
	else if(HoldingRegister_t.ModeCommand_t.ModeCommand_H == FACTORY_MODE)
	{
		//AWAOperationStatus_t.FactoryMode = 0x01;
		switch (HoldingRegister_t.ModeCommand_t.ModeCommand_L)
		{
		static uint8_t count = 0;
			case Electronics_Calibration_mV: //page
			{
				count += 1;
				//AWAOperationStatus_t.FactoryMode = 0x01;
				PH_SEL2_ON(CARD_SLOT_6);
#if 0
				uint16_t sample_no = 50;
				AWAOperationStatus_t.FactoryMode = 0x01;
				uint64_t adc_temp = 0;
				uint16_t adc_val = 0;
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
#endif
				if(count >= 10)
				{
					pH_ElectronicCalibpoints_t.pH_ADCCounts = ADS1115_OperationpHMeasurement(0x01);//(float)adc_temp / (float)sample_no;
					count = 0;
				}
				InputRegister_t.SlotParameter.pH_ADC_Counts = pH_ElectronicCalibpoints_t.pH_ADCCounts;
				InputRegister_t.SlotParameter.pH_Live_Volatge = 0.0f;


					pH_ElectronicCalibrationGetValues();


#if 0
				if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command == 9)
				{
					if(!pH_ElectronicCalibpoints_t.flag_p414)
					{
						pH_ElectronicCalibpoints_t.pH_ElectronicCalib_p414V_count = pH_ElectronicCalibpoints_t.pH_ADCCounts;
						HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_p414mA = pH_ElectronicCalibpoints_t.pH_ElectronicCalib_p414V_count;
						pH_ElectronicCalibpoints_t.flag_p414 = 1;
						HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
					}
					else if(pH_ElectronicCalibpoints_t.flag_p414)
					{
						pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count = pH_ElectronicCalibpoints_t.pH_ADCCounts;
						HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_n414mA = pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count;
						pH_ElectronicCalibpoints_t.flag_p414 = 0;
						HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
					}
				}
#endif
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Electronics_Calibration_mV_Calibrate)//change the command
				{
					pHElectronicCalibrationmV();
//					HoldingRegister_t.SensorCalibration_t.pHElecMeaages = 4;//Calibrated
					/*<TODO: Push the slope and intercept to Last Calibration Input register*/

					/*----------------------------------------------------------------------*/
					AWAOperationStatus_t.FactoryMode = 0x0;
					//Calibration successful
					HoldingRegister_t.SensorCalibration_t.pHElectronicCalibMessages = CALIBRATION_SUCCESSFULL;
					//Flag set as data not saved in the FRAM
					//AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
					//Set the state of which data to store
					//AWADataStoreState.electronicpH = SET;/*<TODO: to RESET after storing the data in FRAM*/
					//HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x50;
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
					HoldingRegister_t.ModeCommand_t.CommonCommandHMI = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

					AWADataStoreState.electronicpH = SET;/*<TODO: to RESET after storing the data in FRAM*/
					break;
				}else{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				break;
			}
			case Electronics_Calibration_I_Out:
			{
				//Enter into Calibration function
				if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Common_Command == 0x03)
				{
					uint8_t channel = HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command;
					HoldingRegister_t.IOUTCalibandTest_t.CurrentOPPoints_t[channel-1].Current_OP_Calib_Point1 = HoldingRegister_t.IOUTCalibandTest_t.AO_5mA_temp_value;
					HoldingRegister_t.IOUTCalibandTest_t.CurrentOPPoints_t[channel-1].Current_OP_Calib_Point2 = HoldingRegister_t.IOUTCalibandTest_t.AO_19mA_temp_value;
					HoldingRegister_t.IOUTCalibandTest_t.AO_5mA_temp_value = 0;
					HoldingRegister_t.IOUTCalibandTest_t.AO_19mA_temp_value = 0;
					CurrentOutputCalibration(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command);

					/*<TODO: Push the slope and intercept to Last Calibration Input register*/

					/*----------------------------------------------------------------------*/

					/*18-8-2021*/
					HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
					//calFlag = 0;/*30/8/2021*/
					AWAOperationStatus_t.ElectronicCal_AO = 0x0;
					//New calibration data is not saved
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
					//Set the state of which data to store
					AWADataStoreState.electronicAO = SET;/*<TODO: to RESET after storing the data in FRAM*/
				}
				else if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Common_Command > 0\
						&&HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Common_Command  < 3)
				{
					Current_OP_Calibrate(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command,\
						HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Common_Command);

					//This flag is set to '1' so it does not give current output in the main loop for the configured parameters during the calibration
					AWAOperationStatus_t.ElectronicCal_AO = 0x1;
				}
//				HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
				HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Common_Command = 0;
				break;
			}
			case I_Out_test:/*30/8/2021*/
			{
				if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Test_Command)
				{
					AWAOperationStatus_t.ElectronicCal_AO = 1;
					currentOutputTest(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command);
					HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Test_Command = 0;
					//Set the Flag for FRAM to store the AO Electronics calibration data
					//AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}
				break;
			}
			case Electronics_Calibration_PT_100:
			{
				AWAOperationStatus_t.FactoryMode = 0x01;
#if 0
				uint16_t sample_no = 50;

				uint64_t adc_temp = 0;
				uint16_t adc_val = 0;
				ADS1115_OperationInit_Temperature();
				for(int i = 0;i<sample_no;i++)
				{
					ADS1115_OperationStartConversion();
					HAL_Delay(2);
					adc_val = ADS1115_OperationADCRead();
					raw_adc.raw_data[0] = (adc_val >> 8);
					raw_adc.raw_data[1] = adc_val;
					adc_temp += raw_adc.data;
				}
#endif
				//Software PT100 select SEL1 pin HIGH
				if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)//PT100
					//SEL1_PT100();
					PH_SEL1_ON(CARD_SLOT_6);
				//Software PT1000 select SEL1 pin LOW
				else
					//SEL1_PT1000();
					PH_SEL1_OFF(CARD_SLOT_6);
				PT_ElectronicCalibration_t.ADCCounts = ADS1115_OperationpHMeasurement(0x02);//(float)adc_temp / (float)sample_no;
//				adc_temp = 0;
				InputRegister_t.SlotParameter.pH_Temperature_ADC_Counts = PT_ElectronicCalibration_t.ADCCounts;
				InputRegister_t.SlotParameter.pH_Temperature_Live_Voltage = 0.0f;
				PT_ElectronicCalibrationGetValues();
#if 0
				if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command == 10)
				{
					if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)//PT100
					{
						if(!PT_ElectronicCalibration_t.flag_15)
						{
							PT_ElectronicCalibration_t.flag_15 = 0x01;
							PT_ElectronicCalibration_t.PT_100_Counts = PT_ElectronicCalibration_t.ADCCounts;
							HoldingRegister_t.IOUTCalibandTest_t.PT100_ADC_Counts_100  = PT_ElectronicCalibration_t.PT_100_Counts;
						}else if(PT_ElectronicCalibration_t.flag_15 == 0x01)
						{
							PT_ElectronicCalibration_t.flag_15 = 0x0;
							PT_ElectronicCalibration_t.PT_150_Counts = PT_ElectronicCalibration_t.ADCCounts;
							HoldingRegister_t.IOUTCalibandTest_t.PT100_ADC_Counts_150  = PT_ElectronicCalibration_t.PT_150_Counts;

						}
					}else if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT1000)//PT100
					{
						if(!PT_ElectronicCalibration_t.flag_15)
						{
							PT_ElectronicCalibration_t.flag_15 = 0x01;
							PT_ElectronicCalibration_t.PT_1000_Counts = PT_ElectronicCalibration_t.ADCCounts;
							HoldingRegister_t.IOUTCalibandTest_t.PT1000_ADC_Counts_1000 = PT_ElectronicCalibration_t.PT_1000_Counts;
						}else if(PT_ElectronicCalibration_t.flag_15 == 0x01)
						{
							PT_ElectronicCalibration_t.flag_15 = 0x0;
							PT_ElectronicCalibration_t.PT_1500_Counts = PT_ElectronicCalibration_t.ADCCounts;
							HoldingRegister_t.IOUTCalibandTest_t.PT1000_ADC_Counts_1500  = PT_ElectronicCalibration_t.PT_1500_Counts;

						}
					}
					HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
				}
#endif
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Electronic_Calibration_PT)
				{
					PT_ElectronicCalibration();

					/*<TODO: Push the slope and intercept to Last Calibration Input register*/

					/*----------------------------------------------------------------------*/
					//Calibration successful
					HoldingRegister_t.SensorCalibration_t.TempElectronicCalibMessages = CALIBRATION_SUCCESSFULL;
					AWAOperationStatus_t.FactoryMode = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
					//Set the state of which data to store
					AWADataStoreState.electronicPT = SET;/*<TODO: to RESET after storing the data in FRAM*/
					//HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x50;
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
					HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
				}else{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				break;
			}
			case CODCoeffs:
			{
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Factory_Calibrate_COD)
				{
					double x_mat[10],y_mat[10];
					for(int i =0;i<10;i++)
					{
						x_mat[i] = HoldingRegister_t.ModeCommand_t.COD_X[i];
						y_mat[i] = HoldingRegister_t.ModeCommand_t.COD_Y[i];
					}

					for(int i = 0;i<10;i++)
					{
						COD_10ptFactoryCalibrationHandle_t.x[i] = x_mat[i];
						COD_10ptFactoryCalibrationHandle_t.y[i] = y_mat[i];
					}

					//gaussEliminationLS(x_mat,y_mat,10,3,1);
					gaussEliminationLS(x_mat,y_mat,10,2,1);

					//Display messages
					HoldingRegister_t.SensorCalibration_t.COD_Factory_Messages = CALIBRATION_SUCCESSFULL;

					/*<TODO: Push the Coefficients to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.factoryCOD = SET;
					Application_LastCaldataToModbus();
					/*---------------------------------------------------------------*/
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;


					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				/*Display messages*/
//				if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION
//						|| HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION
//						||HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure )
//				{
//					HoldingRegister_t.SensorCalibration_t.COD_Factory_Messages = RESET;
//				}
				/**/
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					CODADCCapture(COD_Measure);
#endif
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && PUMPControlHandle_t.u8Flag_measurement)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION)
				{
					//operate the cleaning pump
					PumpOperation(0x01);
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == SET)
				{
					//operate the sample pump
					PumpOperation(0x02);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == RESET)
				{
					pumpOperationStop();
#if 0
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					//Turn of the pump 2
					PUMP2_OFF();
					//PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
					//Reset the HMI inter-locking Flag
					HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP,RESET);
					//Disable interrupt
//					htim6.Instance->DIER &= ~TIM_IT_UPDATE; // disable timer interrupt flag
					//htim6.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer
					//htim6.Instance->SR = 0;
					if(PUMPControlHandle_t.u8Flag_measurement == SET)
					{
						HAL_TIM_Base_Stop_IT(&htim6);
						HAL_TIM_Base_DeInit(&htim6);

						PUMPControlHandle_t.u8Flag_measurement = RESET;
					}
					//reset all the action
					PUMPControlHandle_t.u16TIM6_count = 0;
					PUMPControlHandle_t.u16TIM6_limit = 0;
					PUMPControlHandle_t.u8PUMP_action = 1;
					PUMPControlHandle_t.u8PUMP_delayaction = 0;
					PUMPControlHandle_t.u8PUMP_no = 0;
					//un-block the ADC measurement
					PUMPControlHandle_t.u8Flag_measurement = 0x00;
#endif

				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty == RESET)
				{
					//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
					COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
					COD_MeasurementValues_t.PD2_Zero = COD_MeasurementValues_t.PD2_New;

					//Calculate the COD RAW
					float COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/COD_MeasurementValues_t.PD1_New) - log(COD_MeasurementValues_t.PD2_Zero/COD_MeasurementValues_t.PD2_New));
					//Publish the values to the MODBUS
					InputRegister_t.PV_info.COD_RAW = COD_RAW;
					InputRegister_t.PV_info.PD1_0 = COD_MeasurementValues_t.PD1_Zero;
					InputRegister_t.PV_info.PD2_0 = COD_MeasurementValues_t.PD2_Zero;
					//Reset the command
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					AWADataStoreState.factoryCOD_setzero = SET;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty == SET)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				/*Emergency Pump Stop*/
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == STOP_RUNNING_PUMP)
					pumpOperationStop();
				else
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				break;
			}
			case TSSCoeffs:
			{
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Factory_Calibrate_TSS)
				{
					double x_mat[10],y_mat[10];
					for(int i =0;i<10;i++)
					{
						//x - F , y - G
						x_mat[i] = HoldingRegister_t.ModeCommand_t.TSS_F[i];
						y_mat[i] = HoldingRegister_t.ModeCommand_t.TSS_G[i];
						//Lab - y, Online - x
//						x_mat[i] = HoldingRegister_t.ModeCommand_t.TSS_G[i];
//						y_mat[i] = HoldingRegister_t.ModeCommand_t.TSS_F[i];
					}

					for(int i = 0;i<10;i++)
					{
						TSS_10ptFactoryCalibrationHandle_t.x[i] = x_mat[i];
						TSS_10ptFactoryCalibrationHandle_t.y[i] = y_mat[i];
					}

					gaussEliminationLS(x_mat,y_mat,10,2,2);

					//Display messages
					HoldingRegister_t.SensorCalibration_t.TSS_Factory_Messages = CALIBRATION_SUCCESSFULL;

					/*<TODO: Push the Coefficients to Last Calibration Input register*/
					//Set the state of which data to store
					AWADataStoreState.factoryTSS = SET;
					Application_LastCaldataToModbus();
					/*---------------------------------------------------------------*/

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;


					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				/*Display messages*/
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION
						|| HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION
						||HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure )
				{
					HoldingRegister_t.SensorCalibration_t.TSS_Factory_Messages = RESET;
				}
				/**/
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CODADCCapture(TSS_Measure);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure && PUMPControlHandle_t.u8Flag_measurement)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION)
				{
					//operate the cleaning pump
					PumpOperation(0x01);
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == SET)
				{
					//operate the sample pump
					PumpOperation(0x02);
					//HoldingRegister_t.SensorCalibration_t.TSS_Factory_Messages = SAMPLING_PUMP_STOPPED;
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == RESET)
				{
					pumpOperationStop();
					//HoldingRegister_t.SensorCalibration_t.TSS_Factory_Messages = CLEANING_PUMP_STOPPED;
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty == RESET)
				{
					//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
					//COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
					TSS_MeasurementValues_t.PD2_Zero = TSS_MeasurementValues_t.PD2_New;

					//Calculate the COD RAW
					float TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/TSS_MeasurementValues_t.PD2_New));
					//Publish the values to the MODBUS
					InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
					InputRegister_t.PV_info.TSS_PD2_0 = TSS_MeasurementValues_t.PD2_Zero;
					//Reset the command
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					AWADataStoreState.factoryTSS_setzero = SET;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}
				/*Emergency Pump Stop*/
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == STOP_RUNNING_PUMP)
					pumpOperationStop();
				else
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				break;
			}
			case Relay_test:
			{
				AWAOperationStatus_t.ElectronicCal_AO = 1;
				RelayToggle();
				//RelayToggleCoilInputUpdate();
				break;
			}
			case Select_Range:
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == 0x98)
				{
					AWA_RangeSelect();
					//Reset the common command
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
					//Store the data in FRAM
					AWADataStoreState.analyzerRangeSelect = SET;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}else
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;

			break;
		}
	}
	else if(HoldingRegister_t.ModeCommand_t.ModeCommand_H == CHECK_MODE)
	{
		switch(HoldingRegister_t.ModeCommand_t.ModeCommand_L)
		{
			case COD_Check:
			{
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CheckScreen_CODADCCapture(COD_Measure);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && PUMPControlHandle_t.u8Flag_measurement)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION)
				{
					//operate the cleaning pump
					CheckScreen_PumpOperation(0x01);
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == SET)
				{
					//operate the sample pump
					CheckScreen_PumpOperation(0x02);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == RESET)
				{
					pumpOperationStop();
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty == RESET)
				{
					//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
					COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
					COD_MeasurementValues_t.PD2_Zero = COD_MeasurementValues_t.PD2_New;

					//Calculate the COD RAW
					float COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/COD_MeasurementValues_t.PD1_New) - log(COD_MeasurementValues_t.PD2_Zero/COD_MeasurementValues_t.PD2_New));
					//Publish the values to the MODBUS
					InputRegister_t.PV_info.COD_RAW = COD_RAW;
					InputRegister_t.PV_info.PD1_0 = COD_MeasurementValues_t.PD1_Zero;
					InputRegister_t.PV_info.PD2_0 = COD_MeasurementValues_t.PD2_Zero;
					//Reset the command
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					AWADataStoreState.factoryCOD_setzero = SET;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty != RESET)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				/*Emergency Pump Stop*/
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == STOP_RUNNING_PUMP)
					pumpOperationStop();
				else
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
					CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
				}
				break;
			}
			case TSS_Check:
			{
//				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Read_acid)
//				{
//					Application_ReadAcid();
//				}
//				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Read_sample)
//				{
//					Application_ReadSample();
//				}
//				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
//				if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure && !PUMPControlHandle_t.u8Flag_measurement)
//				{
//					CODADCCapture(TSS_Measure);
//				}
//				//Set as zero
//				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_FACTORY_SETASZERO
//						&& AWAOperationStatus_t.CleaningTankEmpty == RESET)
//				{
//					Application_SetAsZero(2);
//				}
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CheckScreen_CODADCCapture(TSS_Measure);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure && PUMPControlHandle_t.u8Flag_measurement)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP1_ACTION)
				{
					//operate the cleaning pump
					CheckScreen_PumpOperation(0x01);
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == SET)
				{
					//operate the sample pump
					CheckScreen_PumpOperation(0x02);
				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == PUMP2_ACTION && AWAOperationStatus_t.MILSwitchState == RESET)
				{
					pumpOperationStop();
				}
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty == RESET)
				{
					//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
					//COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
					TSS_MeasurementValues_t.PD2_Zero = TSS_MeasurementValues_t.PD2_New;

					//Calculate the COD RAW
					float TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/TSS_MeasurementValues_t.PD2_New));
					//Publish the values to the MODBUS
					InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
					InputRegister_t.PV_info.TSS_PD2_0 = TSS_MeasurementValues_t.PD2_Zero;
					//Reset the command
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
					AWADataStoreState.factoryTSS_setzero = SET;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = SET;

				}else if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_FACTORY_SETASZERO
						&& AWAOperationStatus_t.CleaningTankEmpty != RESET)
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				}
				/*Emergency Pump Stop*/
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == STOP_RUNNING_PUMP)
					pumpOperationStop();
				else
				{
					HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
					CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
				}
				break;
			}
		}
	}
	else if(HoldingRegister_t.ModeCommand_t.ModeCommand_H == SOFTWARE_RESET)
	{
		if(HoldingRegister_t.ModeCommand_t.ModeCommand_L == SOFTWARE_RESET)
			HAL_NVIC_SystemReset();
	}
}

void CardAction(uint8_t CardID)
{
	float cableResistance = 0;
	float pH = 0;
	float ph_slope = 0;
	switch(CardID)
	{
	case MPH:
				AdCounts_pH = 0;
				//select the pH on the CARD
				//SEL2_pH_REDOX_SET();
				PH_SEL2_ON(CARD_SLOT_6);
				//Get ADC counts from the pH card
				AdCounts_pH = ADS1115_OperationpHMeasurement(0x01);
				//de-select the pH card
				//SEL2_pH_REDOX_RESET();
				//NO NEED TO TURN OFF THIS PIN
//				PH_SEL2_OFF(CARD_SLOT_6);
				//if the volatge is -ve
				if(AdCounts_pH > 0x7FFF)
					adc = (AdCounts_pH - 65535.0f) * pH_ElectronicCalibpoints_t.pH_Slope + pH_ElectronicCalibpoints_t.pH_Intercept;
				//if the volatge is +ve
				else
					adc = AdCounts_pH * pH_ElectronicCalibpoints_t.pH_Slope + pH_ElectronicCalibpoints_t.pH_Intercept;
				//if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == THREE_POINT)
				if(pH_SensorCalibpoints_t.measurement_Type == THREE_POINT)
				{
					float calculated_pH = (-16.908f * adc) + 7.0f;

					if(calculated_pH <= pH_SensorCalibpoints_t.middle_value)
					{
						//pH value with temperature compensation
						ph_slope = pH_SensorCalibpoints_t.pH_Solpe * -0.05916f/100.0f;
						ph_slope = 1 / ph_slope;
						//pH = (adc * pH_SensorCalibpoints_t.pH_Solpe * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept);
						pH = (adc * ph_slope * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept);
					}else
					{
						//pH value with temperature compensation
						ph_slope = pH_SensorCalibpoints_t.pH_slope_range_2 * -0.05916f/100.0f;
						ph_slope = 1 / ph_slope;
						//pH = (adc * pH_SensorCalibpoints_t.pH_Solpe * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept);
						pH = (adc * ph_slope * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept_range_2);
					}

				}
				//Two point of single point calibration
				else
				{
					//pH value with temperature compensation
					ph_slope = pH_SensorCalibpoints_t.pH_Solpe * -0.05916f/100.0f;
					ph_slope = 1 / ph_slope;
					//pH = (adc * pH_SensorCalibpoints_t.pH_Solpe * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept);
					pH = (adc * ph_slope * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept);
				}


				if(pH < 0)
					InputRegister_t.PV_info.pH_value = 0;
				else if(pH > 14)
					InputRegister_t.PV_info.pH_value = 14.0f;
				else
					InputRegister_t.PV_info.pH_value= pH;
				//For test - Temperature at 25degC
				//InputRegister_t.PV_info.pH_value = (adc * pH_SensorCalibpoints_t.pH_Solpe * (1.0 / (1.0 + 0.003351686 * (25 - 25))) + pH_SensorCalibpoints_t.pH_Intercept);

				InputRegister_t.PV_info.pH_mV = adc;
				InputRegister_t.SlotParameter.pH_ADC_Counts = AdCounts_pH;
				InputRegister_t.SlotParameter.pH_Live_Volatge = adc;

				//if temp. type selection is PT-100 or PT-1000 then read the temp. sensor
				if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType != 3)
					CardAction(0x02);
				//Manual temperature entry
				else
					InputRegister_t.PV_info.temp_pH = HoldingRegister_t.ModeCommand_t.Temperature_setPoint;
				break;
	case MVUX://Yet to be defined the card wise functions
				//CODADCCapture();
				#if 0
				TempDetNoise = 0;
				TempDetNoise = CardReadADC(CardID,CH0);

				TempDet1Noise_Ch1[ArrayPtr] = TempDetNoise;
				UInputReg.SInputReg.SADCReg[ArrayPtr].Det1Noise_Ch1 = TempDetNoise;

				//UInputReg.SInputReg.SADCReg[dataptr1].Det1Noise_Ch1 = ReadADC(CH0);

			  	TempDetNoise = 0;
				TempDetNoise = CardReadADC(CardID,CH1);


			    /* USER CODE BEGIN 3 */
				if(0x01)//UHoldReg.SHoldReg.ADCReadStart == 0x01)
				{
					if(TimerStart == 0x02)
					{
						TimerStart = 0x00;

						//change this
						//for(t=0; t<500; t++)
						//{
							//UInputReg.WordArr[t] = 0;
						//}

						CycleStart = 1;

						htim2.Instance->DIER |= TIM_IT_UPDATE;
						htim2.Instance->CR1 |= TIM_CR1_CEN;
						RxFlag = 0x00;
						huart3.Instance->CR1 &= ~UART_MODE_RX;// Rx disable
						huart3.Instance->CR1 &= ~UART_IT_RXNE; // UART RX not empty interrupt disable

						dataptr1 = 0; //SSG
						dataptr = 0;

						ArrayPtr = 0;
						//	ADCInit();
					  }

					if(TIM2State == TIME_6MS && ReadADCFlag == 1)
					{
						//HAL_Delay(1);

						TempDetNoise = 0;
						AD7682Init(PD1_CHANNEL);
						//dummy read
						TempDetNoise = SPI_ReadData();//ReadADC(CH0);
						//actual ADC data
						TempDetNoise = SPI_ReadData();

						TempDet1Noise_Ch1[ArrayPtr] = TempDetNoise;
						//UInputReg.SInputReg.SADCReg[ArrayPtr].Det1Noise_Ch1 = TempDetNoise;//no need to send this on modbus

					  	TempDetNoise = 0;
					  	AD7682Init(PD2_CHANNEL);
					  	//dummy read
						TempDetNoise = SPI_ReadData();//ReadADC(CH1);
						//actual ADC Data
						TempDetNoise = SPI_ReadData();

						TempDet2Noise_Ch2[ArrayPtr] = TempDetNoise;
						//UInputReg.SInputReg.SADCReg[ArrayPtr].Det2Noise_Ch2 = TempDetNoise;//no need to send this on modbus

						(dataptr1 < 99)?(dataptr1++):(dataptr1 = 500);
						ReadADCFlag = 0x00;
					}
					if(TIM2State == TIME_8MS && ReadADCFlag == 1)
					{
						TempDetSignal = 0;
						AD7682Init(PD1_CHANNEL);
						//dummy read
						TempDetSignal = SPI_ReadData();//ReadADC(CH0);
						//Actual ADC data
						TempDetSignal = SPI_ReadData();

						TempDet1Signal_Ch1[ArrayPtr] = TempDetSignal;
						//UInputReg.SInputReg.SADCReg[ArrayPtr].Det1Signal_Ch1 = TempDetSignal;//no need to send this on modbus

						TempDetSignal = 0;
						AD7682Init(PD2_CHANNEL);
						//Dummy read
						TempDetSignal = SPI_ReadData();//ReadADC(CH1);
						//Actual ADC data
						TempDetSignal = SPI_ReadData();

						TempDet2Signal_Ch2[ArrayPtr] = TempDetSignal;
						//UInputReg.SInputReg.SADCReg[ArrayPtr].Det2Signal_Ch2 = TempDetSignal;//no need to send this on modbus

						huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

						ArrayPtr++;

						(dataptr < 99)?(dataptr++):(dataptr = 500);
						ReadADCFlag = 0x00;
					}

					if(ArrayPtr >= 100) //if((dataptr1 == 500) && (dataptr == 500))
					{
						ArrayPtr = 0;

						CycleStart = 0;

						//Old code
#if 0
						//19/10/2021
						//averaging the PD1 and PD2 noise and signal data
						for(t=0; t<100; t++)//limit of this for loop will change according to the number of flashes selected
						{
							SignalNoiseDiff_Ch1[t] = TempDet1Signal_Ch1[t] - TempDet1Noise_Ch1[t];
							SignalNoiseDiff_Ch2[t] = TempDet2Signal_Ch2[t] - TempDet2Noise_Ch2[t];
						}

						for(t=0; t<100; t++)//limit of this for loop will change according to the number of flashes selected
						{
							TempMeanPd1 = TempMeanPd1 + SignalNoiseDiff_Ch1[t];
							TempMeanPd2 = TempMeanPd2 + SignalNoiseDiff_Ch2[t];
						}
						TempMeanPd1 = TempMeanPd1/100;
						TempMeanPd2 = TempMeanPd2/100;
#endif

						//Actual code
#if 0
						for(int i = 0;i<100;i++)
						{
							TempMeanPd1 +=(TempDet1Signal_Ch1[i] - TempDet1Noise_Ch1[i])/100;
							TempMeanPd2 +=(TempDet2Signal_Ch2[i] - TempDet2Noise_Ch2[i])/100;
						}
#endif

						//Test Code
#if 1
						for(int i = 0;i<100;i++)
						{
							TempMeanPd1 +=(TempDet1Signal_Ch1[i])/100;
							TempMeanPd2 +=(TempDet2Signal_Ch2[i])/100;
						}
#endif

						//UInputReg.SInputReg.MeanPD1 = (float)(TempMeanPd1/100);
						float volatge_PD1 = (2.5/65535)*(TempMeanPd1);
						InputRegister_t.SlotParameter.PD1_mean = TempMeanPd1;
						InputRegister_t.SlotParameter.PD1_Voltage = volatge_PD1;
						//UInputReg.SInputReg.MeanPD2 = (float)(TempMeanPd2/100);
						float volatge_PD2 = (2.5/65535)*(TempMeanPd2);
						InputRegister_t.SlotParameter.PD2_mean = TempMeanPd2;
						InputRegister_t.SlotParameter.PD2_Voltage = volatge_PD2;
						TempMeanPd1 = 0;
						TempMeanPd2 = 0;

						htim2.Instance->DIER &= ~TIM_IT_UPDATE;
						htim2.Instance->CR1 &= ~TIM_CR1_CEN;

						UHoldReg.SHoldReg.ADCReadStart = 0x00;//change the command register
						TimerStart = 0x02;
						dataptr1 = 0;
						dataptr = 0;

						huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
						huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
					}

				  }
				  //HAL_UART_IRQHandler(&huart3);
#endif
			  /* USER CODE END 3 */
		break;

	case MIL:
				AdCounts_MIL = 0;
				AdCounts_MIL = CardReadADC(CardID,CH0);
		break;

	case MI420:
		AdCounts_pH = 0;
#if 1
		//Software PT100 select SEL1 pin HIGH
		if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)//PT100
			SEL1_PT100();
		//Software PT1000 select SEL1 pin LOW
		else
			SEL1_PT1000();
#endif
		//Initialize the ADC for temperature configuration
		ADS1115_OperationInit_Temperature();

		//Read the sample twice, discard the first reading
		for(int j = 0;j<2;j++)
		{
			ADS1115_OperationStartConversion();
			HAL_Delay(2);
			uint16_t ADC_count = ADS1115_OperationADCRead();
			raw_adc.raw_data[0] = (ADC_count >> 8);
			raw_adc.raw_data[1] = ADC_count;
			AdCounts_pH = raw_adc.data;
		}

		//If PT-100 is selected
		if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)
		{
			adc = AdCounts_pH * PT_ElectronicCalibration_t.PT100_slope + PT_ElectronicCalibration_t.PT100_intercept - HoldingRegister_t.IOUTCalibandTest_t.temperatureSensorCable_resistance;
			volatge = adc;
			adc *= 10.0f;
			//cableResistance = 10.0f * HoldingRegister_t.IOUTCalibandTest_t.temperatureSensorCable_resistance;

		}
		//If PT-1000 is selected
		else if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT1000)
		{
			adc = AdCounts_pH * PT_ElectronicCalibration_t.PT1000_slope + PT_ElectronicCalibration_t.PT1000_intercept;
			volatge = adc;
			//cableResistance = HoldingRegister_t.IOUTCalibandTest_t.temperatureSensorCable_resistance;
		}

		//Subtract the temperature sensor cable resistance from the calculated resistance
		//adc -= cableResistance;

		//ADC counts to temp. calculation
		float temperature = -246.54809247 + adc * (0.23753052489 + adc * (8.8646024011e-6 + 1.5342296758e-13 * adc * adc));

		//Publish the data to modbus
		InputRegister_t.SlotParameter.pH_Temperature_ADC_Counts = AdCounts_pH;
		InputRegister_t.SlotParameter.pH_Temperature_Live_Voltage = volatge;
		InputRegister_t.PV_info.temp_pH = temperature;
		break;
	case COD:
	{
		if(HoldingRegister_t.ModeCommand_t.CommonCommand == AUTO_COD_ZERO)
		{

		}
		break;
	}
	case MO420:
		break;
	//Depricated code
	case 0xaa:
		//readpHTemp();
		break;
	default:
		break;


	}
}

void filtering(uint16_t no_of_flashes)
{
	uint16_t flash_limit = no_of_flashes;


	for(int t=0; t<flash_limit; t++)//limit of this for loop will change according to the number of flashes selected
	{
		if(TempDet1Signal_Ch1[t] >= TempDet1Noise_Ch1[t])
			SignalNoiseDiff_Ch1[t] = TempDet1Signal_Ch1[t] - TempDet1Noise_Ch1[t];
		else
			SignalNoiseDiff_Ch1[t] = - TempDet1Signal_Ch1[t] + TempDet1Noise_Ch1[t];
		if(TempDet2Signal_Ch2[t] >= TempDet2Noise_Ch2[t])
			SignalNoiseDiff_Ch2[t] = TempDet2Signal_Ch2[t] - TempDet2Noise_Ch2[t];
		else
			SignalNoiseDiff_Ch2[t] =  - TempDet2Signal_Ch2[t] + TempDet2Noise_Ch2[t];
	}

	for(int t=0; t<flash_limit; t++)//limit of this for loop will change according to the number of flashes selected
	{
		TempMeanPd1 = TempMeanPd1 + SignalNoiseDiff_Ch1[t];
		TempMeanPd2 = TempMeanPd2 + SignalNoiseDiff_Ch2[t];
	}
	TempMeanPd1 = TempMeanPd1/flash_limit;
	TempMeanPd2 = TempMeanPd2/flash_limit;

	//Ascending order for PD1 and PD2
	CheckScreen_bubble_sort(flash_limit);

}

void CheckScreen_bubble_sort(uint16_t filterLimit)
{
	uint16_t temp = 0;

	for(int i = 0;i<filterLimit;i++)
	{
		filter_data_PD1[i] = SignalNoiseDiff_Ch1[i];
		filter_data_PD2[i] = SignalNoiseDiff_Ch2[i];
	}

	//filtering of PD1
	for(int i = 0; i< filterLimit - 1 ; i++)
	{
		for(int j = i+1 ; j<filterLimit; j++)
		{
			if(filter_data_PD1[i] >= filter_data_PD1[j])
			{
				temp = filter_data_PD1[i];
				filter_data_PD1[i] = filter_data_PD1[j];
				filter_data_PD1[j] = temp;
			}
		}
	}

	//filtering of PD2
	for(int i = 0; i< filterLimit - 1 ; i++)
	{
		for(int j = i+1 ; j<filterLimit; j++)
		{
			if(filter_data_PD2[i] >= filter_data_PD2[j])
			{
				temp = filter_data_PD2[i];
				filter_data_PD2[i] = filter_data_PD2[j];
				filter_data_PD2[j] = temp;
			}
		}
	}
	int count = 0;

	//Limits for filtering
	uint16_t limit_min = filterLimit * 0.2f; 		/*20%  of the total limit*/
	uint16_t limit_max = filterLimit - limit_min;
	uint16_t data_range = limit_max - limit_min;

	flash_limit_min = limit_min;
	flash_limit_max = limit_max;


	//removed first 10 and last 10 data
	for(int i=limit_min;i<limit_max;i++)
	{
		filter_data_PD1_mean += filter_data_PD1[i];
		filter_data_PD2_mean += filter_data_PD2[i];
		count += 1;
	}

	//Averaged 80 points data
	filter_data_PD1_mean /= data_range;
	filter_data_PD2_mean /= data_range;

	//Get the minimum and maximum
	HoldingRegister_t.ModeCommand_t.CS_PD_1_MIN = filter_data_PD1[limit_min];
	HoldingRegister_t.ModeCommand_t.CS_PD_1_MAX = filter_data_PD1[limit_max - 1];
	HoldingRegister_t.ModeCommand_t.CS_PD_2_MIN = filter_data_PD2[limit_min];
	HoldingRegister_t.ModeCommand_t.CS_PD_2_MAX = filter_data_PD2[limit_max - 1];

}
uint8_t CheckScreen_CODADCCapture(uint8_t command)
{
	//variables for calculation
	float COD_RAW = 0;
	float TSS_RAW = 0;

	//this flag will be set when the PD1 and PD2 zero values have been measured and stored
	uint8_t flash = 0;

	uint8_t no_of_flashes = 0;

	//if the command is for cod measurernt and zeroing of the PD1 and PD2 is done will only then it will measure and perform flashing
	if((command == COD_Measure || command == AUTO_COD_MEASURE) && COD_MeasurementStatus.COD_ZERO == 0x01)
	{
		//set the flag to 1 as auto zeroing is done
		flash = 1;
	}else if(command == AUTO_COD_ZERO){
		//Enter the Flashing operation to perform the Auto Zeroing
		flash = 1;
	}else{
		//reset the common command flag as Auto Zeroing is not yet done
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		return 0;
	}
	if(flash)
	{
		//Set the COD flashing operation to prevent the program to enter into other card action
		AWAOperationStatus_t.CODFlashOperation = 0x01;
		if(TimerStart == 0x02)
		{
			TimerStart = 0x00;
			CycleStart = 1;

			htim2.Instance->DIER |= TIM_IT_UPDATE;
			htim2.Instance->CR1 |= TIM_CR1_CEN;

			dataptr1 = 0; //SSG
			dataptr = 0;

			ArrayPtr = 0;
			/*TDOD: Check for the number of flashes to be performed*/
			no_of_flashes = HoldingRegister_t.ModeCommand_t.CS_FLASH;
			if(no_of_flashes == 1)
				noOfFlashes = 100;
			else if(no_of_flashes == 2)
				noOfFlashes = 500;

			//	ADCInit();
			//Set the COD measure flag to "1", tp indicate the start of flashing sequence
			cod_flash_operation = 1;
			//Set HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_MEASURE,SET);

		  }

		if(TIM2State == TIME_6MS && ReadADCFlag == 1)
		{
			//HAL_Delay(1);

			TempDetNoise = 0;
			AD7682Init(PD1_CHANNEL);
			//dummy read
			//TempDetNoise = SPI_ReadData();//ReadADC(CH0);
			//actual ADC data
			TempDetNoise = SPI_ReadData();

			TempDet1Noise_Ch1[ArrayPtr] = TempDetNoise;
			//UInputReg.SInputReg.SADCReg[ArrayPtr].Det1Noise_Ch1 = TempDetNoise;//no need to send this on modbus

		  	TempDetNoise = 0;
		  	AD7682Init(PD2_CHANNEL);
		  	//dummy read
			//TempDetNoise = SPI_ReadData();//ReadADC(CH1);
			//actual ADC Data
			TempDetNoise = SPI_ReadData();

			TempDet2Noise_Ch2[ArrayPtr] = TempDetNoise;
			//UInputReg.SInputReg.SADCReg[ArrayPtr].Det2Noise_Ch2 = TempDetNoise;//no need to send this on modbus

			(dataptr1 < 99)?(dataptr1++):(dataptr1 = 500);
			ReadADCFlag = 0x00;
		}
		if(TIM2State == TIME_8MS && ReadADCFlag == 1)
		{
			TempDetSignal = 0;
			//configure the ADC for PD1 ADC channel
			AD7682Init(PD1_CHANNEL);
			//dummy read
			//TempDetSignal = SPI_ReadData();//ReadADC(CH0);
			//Actual ADC data
			TempDetSignal = SPI_ReadData();

			//store the Signal ADC reading into the array
			TempDet1Signal_Ch1[ArrayPtr] = TempDetSignal;
			//reset the buffer
			TempDetSignal = 0;
			//configure the ADC for PD2 ADC channel
			AD7682Init(PD2_CHANNEL);
			//Dummy read
			//TempDetSignal = SPI_ReadData();//ReadADC(CH1);
			//Actual ADC data
			TempDetSignal = SPI_ReadData();

			//store the Signal ADC reading into the array
			TempDet2Signal_Ch2[ArrayPtr] = TempDetSignal;

			//huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			ArrayPtr++;

			(dataptr < 99)?(dataptr++):(dataptr = 500);
			ReadADCFlag = 0x00;
		}

		if(ArrayPtr >= noOfFlashes) //if((dataptr1 == 500) && (dataptr == 500))
		{
			ArrayPtr = 0;

			CycleStart = 0;

			filtering(noOfFlashes);

			//new mean
			TempMeanPd1 = filter_data_PD1_mean;
			TempMeanPd2 = filter_data_PD2_mean;

			filter_data_PD1_mean = 0;
			filter_data_PD2_mean = 0;
			//Perform this condition dusring Auto zero
			if(command == AUTO_COD_ZERO)
			{
				//store the pD1 and PD2 mean vales to the global PD1 zero and PD2 zero
				PD1_Zero = TempMeanPd1;
				PD2_Zero = TempMeanPd2;

				//publish the PD1 and PD2 zero values to the modbus
				InputRegister_t.PV_info.PD1_0 = PD1_Zero;
				InputRegister_t.PV_info.PD2_0 = PD2_Zero;
				InputRegister_t.PV_info.TSS_PD2_0 = PD2_Zero;

				COD_MeasurementValues_t.PD1_Zero = PD1_Zero;
				COD_MeasurementValues_t.PD2_Zero = PD2_Zero;

				//TSS PD2_0 value
				TSS_MeasurementValues_t.PD2_Zero = PD2_Zero;

				//Flag set when the PD1 and PD2 zero values are measured and stored
				COD_MeasurementStatus.COD_ZERO = 0x01;
			}
			//Perform this condition during the measure or auto measure command
			else if(command == AUTO_COD_MEASURE || command == COD_Measure)
			{
				//store the pD1 and PD2 mean vales to the global PD1 zero and PD2 new
				PD1_new = TempMeanPd1;
				PD2_new = TempMeanPd2;

				COD_MeasurementValues_t.PD1_New = PD1_new;
				COD_MeasurementValues_t.PD2_New = PD2_new;

				//TSS
				TSS_MeasurementValues_t.PD2_New = PD2_new;

				//publish the PD1 and PD2 new values to the modbus
				InputRegister_t.PV_info.PD1_MEAN = PD1_new;
				InputRegister_t.PV_info.PD2_MEAN = PD2_new;

				//Get the PD1(0) and PD2(0) from the modbus map to the local variable
				COD_MeasurementValues_t.PD1_Zero = InputRegister_t.PV_info.PD1_0;
				COD_MeasurementValues_t.PD2_Zero = InputRegister_t.PV_info.PD2_0;

				//Get the PD2(0) from the modbus map to the local variable / this value can be changed in factory 10 pt cal
				TSS_MeasurementValues_t.PD2_Zero = InputRegister_t.PV_info.TSS_PD2_0;//Make a default value in the main

				//Calculations for COD raw
				COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/PD1_new) - log(COD_MeasurementValues_t.PD2_Zero/PD2_new));

				//Calculation of TSS raw
				TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/PD2_new));

				//Quadratic equation - compensation (Factory Calibration)
				float c[4];//BOD
				float k[3];//TSS

				//COD coefficient
				for(int i = 0;i<4;i++)
					c[i] = HoldingRegister_t.ModeCommand_t.C[i];

				//TSS coefficient
				for(int i = 0;i<3;i++)
					k[i] = HoldingRegister_t.ModeCommand_t.TSS_K[i];

				//COD_RAW = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];
				float factory_cod_value = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];

				//TSS factory equation
				float factory_tss_value = pow(TSS_RAW,2) *k[2] + pow(TSS_RAW,1) *k[1] + k[0];

				//COD actual value
				COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;

				//TSS value
				TSS_MeasurementValues_t.Cal_Value = factory_tss_value * TSS_SensorCalibration_t.slope + TSS_SensorCalibration_t.intercept;

				//BOD value
				InputRegister_t.PV_info.BODValue = 0.5f * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;

				//TOC Value
				InputRegister_t.PV_info.TOC = 0.3f * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;

				//Publish the COD_RAW on modbus
				//InputRegister_t.PV_info.COD_RAW = COD_MeasurementValues_t.RAW_Value;
				InputRegister_t.PV_info.COD_RAW = COD_RAW;
				if(COD_MeasurementValues_t.Cal_Value < 0)
				{
					InputRegister_t.PV_info.CODValue = 0.0f;	/*<Display to the Customer,only positive value*/
					//BOD value
					InputRegister_t.PV_info.BODValue = 0;		/*<Display to the Customer,only positive value*/
				}
				else if(COD_MeasurementValues_t.Cal_Value > COD_UpperLimit)
				{
					InputRegister_t.PV_info.CODValue = COD_UpperLimit;
					InputRegister_t.PV_info.BODValue = BOD_UpperLimit;
				}
				else
				{
					InputRegister_t.PV_info.CODValue = COD_MeasurementValues_t.Cal_Value;
					//BOD value
					InputRegister_t.PV_info.BODValue = HoldingRegister_t.ModeCommand_t.BOD_CF * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;
				}
				InputRegister_t.PV_info.CODValueUser = COD_MeasurementValues_t.Cal_Value; 	/*<Reference for the Developer, can be negative value*/
				InputRegister_t.PV_info.BODValueUser = HoldingRegister_t.ModeCommand_t.BOD_CF * COD_MeasurementValues_t.Cal_Value;

				//Publish the TSS value to the modbus
				InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
				if(TSS_MeasurementValues_t.Cal_Value < 0)
					InputRegister_t.PV_info.TSSValue = 0.0f;	/*<Display to the Customer,only positive value*/
				else if(TSS_MeasurementValues_t.Cal_Value > TSS_UpperLimit)
				{
					InputRegister_t.PV_info.TSSValue = TSS_UpperLimit;
				}
				else
					InputRegister_t.PV_info.TSSValue = TSS_MeasurementValues_t.Cal_Value;
				InputRegister_t.PV_info.TSSValueUser = TSS_MeasurementValues_t.Cal_Value; 	/*<Reference for the Developer, can be negative value*/

				//Set the FRAM data storage flag
				AWAOperationStatus_t.AWADataSave_ProcessValues = SET;

				AWADataStoreState.analyzerPocessvalue = SET;

			}

			//RSD
			double sum = 0;
			double sum2 = 0;
			double var = 0;
			double rsd = 0;
			double rsd2 = 0;

			PD1_new = TempMeanPd1;
			PD2_new = TempMeanPd2;

			//(new_value - mean)^2
			int b = 0;

			for(int i = flash_limit_min,b = 0;i<flash_limit_max;i++,b++)
			{
				sd[b] = pow((filter_data_PD1[i] - PD1_new),2)/(flash_limit_max - flash_limit_min);
				sd2[b] = pow((filter_data_PD2[i] - PD2_new),2)/(flash_limit_max - flash_limit_min);
			}

			for(int i = 0;i<(flash_limit_max - flash_limit_min);i++)
			{
				sum += sd[i];
				sum2 += sd2[i];
			}
			//SD
			sum = sqrt(sum);
			sum2 = sqrt(sum2);

			//variance
			var = pow(sum,2);

			//RSD
			rsd = (sum/PD1_new) * 100.0f;
			rsd2 = (sum2/PD2_new) * 100.0f;


			COD_MeasurementValues_t.RSD1 = rsd;
			COD_MeasurementValues_t.RSD2 = rsd2;

			InputRegister_t.PV_info.RSD1 = rsd;
			InputRegister_t.PV_info.RSD2 = rsd2;
			//printf("RSD 1: %f, RSD 2: %f",rsd,rsd2);
			//rest the PD1 and PD2 mean buffers
			TempMeanPd1 = 0;
			TempMeanPd2 = 0;

			//Turn off the timer 2
			htim2.Instance->DIER &= ~TIM_IT_UPDATE;
			htim2.Instance->CR1 &= ~TIM_CR1_CEN;

			UHoldReg.SHoldReg.ADCReadStart = 0x00;//change the command register
			TimerStart = 0x02;
			dataptr1 = 0;
			dataptr = 0;

			//Turn of the modbus receive query
			//huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
			//huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			//reset the common command flag
			HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;

			HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

			//set the flashing operation flag to 2 as flashing operation is completed successfully
			cod_flash_operation = 2;

			//Reset the COD flashing operation to perform other card action
			AWAOperationStatus_t.CODFlashOperation = 0x00;

			//Reset the HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_MEASURE,RESET);
			//HMI bits for MIMICS
			CoilStatusRegister_t.CoilStatus_t.measure = RESET;
			CoilStatusRegister_t.CoilStatus_t.read_acid = RESET;
			CoilStatusRegister_t.CoilStatus_t.read_sample = RESET;
			CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
		}

	  }
}
uint8_t CODADCCapture(uint8_t command)
{
	//variables for calculation
	float COD_RAW = 0;
	float TSS_RAW = 0;

	//this flag will be set when the PD1 and PD2 zero values have been measured and stored
	uint8_t flash = 0;

	//if the command is for cod measurernt and zeroing of the PD1 and PD2 is done will only then it will measure and perform flashing
	if((command == COD_Measure || command == AUTO_COD_MEASURE) && COD_MeasurementStatus.COD_ZERO == 0x01)
	{
		//set the flag to 1 as auto zeroing is done
		flash = 1;
	}else if(command == AUTO_COD_ZERO){
		//Enter the Flashing operation to perform the Auto Zeroing
		flash = 1;
	}else{
		//reset the common command flag as Auto Zeroing is not yet done
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		return 0;
	}
	if(flash)
	{
		//Set the COD flashing operation to prevent the program to enter into other card action
		AWAOperationStatus_t.CODFlashOperation = 0x01;
		if(TimerStart == 0x02)
		{
			TimerStart = 0x00;
			CycleStart = 1;

			htim2.Instance->DIER |= TIM_IT_UPDATE;
			htim2.Instance->CR1 |= TIM_CR1_CEN;
#if 0 //if 0, it will accept the MODBUS query even during the COD measurement process
			RxFlag = 0x00;
			//turn of the MODBUS receive query
			huart3.Instance->CR1 &= ~UART_MODE_RX;// Rx disable
			huart3.Instance->CR1 &= ~UART_IT_RXNE; // UART RX not empty interrupt disable
#endif
			dataptr1 = 0; //SSG
			dataptr = 0;

			ArrayPtr = 0;
			//	ADCInit();
			//Set the COD measure flag to "1", tp indicate the start of flashing sequence
			cod_flash_operation = 1;
			//Set HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_MEASURE,SET);

			if(HoldingRegister_t.ModeCommand_t.ModeCommand_H != FACTORY_MODE)
				CoilStatusRegister_t.CoilStatus_t.measure = SET;

			//Reset the trend capture trigger
			//CoilStatusRegister_t.CoilStatus_t.TRED_TRIGGER = RESET;

		  }

		if(TIM2State == TIME_6MS && ReadADCFlag == 1)
		{
			//HAL_Delay(1);

			TempDetNoise = 0;
			AD7682Init(PD1_CHANNEL);
			//dummy read
			//TempDetNoise = SPI_ReadData();//ReadADC(CH0);
			//actual ADC data
			TempDetNoise = SPI_ReadData();

			TempDet1Noise_Ch1[ArrayPtr] = TempDetNoise;
			//UInputReg.SInputReg.SADCReg[ArrayPtr].Det1Noise_Ch1 = TempDetNoise;//no need to send this on modbus

		  	TempDetNoise = 0;
		  	AD7682Init(PD2_CHANNEL);
		  	//dummy read
			//TempDetNoise = SPI_ReadData();//ReadADC(CH1);
			//actual ADC Data
			TempDetNoise = SPI_ReadData();

			TempDet2Noise_Ch2[ArrayPtr] = TempDetNoise;
			//UInputReg.SInputReg.SADCReg[ArrayPtr].Det2Noise_Ch2 = TempDetNoise;//no need to send this on modbus

			(dataptr1 < 99)?(dataptr1++):(dataptr1 = 500);
			ReadADCFlag = 0x00;
		}
		if(TIM2State == TIME_8MS && ReadADCFlag == 1)
		{
			TempDetSignal = 0;
			//configure the ADC for PD1 ADC channel
			AD7682Init(PD1_CHANNEL);
			//dummy read
			//TempDetSignal = SPI_ReadData();//ReadADC(CH0);
			//Actual ADC data
			TempDetSignal = SPI_ReadData();

			//store the Signal ADC reading into the array
			TempDet1Signal_Ch1[ArrayPtr] = TempDetSignal;
			//reset the buffer
			TempDetSignal = 0;
			//configure the ADC for PD2 ADC channel
			AD7682Init(PD2_CHANNEL);
			//Dummy read
			//TempDetSignal = SPI_ReadData();//ReadADC(CH1);
			//Actual ADC data
			TempDetSignal = SPI_ReadData();

			//store the Signal ADC reading into the array
			TempDet2Signal_Ch2[ArrayPtr] = TempDetSignal;

			//huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			ArrayPtr++;

			(dataptr < 99)?(dataptr++):(dataptr = 500);
			ReadADCFlag = 0x00;
		}

		if(ArrayPtr >= 100) //if((dataptr1 == 500) && (dataptr == 500))
		{
			ArrayPtr = 0;

			CycleStart = 0;


			//19/10/2021
			//averaging the PD1 and PD2 noise and signal data
			for(int t=0; t<100; t++)//limit of this for loop will change according to the number of flashes selected
			{
				if(TempDet1Signal_Ch1[t] >= TempDet1Noise_Ch1[t])
					SignalNoiseDiff_Ch1[t] = TempDet1Signal_Ch1[t] - TempDet1Noise_Ch1[t];
				else
					SignalNoiseDiff_Ch1[t] = - TempDet1Signal_Ch1[t] + TempDet1Noise_Ch1[t];
				if(TempDet2Signal_Ch2[t] >= TempDet2Noise_Ch2[t])
					SignalNoiseDiff_Ch2[t] = TempDet2Signal_Ch2[t] - TempDet2Noise_Ch2[t];
				else
					SignalNoiseDiff_Ch2[t] =  - TempDet2Signal_Ch2[t] + TempDet2Noise_Ch2[t];
			}

			for(int t=0; t<100; t++)//limit of this for loop will change according to the number of flashes selected
			{
				TempMeanPd1 = TempMeanPd1 + SignalNoiseDiff_Ch1[t];
				TempMeanPd2 = TempMeanPd2 + SignalNoiseDiff_Ch2[t];
			}
			TempMeanPd1 = TempMeanPd1/100;
			TempMeanPd2 = TempMeanPd2/100;

			//Ascending order for PD1 and PD2
			bubble_sort();

			//new mean
			TempMeanPd1 = filter_data_PD1_mean;
			TempMeanPd2 = filter_data_PD2_mean;

			filter_data_PD1_mean = 0;
			filter_data_PD2_mean = 0;
			//Perform this condition dusring Auto zero
			if(command == AUTO_COD_ZERO && HoldingRegister_t.ModeCommand_t.AUTO_ZERO == SET)
			{
				//store the pD1 and PD2 mean vales to the global PD1 zero and PD2 zero
				PD1_Zero = TempMeanPd1;
				PD2_Zero = TempMeanPd2;

				//publish the PD1 and PD2 zero values to the modbus
				InputRegister_t.PV_info.PD1_0 = PD1_Zero;
				InputRegister_t.PV_info.PD2_0 = PD2_Zero;
				InputRegister_t.PV_info.TSS_PD2_0 = PD2_Zero;

				COD_MeasurementValues_t.PD1_Zero = PD1_Zero;
				COD_MeasurementValues_t.PD2_Zero = PD2_Zero;

				//TSS PD2_0 value
				TSS_MeasurementValues_t.PD2_Zero = PD2_Zero;

				//Flag set when the PD1 and PD2 zero values are measured and stored
				COD_MeasurementStatus.COD_ZERO = 0x01;

				/*-----------------------------*/
				//TSS_MeasurementValues_t.PD2_Zero = TSS_MeasurementValues_t.PD2_New;

				//Calculate the COD RAW
				float TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/TSS_MeasurementValues_t.PD2_New));
				//Publish the values to the MODBUS
				InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
				//InputRegister_t.PV_info.TSS_PD2_0 = TSS_MeasurementValues_t.PD2_Zero;
				//Reset the command
				//HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;
				AWADataStoreState.factoryTSS_setzero = SET;
				//Flag set as data not saved in the FRAM
				//AWAOperationStatus_t.AWADataSave_Calibration = SET;
				/*******************************/

				/*--------------------------------*/
				//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
				//COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
				//COD_MeasurementValues_t.PD2_Zero = COD_MeasurementValues_t.PD2_New;

				//Calculate the COD RAW
				float COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/COD_MeasurementValues_t.PD1_New) - log(COD_MeasurementValues_t.PD2_Zero/COD_MeasurementValues_t.PD2_New));
				//Publish the values to the MODBUS
				InputRegister_t.PV_info.COD_RAW = COD_RAW;
				//Reset the command
				HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				AWADataStoreState.factoryCOD_setzero = SET;
				//Flag set as data not saved in the FRAM
				AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				/**********************************/
			}
			//Perform this condition during the measure or auto measure command
			else if(command == AUTO_COD_MEASURE || command == COD_Measure)
			{
				//store the pD1 and PD2 mean vales to the global PD1 zero and PD2 new
				PD1_new = TempMeanPd1;
				PD2_new = TempMeanPd2;

				COD_MeasurementValues_t.PD1_New = PD1_new;
				COD_MeasurementValues_t.PD2_New = PD2_new;

				//TSS
				TSS_MeasurementValues_t.PD2_New = PD2_new;

				//publish the PD1 and PD2 new values to the modbus
				InputRegister_t.PV_info.PD1_MEAN = PD1_new;
				InputRegister_t.PV_info.PD2_MEAN = PD2_new;

				//Get the PD1(0) and PD2(0) from the modbus map to the local variable
				COD_MeasurementValues_t.PD1_Zero = InputRegister_t.PV_info.PD1_0;
				COD_MeasurementValues_t.PD2_Zero = InputRegister_t.PV_info.PD2_0;

				//Get the PD2(0) from the modbus map to the local variable / this value can be changed in factory 10 pt cal
				TSS_MeasurementValues_t.PD2_Zero = InputRegister_t.PV_info.TSS_PD2_0;//Make a default value in the main

				//Calculations for COD raw
				COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/PD1_new) - log(COD_MeasurementValues_t.PD2_Zero/PD2_new));

				//Calculation of TSS raw
				TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/PD2_new));

				//Quadratic equation - compensation (Factory Calibration)
				float c[4];//BOD
				float k[3];//TSS

				//COD coefficient
				for(int i = 0;i<4;i++)
					c[i] = HoldingRegister_t.ModeCommand_t.C[i];

				//TSS coefficient
				for(int i = 0;i<3;i++)
					k[i] = HoldingRegister_t.ModeCommand_t.TSS_K[i];

				//COD_RAW = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];
				float factory_cod_value = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];

				//TSS factory equation
				float factory_tss_value = pow(TSS_RAW,2) *k[2] + pow(TSS_RAW,1) *k[1] + k[0];

				//Get the sensor calibration factor from modus
				COD_SensorCalibration_t.slope = HoldingRegister_t.SensorCalibration_t.COD_CF;
				COD_SensorCalibration_t.intercept = HoldingRegister_t.SensorCalibration_t.COD_Intercept;
//				COD_SensorCalibration_t.slope_range_2 = HoldingRegister_t.SensorCalibration_t.COD_CF_RANGE_2;
//				COD_SensorCalibration_t.intercept_range_2 = HoldingRegister_t.SensorCalibration_t.COD_Intercept_RANGE_2;


				//COD actual value
//				if(COD_SensorCalibration_t.measurementType == 0x04)
//				{
//#if 1
//					if(factory_cod_value <= COD_SensorCalibration_t.middle_value)
//					{
//						COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;
//					}else{
//						COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope_range_2 + COD_SensorCalibration_t.intercept_range_2;
//					}
//#else
//				COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;
//#endif
//				}else
//				{
//					COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;
//				}
				COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;
				//COD value with Factory Calibration not sensor
				//COD_MeasurementValues_t.Cal_Value = factory_cod_value;

				/*<DEPRICATED CODE*/
				//TSS value
				//TSS_MeasurementValues_t.Cal_Value = factory_tss_value;

				//TSS value
				TSS_MeasurementValues_t.Cal_Value = factory_tss_value * TSS_SensorCalibration_t.slope + TSS_SensorCalibration_t.intercept;

				//BOD value
				InputRegister_t.PV_info.BODValue = 0.5f * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;

				//TOC Value
				InputRegister_t.PV_info.TOC = 0.3f * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;

				//Publish the COD_RAW on modbus
				//InputRegister_t.PV_info.COD_RAW = COD_MeasurementValues_t.RAW_Value;
				InputRegister_t.PV_info.COD_RAW = COD_RAW;
				if(COD_MeasurementValues_t.Cal_Value < 0)
				{
					InputRegister_t.PV_info.CODValue = 0.0f;	/*<Display to the Customer,only positive value*/
					//BOD value
					InputRegister_t.PV_info.BODValue = 0;		/*<Display to the Customer,only positive value*/
				}
				else if(COD_MeasurementValues_t.Cal_Value > COD_UpperLimit)
				{
					InputRegister_t.PV_info.CODValue = COD_UpperLimit;
					InputRegister_t.PV_info.BODValue = BOD_UpperLimit;
				}
				else
				{
					InputRegister_t.PV_info.CODValue = COD_MeasurementValues_t.Cal_Value;
					//BOD value
					InputRegister_t.PV_info.BODValue = HoldingRegister_t.ModeCommand_t.BOD_CF * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;
				}
				InputRegister_t.PV_info.CODValueUser = COD_MeasurementValues_t.Cal_Value; 	/*<Reference for the Developer, can be negative value*/
				InputRegister_t.PV_info.BODValueUser = HoldingRegister_t.ModeCommand_t.BOD_CF * COD_MeasurementValues_t.Cal_Value;

				//Publish the TSS value to the modbus
				InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
				if(TSS_MeasurementValues_t.Cal_Value < 0)
					InputRegister_t.PV_info.TSSValue = 0.0f;	/*<Display to the Customer,only positive value*/
				else if(TSS_MeasurementValues_t.Cal_Value > TSS_UpperLimit)
				{
					InputRegister_t.PV_info.TSSValue = TSS_UpperLimit;
				}
				else
					InputRegister_t.PV_info.TSSValue = TSS_MeasurementValues_t.Cal_Value;
				InputRegister_t.PV_info.TSSValueUser = TSS_MeasurementValues_t.Cal_Value; 	/*<Reference for the Developer, can be negative value*/

				//Set the FRAM data storage flag
				AWAOperationStatus_t.AWADataSave_ProcessValues = SET;

				AWADataStoreState.analyzerPocessvalue = SET;

				//Set the trend capture trigger
				//CoilStatusRegister_t.CoilStatus_t.TRED_TRIGGER = SET;

			}

			//RSD
			double sum = 0;
			double sum2 = 0;
			double var = 0;
			double rsd = 0;
			double rsd2 = 0;

			PD1_new = TempMeanPd1;
			PD2_new = TempMeanPd2;

			//(new_value - mean)^2
			int b = 0;
			//for(int i=  0;i<100;i++)
			//for(int i = 20,b = 0;i<80;i++,b++)
			for(int i = COD_OUT_FILTER_MIN,b = 0;i<COD_OUT_FILTER_MAX;i++,b++)
			{
				//sd[i] = pow((SignalNoiseDiff_Ch1[i] - PD1_new),2)/100;
				//sd2[i] = pow((SignalNoiseDiff_Ch2[i] - PD2_new),2)/100;

				sd[b] = pow((filter_data_PD1[i] - PD1_new),2)/COD_OUT_FILTER_RANGE;
				sd2[b] = pow((filter_data_PD2[i] - PD2_new),2)/COD_OUT_FILTER_RANGE;
			}

			//for(int i = 0;i<100;i++)
			//for(int i = 0;i<60;i++)
			for(int i = 0;i<COD_OUT_FILTER_RANGE;i++)
			{
				sum += sd[i];
				sum2 += sd2[i];
			}
			//SD
			sum = sqrt(sum);
			sum2 = sqrt(sum2);

			//variance
			var = pow(sum,2);

			//RSD
			rsd = (sum/PD1_new) * 100.0f;
			rsd2 = (sum2/PD2_new) * 100.0f;


			COD_MeasurementValues_t.RSD1 = rsd;
			COD_MeasurementValues_t.RSD2 = rsd2;

			InputRegister_t.PV_info.RSD1 = rsd;
			InputRegister_t.PV_info.RSD2 = rsd2;
			//printf("RSD 1: %f, RSD 2: %f",rsd,rsd2);
			//rest the PD1 and PD2 mean buffers
			TempMeanPd1 = 0;
			TempMeanPd2 = 0;

			//Turn off the timer 2
			htim2.Instance->DIER &= ~TIM_IT_UPDATE;
			htim2.Instance->CR1 &= ~TIM_CR1_CEN;

			UHoldReg.SHoldReg.ADCReadStart = 0x00;//change the command register
			TimerStart = 0x02;
			dataptr1 = 0;
			dataptr = 0;

			//Turn of the modbus receive query
			//huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
			//huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			//reset the common command flag
			HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;

			HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

			//set the flashing operation flag to 2 as flashing operation is completed successfully
			cod_flash_operation = 2;

			//Reset the COD flashing operation to perform other card action
			AWAOperationStatus_t.CODFlashOperation = 0x00;

			//Reset the HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_MEASURE,RESET);
			//HMI bits for MIMIC
			CoilStatusRegister_t.CoilStatus_t.measure = RESET;
			CoilStatusRegister_t.CoilStatus_t.read_acid = RESET;
			CoilStatusRegister_t.CoilStatus_t.read_sample = RESET;
			CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;

			inProgress = RESET;
		}

	  }
}

uint8_t CODADCCapture_Sensor(uint8_t command,uint8_t sens_calib_point)
{
	//variables for calculation
	float COD_RAW = 0;
	float TSS_RAW = 0;

	//this flag will be set when the PD1 and PD2 zero values have been measured and stored
	uint8_t flash = 0;

	//if the command is for cod measurernt and zeroing of the PD1 and PD2 is done will only then it will measure and perform flashing
	if((command == COD_Measure || command == AUTO_COD_MEASURE) && COD_MeasurementStatus.COD_ZERO == 0x01)
	{
		//set the flag to 1 as auto zeroing is done
		flash = 1;
	}else if(command == AUTO_COD_ZERO){
		//Enter the Flashing operation to perform the Auto Zeroing
		flash = 1;
	}else{
		//reset the common command flag as Auto Zeroing is not yet done
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		return 0;
	}
	if(flash)
	{
		//Set the COD flashing operation to prevent the program to enter into other card action
		AWAOperationStatus_t.CODFlashOperation = 0x01;
		if(TimerStart == 0x02)
		{
			TimerStart = 0x00;
			CycleStart = 1;

			htim2.Instance->DIER |= TIM_IT_UPDATE;
			htim2.Instance->CR1 |= TIM_CR1_CEN;
#if 0 //if 0, it will accept the MODBUS query even during the COD measurement process
			RxFlag = 0x00;
			//turn of the MODBUS receive query
			huart3.Instance->CR1 &= ~UART_MODE_RX;// Rx disable
			huart3.Instance->CR1 &= ~UART_IT_RXNE; // UART RX not empty interrupt disable
#endif
			dataptr1 = 0; //SSG
			dataptr = 0;

			ArrayPtr = 0;
			//	ADCInit();
			//Set the COD measure flag to "1", tp indicate the start of flashing sequence
			cod_flash_operation = 1;

			//Set the HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_MEASURE,SET); CoilStatusRegister_t.CoilStatus_t.measure = SET;
		  }

		if(TIM2State == TIME_6MS && ReadADCFlag == 1)
		{
			//HAL_Delay(1);

			TempDetNoise = 0;
			AD7682Init(PD1_CHANNEL);
			//dummy read
			//TempDetNoise = SPI_ReadData();//ReadADC(CH0);
			//actual ADC data
			TempDetNoise = SPI_ReadData();

			TempDet1Noise_Ch1[ArrayPtr] = TempDetNoise;
			//UInputReg.SInputReg.SADCReg[ArrayPtr].Det1Noise_Ch1 = TempDetNoise;//no need to send this on modbus

		  	TempDetNoise = 0;
		  	AD7682Init(PD2_CHANNEL);
		  	//dummy read
			//TempDetNoise = SPI_ReadData();//ReadADC(CH1);
			//actual ADC Data
			TempDetNoise = SPI_ReadData();

			TempDet2Noise_Ch2[ArrayPtr] = TempDetNoise;
			//UInputReg.SInputReg.SADCReg[ArrayPtr].Det2Noise_Ch2 = TempDetNoise;//no need to send this on modbus

			(dataptr1 < 99)?(dataptr1++):(dataptr1 = 500);
			ReadADCFlag = 0x00;
		}
		if(TIM2State == TIME_8MS && ReadADCFlag == 1)
		{
			TempDetSignal = 0;
			//configure the ADC for PD1 ADC channel
			AD7682Init(PD1_CHANNEL);
			//dummy read
			//TempDetSignal = SPI_ReadData();//ReadADC(CH0);
			//Actual ADC data
			TempDetSignal = SPI_ReadData();

			//store the Signal ADC reading into the array
			TempDet1Signal_Ch1[ArrayPtr] = TempDetSignal;
			//reset the buffer
			TempDetSignal = 0;
			//configure the ADC for PD2 ADC channel
			AD7682Init(PD2_CHANNEL);
			//Dummy read
			//TempDetSignal = SPI_ReadData();//ReadADC(CH1);
			//Actual ADC data
			TempDetSignal = SPI_ReadData();

			//store the Signal ADC reading into the array
			TempDet2Signal_Ch2[ArrayPtr] = TempDetSignal;

			huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			ArrayPtr++;

			(dataptr < 99)?(dataptr++):(dataptr = 500);
			ReadADCFlag = 0x00;
		}

		if(ArrayPtr >= 100) //if((dataptr1 == 500) && (dataptr == 500))
		{
			ArrayPtr = 0;

			CycleStart = 0;


			//19/10/2021
			//averaging the PD1 and PD2 noise and signal data
			for(int t=0; t<100; t++)//limit of this for loop will change according to the number of flashes selected
			{
				if(TempDet1Signal_Ch1[t] >= TempDet1Noise_Ch1[t])
					SignalNoiseDiff_Ch1[t] = TempDet1Signal_Ch1[t] - TempDet1Noise_Ch1[t];
				else
					SignalNoiseDiff_Ch1[t] = - TempDet1Signal_Ch1[t] + TempDet1Noise_Ch1[t];
				if(TempDet2Signal_Ch2[t] >= TempDet2Noise_Ch2[t])
					SignalNoiseDiff_Ch2[t] = TempDet2Signal_Ch2[t] - TempDet2Noise_Ch2[t];
				else
					SignalNoiseDiff_Ch2[t] =  - TempDet2Signal_Ch2[t] + TempDet2Noise_Ch2[t];
			}

			for(int t=0; t<100; t++)//limit of this for loop will change according to the number of flashes selected
			{
				TempMeanPd1 = TempMeanPd1 + SignalNoiseDiff_Ch1[t];
				TempMeanPd2 = TempMeanPd2 + SignalNoiseDiff_Ch2[t];
			}
			TempMeanPd1 = TempMeanPd1/100;
			TempMeanPd2 = TempMeanPd2/100;

			//Ascending order for PD1 and PD2
			bubble_sort();

			//new mean
			TempMeanPd1 = filter_data_PD1_mean;
			TempMeanPd2 = filter_data_PD2_mean;

			filter_data_PD1_mean = 0;
			filter_data_PD2_mean = 0;

			//will never be called in the sensor calibration
			//Perform this condition dusring Auto zero
			if(command == AUTO_COD_ZERO)
			{
				//store the pD1 and PD2 mean vales to the global PD1 zero and PD2 zero
				PD1_Zero = TempMeanPd1;
				PD2_Zero = TempMeanPd2;

				//publish the PD1 and PD2 zero values to the modbus
				InputRegister_t.PV_info.PD1_0 = PD1_Zero;
				InputRegister_t.PV_info.PD2_0 = PD2_Zero;

				COD_MeasurementValues_t.PD1_Zero = PD1_Zero;
				COD_MeasurementValues_t.PD2_Zero = PD2_Zero;

				//Flag set when the PD1 and PD2 zero values are measured and stored
				COD_MeasurementStatus.COD_ZERO = 0x01;
			}
			//Perfrom this condition during the measure or auto measure command
			else if(command == AUTO_COD_MEASURE || command == COD_Measure)
			{
				//store the pD1 and PD2 mean vales to the global PD1 zero and PD2 new
				PD1_new = TempMeanPd1;
				PD2_new = TempMeanPd2;

				COD_MeasurementValues_t.PD1_New = PD1_new;
				COD_MeasurementValues_t.PD2_New = PD2_new;

//----------------------------------------------------------------------------------------------------------------------------------------//
				//publish the PD1 and PD2 new values to the modbus
				//InputRegister_t.PV_info.PD1_MEAN = PD1_new;
				//InputRegister_t.PV_info.PD2_MEAN = PD2_new;
//----------------------------------------------------------------------------------------------------------------------------------------//

				//Get the PD1(0) and PD2(0) from the modbus map to the local variable
				COD_MeasurementValues_t.PD1_Zero = InputRegister_t.PV_info.PD1_0;
				COD_MeasurementValues_t.PD2_Zero = InputRegister_t.PV_info.PD2_0;

				//Get the PD2(0) from the modbus map to the local variable
				TSS_MeasurementValues_t.PD2_Zero = InputRegister_t.PV_info.TSS_PD2_0;

				//Quadratic equation - compensation (Factory Calibration)
				float c[4];//BOD
				float k[3];//TSS

				//COD coefficient
				for(int i = 0;i<4;i++)
					c[i] = HoldingRegister_t.ModeCommand_t.C[i];

				for(int i = 0; i<3;i++)
					k[i] = HoldingRegister_t.ModeCommand_t.TSS_K[i];

				//Calculations for COD raw
				COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/PD1_new) - log(COD_MeasurementValues_t.PD2_Zero/PD2_new));

				float factory_cod_value = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];

				//Calculations for TSS raw
				TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/PD2_new));

				float factory_tss_value = pow(TSS_RAW,2) *k[2] + pow(TSS_RAW,1) *k[1] + k[0];
				//Quadratic equation - compensation (Factory Calibration)
				//float c[4];
				//for(int i = 0;i<4;i++)
					//c[i] = HoldingRegister_t.ModeCommand_t.C[i];

				//COD_RAW = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];
				//float factory_cod_value = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];

//----------------------------------------------------------------------------------------------------------------------------------------//
				//Removed as COD_raw value should not be updated on the main screen

				//Remove the offset from the cod raw value
				//COD_MeasurementValues_t.RAW_Value = COD_RAW - COD_SensorCalibration_t.zero_value;
				//AWA_MeasurementHandle_t *pCOD_Meas;

				//COD actual value
				//COD_MeasurementValues_t.Cal_Value = COD_MeasurementValues_t.RAW_Value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;
				//COD_MeasurementValues_t.Cal_Value = factory_cod_value;

				//Publish the COD_RAW on modbus
				//InputRegister_t.PV_info.COD_RAW = COD_MeasurementValues_t.RAW_Value;


				//InputRegister_t.PV_info.COD_RAW = COD_RAW;
				//InputRegister_t.PV_info.CODValue = COD_MeasurementValues_t.Cal_Value;
//----------------------------------------------------------------------------------------------------------------------------------------//

				if(sens_calib_point == 1)
					HoldingRegister_t.SensorCalibration_t.COD_Y1 = factory_cod_value;
				else if(sens_calib_point == 2)
					HoldingRegister_t.SensorCalibration_t.COD_Y2 = factory_cod_value;
				else if(sens_calib_point == 3)
					HoldingRegister_t.SensorCalibration_t.COD_Y3 = factory_cod_value;
				else if(sens_calib_point == 4)
					HoldingRegister_t.SensorCalibration_t.TSS_Y1 = factory_tss_value;
				else if(sens_calib_point == 5)
					HoldingRegister_t.SensorCalibration_t.TSS_Y2 = factory_tss_value;
				else if(sens_calib_point == 6)
					HoldingRegister_t.SensorCalibration_t.TSS_Y3 = factory_tss_value;
			}

			//RSD
			double sum = 0;
			double sum2 = 0;
			double var = 0;
			double rsd = 0;
			double rsd2 = 0;

			PD1_new = TempMeanPd1;
			PD2_new = TempMeanPd2;

			//(new_value - mean)^2
			//int b = 0;
			//for(int i=  0;i<100;i++)
			//for(int i = 20,b = 0;i<80;i++,b++)
			for(int i = COD_OUT_FILTER_MIN,b = 0;i<COD_OUT_FILTER_MAX;i++,b++)
			{
				//sd[i] = pow((SignalNoiseDiff_Ch1[i] - PD1_new),2)/100;
				//sd2[i] = pow((SignalNoiseDiff_Ch2[i] - PD2_new),2)/100;

				sd[b] = pow((filter_data_PD1[i] - PD1_new),2)/COD_OUT_FILTER_RANGE;
				sd2[b] = pow((filter_data_PD2[i] - PD2_new),2)/COD_OUT_FILTER_RANGE;
			}

			//for(int i = 0;i<100;i++)
			//for(int i = 0;i<60;i++)
			for(int i = 0;i<COD_OUT_FILTER_RANGE;i++)
			{
				sum += sd[i];
				sum2 += sd2[i];
			}
			//SD
			sum = sqrt(sum);
			sum2 = sqrt(sum2);

			//variance
			var = pow(sum,2);

			//RSD
			rsd = (sum/PD1_new) * 100.0f;
			rsd2 = (sum2/PD2_new) * 100.0f;


			COD_MeasurementValues_t.RSD1 = rsd;
			COD_MeasurementValues_t.RSD2 = rsd2;

			//InputRegister_t.PV_info.RSD1 = rsd;
			//InputRegister_t.PV_info.RSD2 = rsd2;
			//printf("RSD 1: %f, RSD 2: %f",rsd,rsd2);
			//rest the PD1 and PD2 mean buffers
			TempMeanPd1 = 0;
			TempMeanPd2 = 0;

			//Turn off the timer 2
			htim2.Instance->DIER &= ~TIM_IT_UPDATE;
			htim2.Instance->CR1 &= ~TIM_CR1_CEN;

			UHoldReg.SHoldReg.ADCReadStart = 0x00;//change the command register
			TimerStart = 0x02;
			dataptr1 = 0;
			dataptr = 0;

			//Turn of the modbus receive query
			huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
			huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			//reset the common command flag
			HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;

			HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

			//set the flashing operation flag to 2 as flashing operation is completed successfully
			cod_flash_operation = 2;

			//Reset the COD flashing operation to perform other card action
			AWAOperationStatus_t.CODFlashOperation = 0x00;

			//Reset the HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_MEASURE,RESET);

			HMIInterlockingFlag(HMI_INTERLOCKING_RESETALL, RESET);

			//HMI bits for MIMIC
			CoilStatusRegister_t.CoilStatus_t.measure = RESET;
			CoilStatusRegister_t.CoilStatus_t.read_acid = RESET;
			CoilStatusRegister_t.CoilStatus_t.read_sample = RESET;
			CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
		}

	  }
}

void bubble_sort(void)
{
	uint16_t temp = 0;

	for(int i = 0;i<100;i++)
	{
		filter_data_PD1[i] = SignalNoiseDiff_Ch1[i];
		filter_data_PD2[i] = SignalNoiseDiff_Ch2[i];
	}

	//filtering of PD1
	for(int i = 0; i<99 ; i++)
	{
		for(int j = i+1 ; j<100; j++)
		{
			if(filter_data_PD1[i] >= filter_data_PD1[j])
			{
				temp = filter_data_PD1[i];
				filter_data_PD1[i] = filter_data_PD1[j];
				filter_data_PD1[j] = temp;
			}
		}
	}

	//filtering of PD2
	for(int i = 0; i<99 ; i++)
	{
		for(int j = i+1 ; j<100; j++)
		{
			if(filter_data_PD2[i] >= filter_data_PD2[j])
			{
				temp = filter_data_PD2[i];
				filter_data_PD2[i] = filter_data_PD2[j];
				filter_data_PD2[j] = temp;
			}
		}
	}
	int count = 0;
	//removed first 10 and last 10 data
	for(int i=COD_OUT_FILTER_MIN;i<COD_OUT_FILTER_MAX;i++)
	{
		filter_data_PD1_mean += filter_data_PD1[i];
		filter_data_PD2_mean += filter_data_PD2[i];
		count += 1;
	}

	//Averaged 80 points data
	filter_data_PD1_mean /= COD_OUT_FILTER_RANGE;
	filter_data_PD2_mean /= COD_OUT_FILTER_RANGE;


}

void HMIInterlockingFlag(uint8_t lock,uint8_t state)
{
	switch(lock)
	{
		case HMI_INTERLOCK_ACID_PUMP:
		case HMI_INTERLOCK_SAMPLE_PUMP:
		case HMI_INTERLOCK_MEASURE:
		case HMI_INTERLOCK_SENSOR_READACID:
		case HMI_INTERLOCK_SENSOR_READSAMPLE:
		case HMI_INTERLOCKING_RESETALL:
			if(state == SET)
				CoilStatusRegister_t.CoilStatus_t.button_press = SET;
			else
				CoilStatusRegister_t.CoilStatus_t.button_press = RESET;
			break;
//		case HMI_INTERLOCK_SENSOR_READACID:
//			CoilStatusRegister_t.CoilStatus_t.read_sample = SET;
//			break;
//		case HMI_INTERLOCK_SENSOR_READSAMPLE:
//			CoilStatusRegister_t.CoilStatus_t.read_acid = SET;
//			break;
//		case HMI_INTERLOCKING_RESETALL:
//			CoilStatusRegister_t.CoilStatus_t.read_acid = RESET;
//			CoilStatusRegister_t.CoilStatus_t.read_sample = RESET;
//			break;
	}

}

void RelayToggle(void)
{

#if 0
	if(CoilStatusRegister_t.CoilStatus_t.relay[0] == 1)
	{
		RELAY_1_ON();
	}else{
		RELAY_1_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[1] == 1)
	{
		RELAY_2_ON();
	}else{
		RELAY_2_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[2] == 1)
	{
		RELAY_3_ON();
	}else{
		RELAY_3_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[3] == 1)
	{
		RELAY_4_ON();
	}else{
		RELAY_4_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[4] == 1)
	{
		RELAY_5_ON();
	}else{
		RELAY_5_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[5] == 1)
	{
		RELAY_6_ON();
	}else{
		RELAY_6_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[6] == 1)
	{
		RELAY_7_ON();
	}else{
		RELAY_7_OFF();
	}
	if(CoilStatusRegister_t.CoilStatus_t.relay[7] == 1)
	{
		RELAY_8_ON();
	}else{
		RELAY_8_OFF();
	}
#endif
#if 1
	if(HoldingRegister_t.ModeCommand_t.RL[0] == 1)
	{
		RELAY_1_ON();
	}else{
		RELAY_1_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[1] == 1)
	{
		RELAY_2_ON();
	}else{
		RELAY_2_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[2] == 1)
	{
		RELAY_3_ON();
	}else{
		RELAY_3_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[3] == 1)
	{
		RELAY_4_ON();
	}else{
		RELAY_4_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[4] == 1)
	{
		RELAY_5_ON();
	}else{
		RELAY_5_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[5] == 1)
	{
		RELAY_6_ON();
	}else{
		RELAY_6_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[6] == 1)
	{
		RELAY_7_ON();
	}else{
		RELAY_7_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.RL[7] == 1)
	{
		RELAY_8_ON();
	}else{
		RELAY_8_OFF();
	}
#endif
}

void RelayToggleCoilInputUpdate(void)
{
	for(int i =0;i<8;i++)
	{
		CoilInputRegister_t.CoilInput_t.Relay[i] = CoilStatusRegister_t.CoilStatus_t.relay[i];
	}
}

void PumpOperation(uint8_t PumpNo)
{
	if(PUMPControlHandle_t.u8PUMP_action == OPERATION_PUMP_ON)//0x01)//pump on time
	{
		PUMPControlHandle_t.u16TIM6_count = 0;//reset
		if(PumpNo == PUMP1)//0x01)//pump1
		{
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.PUMP1_ONTIME;
			PUMPControlHandle_t.u8PUMP_no = 0x01;
			//block the ADC measurement
			PUMPControlHandle_t.u8Flag_measurement = 0x01;
			VALVE_ON();//will change according to the valve operation
			//Delay for 5 sec
			HAL_Delay(500);
			//turn on pump 1
			PUMP1_ON();
			//Set the HMI inter-locking flag for sample pump
			HMIInterlockingFlag(HMI_INTERLOCK_ACID_PUMP, SET);

			if(HoldingRegister_t.ModeCommand_t.ModeCommand_H != FACTORY_MODE)
			{
				CoilStatusRegister_t.CoilStatus_t.read_acid = SET;
				CoilStatusRegister_t.CoilStatus_t.NoProcess = RESET;
			}
		}
		else if (PumpNo == PUMP2)//0x02)//pump2
		{
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.PUMP2_ONTIME;
			PUMPControlHandle_t.u8PUMP_no = 0x02;
			//block the ADC measurement
			PUMPControlHandle_t.u8Flag_measurement = 0x01;
			VALVE_OFF();//will change according to the valve operation
			//turn off pump 2
			PUMP2_ON();
			//turn on the pump 2
			HAL_GPIO_WritePin(SYS_LED_STATUS_GPIO_Port, SYS_LED_STATUS_Pin, GPIO_PIN_RESET);
			//Set the HMI inter-locking flag for sample pump
			HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP, SET);

			if(HoldingRegister_t.ModeCommand_t.ModeCommand_H != FACTORY_MODE)
			{
				CoilStatusRegister_t.CoilStatus_t.read_sample = SET;
				CoilStatusRegister_t.CoilStatus_t.NoProcess = RESET;
			}
		}
		//Enable Interrupt
//		htim6.Instance->DIER |= TIM_IT_UPDATE; // enable timer interrupt flag
//		htim6.Instance->CR1 |= TIM_CR1_CEN; // enable timer
//		HAL_TIM_Base_Start_IT(&htim6);
		HAL_TIM_Base_Init(&htim6);
		htim6.Instance->DIER |= TIM_IT_UPDATE; // enable timer interrupt flag
		htim6.Instance->CR1 |= TIM_CR1_CEN; // enable timer
	}
	if(PUMPControlHandle_t.u8PUMP_action == OPERATION_PUMP_OFF)//0x02)//switch from the pump on/off to de-gas time (pump delay time)
	{
		PUMPControlHandle_t.u16TIM6_count = 0;//reset
		if(PumpNo == 0x01)//pump1
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.PUMP1_DELAY;
		else if (PumpNo == 0x02)//pump2
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.PUMP2_DELAY;
		HAL_GPIO_WritePin(SYS_LED_ERROR_GPIO_Port, SYS_LED_ERROR_Pin, GPIO_PIN_RESET);
		PUMPControlHandle_t.u8PUMP_delayaction = 0x01;
	}
	if(PUMPControlHandle_t.u8PUMP_action == 0x03);//pump in progress
	if(PUMPControlHandle_t.u8PUMP_action == 0x04)//operation complete (not required)
	{
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
	}
	/*Modular for Emergency stop action*/
	if(PUMPControlHandle_t.u8PUMP_action == PUMP1_TURN_OFF)//0x05)//turn off the PUMP1
	{
		//Turn off the pump 1
		PUMP1_OFF();
		//Turn off the Valve
		VALVE_OFF();
		PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
		//Set the HMI inter-locking flag for sample pump
		HMIInterlockingFlag(HMI_INTERLOCK_ACID_PUMP, RESET);
	}
	if(PUMPControlHandle_t.u8PUMP_action == PUMP2_TURN_OFF)//0x06)//turn off the PUMP2
	{
		//Turn of the pump 2
		PUMP2_OFF();
		PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
		//Reset the HMI inter-locking Flag
		HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP,RESET);
	}
	if(PUMPControlHandle_t.u8PUMP_action == 0x07)
	{
		//reset all the action
		PUMPControlHandle_t.u16TIM6_count = 0;
		PUMPControlHandle_t.u16TIM6_limit = 0;
		PUMPControlHandle_t.u8PUMP_action = 1;
		PUMPControlHandle_t.u8PUMP_delayaction = 0;
		PUMPControlHandle_t.u8PUMP_no = 0;
		//un-block the ADC measurement
		PUMPControlHandle_t.u8Flag_measurement = 0x00;
		//TEST
		HAL_GPIO_WritePin(SYS_LED_ERROR_GPIO_Port, SYS_LED_ERROR_Pin, GPIO_PIN_SET);
		//Disable interrupt
		htim6.Instance->DIER &= ~TIM_IT_UPDATE; // disable timer interrupt flag
		htim6.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer

		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;

		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
	}
}

//Check screen
void CheckScreen_PumpOperation(uint8_t PumpNo)
{
	if(PUMPControlHandle_t.u8PUMP_action == OPERATION_PUMP_ON)//0x01)//pump on time
	{
		PUMPControlHandle_t.u16TIM6_count = 0;//reset
		if(PumpNo == PUMP1)//0x01)//pump1
		{
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.CS_PUMP1_ONTIME;
			PUMPControlHandle_t.u8PUMP_no = 0x01;
			//block the ADC measurement
			PUMPControlHandle_t.u8Flag_measurement = 0x01;
			VALVE_ON();//will change according to the valve operation
			//Delay for 5 sec
			HAL_Delay(500);
			//turn on pump 1
			PUMP1_ON();
			//Set the HMI inter-locking flag for sample pump
			HMIInterlockingFlag(HMI_INTERLOCK_ACID_PUMP, SET);CoilStatusRegister_t.CoilStatus_t.read_acid = SET;CoilStatusRegister_t.CoilStatus_t.NoProcess = RESET;
		}
		else if (PumpNo == PUMP2)//0x02)//pump2
		{
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.CS_PUMP2_ONTIME;
			PUMPControlHandle_t.u8PUMP_no = 0x02;
			//block the ADC measurement
			PUMPControlHandle_t.u8Flag_measurement = 0x01;
			VALVE_OFF();//will change according to the valve operation
			//turn off pump 2
			PUMP2_ON();
			//turn on the pump 2
			HAL_GPIO_WritePin(SYS_LED_STATUS_GPIO_Port, SYS_LED_STATUS_Pin, GPIO_PIN_RESET);
			//Set the HMI inter-locking flag for sample pump
			HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP, SET);CoilStatusRegister_t.CoilStatus_t.read_sample = SET;CoilStatusRegister_t.CoilStatus_t.NoProcess = RESET;
		}
		//Enable Interrupt
//		htim6.Instance->DIER |= TIM_IT_UPDATE; // enable timer interrupt flag
//		htim6.Instance->CR1 |= TIM_CR1_CEN; // enable timer
//		HAL_TIM_Base_Start_IT(&htim6);
		HAL_TIM_Base_Init(&htim6);
		htim6.Instance->DIER |= TIM_IT_UPDATE; // enable timer interrupt flag
		htim6.Instance->CR1 |= TIM_CR1_CEN; // enable timer
	}
	if(PUMPControlHandle_t.u8PUMP_action == OPERATION_PUMP_OFF)//0x02)//switch from the pump on/off to de-gas time (pump delay time)
	{
		PUMPControlHandle_t.u16TIM6_count = 0;//reset
		if(PumpNo == 0x01)//pump1
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.CS_PUMP1_DELAY;
		else if (PumpNo == 0x02)//pump2
			PUMPControlHandle_t.u16TIM6_limit = HoldingRegister_t.ModeCommand_t.CS_PUMP2_DELAY;
		HAL_GPIO_WritePin(SYS_LED_ERROR_GPIO_Port, SYS_LED_ERROR_Pin, GPIO_PIN_RESET);
		PUMPControlHandle_t.u8PUMP_delayaction = 0x01;
	}
	if(PUMPControlHandle_t.u8PUMP_action == 0x03);//pump in progress
	if(PUMPControlHandle_t.u8PUMP_action == 0x04)//operation complete (not required)
	{
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
	}
	/*Modular for Emergency stop action*/
	if(PUMPControlHandle_t.u8PUMP_action == PUMP1_TURN_OFF)//0x05)//turn off the PUMP1
	{
		//Turn off the pump 1
		PUMP1_OFF();
		//Turn off the Valve
		VALVE_OFF();
		PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
		//Set the HMI inter-locking flag for sample pump
		HMIInterlockingFlag(HMI_INTERLOCK_ACID_PUMP, RESET);
	}
	if(PUMPControlHandle_t.u8PUMP_action == PUMP2_TURN_OFF)//0x06)//turn off the PUMP2
	{
		//Turn of the pump 2
		PUMP2_OFF();
		PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
		//Reset the HMI inter-locking Flag
		HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP,RESET);
	}
	if(PUMPControlHandle_t.u8PUMP_action == 0x07)
	{
		//reset all the action
		PUMPControlHandle_t.u16TIM6_count = 0;
		PUMPControlHandle_t.u16TIM6_limit = 0;
		PUMPControlHandle_t.u8PUMP_action = 1;
		PUMPControlHandle_t.u8PUMP_delayaction = 0;
		PUMPControlHandle_t.u8PUMP_no = 0;
		//un-block the ADC measurement
		PUMPControlHandle_t.u8Flag_measurement = 0x00;
		//TEST
		HAL_GPIO_WritePin(SYS_LED_ERROR_GPIO_Port, SYS_LED_ERROR_Pin, GPIO_PIN_SET);
		//Disable interrupt
		htim6.Instance->DIER &= ~TIM_IT_UPDATE; // disable timer interrupt flag
		htim6.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer

		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;

		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		CoilStatusRegister_t.CoilStatus_t.read_acid = RESET;
		CoilStatusRegister_t.CoilStatus_t.read_sample = RESET;
		CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
	}
}
void currentOutputTest(uint8_t CurrentChannel)
{

	uint32_t PWM_count= (CurrentTestValues_mA[HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Test_Command - 1] - CurrentOutputCalibration_t[CurrentChannel-1].AO_intercept)/CurrentOutputCalibration_t[CurrentChannel-1].AO_slope;
	PWMCurrentCalibOutput(CurrentChannel,PWM_count);
	InputRegister_t.PV_info.Iout_value[CurrentChannel - 1] = CurrentTestValues_mA[HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Test_Command - 1];
}

void analyzerRangeSelectFRAMSaveData(void)
{
	  //COD Sensor calibration data store
	  FRAM_OperationWrite(FRAM_ADDRESS_COD_SENS_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,8);
	  //TSS Sensor calibration data store
	  FRAM_OperationWrite(FRAM_ADDRESS_TSS_SENS_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,8);
	  //PH Sensor calibration store
	  FRAM_OperationWrite(FRAM_ADDRESS_pH_SENS_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,8);
	  //COD Factory 10pt calibration data store
	  FRAM_OperationWrite(FRAM_ADDRESS_COD_FACTORY_CALIB,(uint8_t*)&COD_10ptFactoryCalibrationHandle_t.bytes,96);
	  //TSS Factory 10pt calibration data store
	  FRAM_OperationWrite(FRAM_ADDRESS_TSS_FACTORY_CALIB,(uint8_t*)&TSS_10ptFactoryCalibrationHandle_t.bytes,96);
	  //Complete MODBUS data storage
	  FRAM_OperationWrite(FRAM_ADDRESS_MIN, (uint8_t*)&HoldingRegister_t.bytes, sizeof(HoldingRegister_t.bytes));
	  /*TODO:Also store the process parameter upper limits*/
	  FRAM_OperationWrite(FRAM_ADDRESS_COD_UPPERLIMIT, (uint8_t*)&COD_UpperLimit, sizeof(COD_UpperLimit));
	  //Store TSS upper limit
	  FRAM_OperationWrite(FRAM_ADDRESS_TSS_UPPERLIMIT, (uint8_t*)&TSS_UpperLimit, sizeof(TSS_UpperLimit));
	  memset(&framdata,'\0',sizeof(framdata));
	  //Store COD PD1 and PD2 zero
	  framdata.fData[0] = InputRegister_t.PV_info.PD1_0;
	  framdata.fData[1] = InputRegister_t.PV_info.PD2_0;
	  //COD PD1(0) and PD2(0)
	  FRAM_OperationWrite(FRAM_ADDRESS_COD_PD_ZERO,(uint8_t*)&framdata.bytes,8);
	  //Store TSS PD2 zero
	  memset(&framdata,'\0',sizeof(framdata));
	  framdata.fData[0] = InputRegister_t.PV_info.TSS_PD2_0;
	  //TSS PD2(0)
	  FRAM_OperationWrite(FRAM_ADDRESS_TSS_PD_ZERO,(uint8_t*)&framdata.bytes,4);
}
void ModbusSaveConfiguration(uint8_t data)
{
	  HAL_Delay(10);

	  if(data == SAVE_CONFIGURATION_DATA)//Configuration data
	  {
		  //Complete MODBUS data storage
		  FRAM_OperationWrite(FRAM_ADDRESS_MIN, (uint8_t*)&HoldingRegister_t.bytes, sizeof(HoldingRegister_t.bytes));
	  }
	  else if(data == SAVE_PROCESSPARAMETER_DATA)
	  {
		  //Reset the FRAM save Calibration flag
		  AWAOperationStatus_t.AWADataSave_ProcessValues = RESET;
		  if(AWADataStoreState.analyzerPocessvalue)
		  {
			  //Store COD,BOD,TSS and TOC values into FRAM
			  FRAM_OperationWrite(FRAM_ADDRESS_PROCESSVALUE,(uint8_t*)&InputRegister_t.bytes[0],28);

			  /*Read the Check Screen parameters*/
			  //FRAM_OperationWrite(FRAM_ADDRESS_CHECKSCREENVALUES,(uint8_t*)&HoldingRegister_t.ModeCommand_t.CS_PD_1_MIN,16);

			  //Complete MODBUS data storage
			  FRAM_OperationWrite(FRAM_ADDRESS_MIN, (uint8_t*)&HoldingRegister_t.bytes, sizeof(HoldingRegister_t.bytes));

			  AWADataStoreState.analyzerPocessvalue = RESET;
		  }
	  }
	  else if(data == SAVE_CALIBRATION_DATA)//Calibration data storage
	  {
		  //Reset the FRAM save Calibration flag
		  AWAOperationStatus_t.AWADataSave_Calibration = 0x00;

		  //TEST
		  if(AWADataStoreState.electronicAO)
		  {
			  //AO Electronic calibration data store
		//	  FRAM_OperationWrite(FRAM_ADDRESS_MIN + sizeof(HoldingRegister_t.bytes),(uint8_t*)&CurrentOutputCalibration_t,sizeof(CurrentOutputCalibration_t));
			  FRAM_OperationWrite(FRAM_ADDRESS_AO_ELEC_CALIB,(uint8_t*)&CurrentOutputCalibration_t,sizeof(CurrentOutputCalibration_t) * 8);

			  AWADataStoreState.electronicAO = RESET;
		  }
		  if(AWADataStoreState.electronicPT)
		  {
			  //PT temperature sensor electronic calibration storage (PT-100 AND PT-1000)
			  FRAM_OperationWrite(FRAM_ADDRESS_PT_ELEC_CALIB,(uint8_t*)&PT_ElectronicCalibration_t.bytes,16);

			  AWADataStoreState.electronicPT = RESET;
		  }
		  if(AWADataStoreState.electronicpH)
		  {
			  //pH Electronic calibration storage
			  FRAM_OperationWrite(FRAM_ADDRESS_pH_ELEC_CALIB,(uint8_t*)&pH_ElectronicCalibpoints_t.bytes[8],8);

			  AWADataStoreState.electronicpH = RESET;
		  }
		  if(AWADataStoreState.sensorCOD)
		  {
			  //COD Sensor calibration data store
			  //FRAM_OperationWrite(FRAM_ADDRESS_COD_SENS_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,8);
			  FRAM_OperationWrite(FRAM_ADDRESS_COD_SENSOR_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,48);

			  //Storing in Last calibration Space
			  FRAM_OperationWrite(FRAM_ADDRESS_CODSENSORLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 328],204); //Storing COD factory calibration with overflow flag

			  AWADataStoreState.sensorCOD = RESET;
		  }
		  if(AWADataStoreState.sensorTSS)
		  {
			  //TSS Sensor calibration data store
			  //FRAM_OperationWrite(FRAM_ADDRESS_TSS_SENS_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,8);
			  FRAM_OperationWrite(FRAM_ADDRESS_TSS_SENSOR_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,48);

			  //Storing in Last calibration Space
			  //FRAM_OperationWrite(FRAM_ADDRESS_TSSSENSORLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 452],204); //Storing COD factory calibration with overflow flag
			  FRAM_OperationWrite(FRAM_ADDRESS_TSSSENSORLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 532],204); //Storing COD factory calibration with overflow flag
			  AWADataStoreState.sensorTSS = RESET;
		  }
		  if(AWADataStoreState.sensorpH)
		  {
			  //PH Sensor calibration store
			  //FRAM_OperationWrite(FRAM_ADDRESS_pH_SENS_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,32);
			  FRAM_OperationWrite(FRAM_ADDRESS_PH_SENSOR_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,90);

			  //Storing in Last calibration Space
			  //FRAM_OperationWrite(FRAM_ADDRESS_pHSENSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 576],124); //Storing COD factory calibration with overflow flag
			  FRAM_OperationWrite(FRAM_ADDRESS_PHSENSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 736],204); //Storing COD factory calibration with overflow flag
			  AWADataStoreState.sensorpH = RESET;
		  }
		  if(AWADataStoreState.factoryCOD)
		  {
			  //COD Factory 10pt calibration data store
			  FRAM_OperationWrite(FRAM_ADDRESS_COD_FACTORY_CALIB,(uint8_t*)&COD_10ptFactoryCalibrationHandle_t.bytes,96); //Yet to save the COD SF, total bytes 100

			  //Storing in Last calibration Space
			  FRAM_OperationWrite(FRAM_ADDRESS_LASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t)],164); //Storing COD factory calibration with overflow flag

			  AWADataStoreState.factoryCOD = RESET;

			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.factoryCOD_setzero)
		  {
			  memset(&framdata,'\0',sizeof(framdata));

			  framdata.fData[0] = InputRegister_t.PV_info.PD1_0;
			  framdata.fData[1] = InputRegister_t.PV_info.PD2_0;
			  //COD PD1(0) and PD2(0)
			  FRAM_OperationWrite(FRAM_ADDRESS_COD_PD_ZERO,(uint8_t*)&framdata.bytes,8);

			  AWADataStoreState.factoryCOD_setzero = RESET;

			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.factoryTSS)
		  {
			  //TSS Factory 10pt calibration data store
			  FRAM_OperationWrite(FRAM_ADDRESS_TSS_FACTORY_CALIB,(uint8_t*)&TSS_10ptFactoryCalibrationHandle_t.bytes,96); //Yet to save the COD SF, total bytes 100

			  //Storing in Last calibration Space
			  FRAM_OperationWrite(FRAM_ADDRESS_TSSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 164],164); //Storing COD factory calibration with overflow flag

			  AWADataStoreState.factoryTSS = RESET;

			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.factoryTSS_setzero)
		  {
			  memset(&framdata,'\0',sizeof(framdata));

			  framdata.fData[0] = InputRegister_t.PV_info.TSS_PD2_0;
			  //TSS PD2(0)
			  FRAM_OperationWrite(FRAM_ADDRESS_TSS_PD_ZERO,(uint8_t*)&framdata.bytes,4);

			  AWADataStoreState.factoryTSS_setzero = RESET;

			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.analyzerRangeSelect)
		  {
			  uint8_t flag = 0;
			  flag = SET;
			  FRAM_OperationWrite(FRAM_ADDRESS_RANGESELECT_FLAG,(uint8_t*)&flag,1);
			  AWADataStoreState.analyzerRangeSelect = RESET;
			  /*Save the new configuration data into FRAM*/
			  analyzerRangeSelectFRAMSaveData();

			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.modbusConfigport1)
		  {

			  FRAM_OperationWrite(FRAM_ADDRESS_MODBUSCONFIGURATION,(uint8_t*)&PORT_1,sizeof(PORT_1));
			  AWADataStoreState.modbusConfigport1 = RESET;
			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.modbusConfigport2)
		  {

			  FRAM_OperationWrite(FRAM_ADDRESS_MODBUSCONFIGURATION + sizeof(PORT_1),(uint8_t*)&PORT_2,sizeof(PORT_1));
			  AWADataStoreState.modbusConfigport2 = RESET;
			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }
		  if(AWADataStoreState.modbusConfigport3)
		  {

			  FRAM_OperationWrite(FRAM_ADDRESS_MODBUSCONFIGURATION + (2 * sizeof(PORT_1)),(uint8_t*)&PORT_3,sizeof(PORT_1));
			  AWADataStoreState.modbusConfigport3 = RESET;
			  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
		  }

	  }


}

void MODBUSInit(void)
{
	memcpy(&HoldingRegister_t.MODBUS_PORTConfig_t.PORT_2,&PORT_2,sizeof(PORT_2));
	memcpy(&HoldingRegister_t.MODBUS_PORTConfig_t.PORT_3,&PORT_3,sizeof(PORT_3));

	//Select the baudrate according to the selection
	baudrateSelection(&PORT_2);
	baudrateSelection(&PORT_3);

	//Publish the selected baudrate value on the MODBUS
	HoldingRegister_t.MODBUS_PORTConfig_t.PORT_2.baudrate = PORT_2.baudrate;
	HoldingRegister_t.MODBUS_PORTConfig_t.PORT_3.baudrate = PORT_3.baudrate;

	//Reset the common command
	HoldingRegister_t.ModeCommand_t.CommonCommand = RESET;

	//Stop the MODBUS communication
	resetMODBUSComm(&huart1, &htim5);
	resetMODBUSComm(&huart3, &htim12);

	//Configure the new MODBUS configuration
	MODBUSConfiguration(&huart1, PORT_2);
	MODBUSConfiguration(&huart3, PORT_3);

	//Start the MODBUS communication
	beginMODBUSComm(&htim12, &uart_rx_timeout_counter_uart3, &uart_rx_bytes_uart3, &MOD3_RxFlag,3);
	beginMODBUSComm(&htim5, &uart_rx_timeout_counter_uart1, &uart_rx_bytes_uart1, &RxFlag,2);
}
void ModbusReadConfiguration(void)
{
	//Index count for last calibration data
	uint8_t index_count = 0;

	//Read modbus configuration
//	FRAM_OperationRead(FRAM_ADDRESS_MODBUSCONFIGURATION + (1 * sizeof(PORT_1)),(uint8_t*)&PORT_2,sizeof(PORT_1));
//	FRAM_OperationRead(FRAM_ADDRESS_MODBUSCONFIGURATION + (2 * sizeof(PORT_1)),(uint8_t*)&PORT_3,sizeof(PORT_1));
//	MODBUSInit();

	//Read the complete MODBUS configuration from FRAM
	FRAM_OperationRead(FRAM_ADDRESS_MIN, (uint8_t*)&HoldingRegister_t.bytes, sizeof(HoldingRegister_t.bytes));

	//Read the AO electronic calibration data
	FRAM_OperationRead(FRAM_ADDRESS_AO_ELEC_CALIB, (uint8_t*)&CurrentOutputCalibration_t, sizeof(CurrentOutputCalibration_t));

	//Read the pH electronic calibration data
	FRAM_OperationRead(FRAM_ADDRESS_pH_ELEC_CALIB, (uint8_t*)&pH_ElectronicCalibpoints_t.bytes[8], 8);

	//Read the pH sensor calibration data
	//FRAM_OperationRead(FRAM_ADDRESS_pH_SENS_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,32);
	FRAM_OperationRead(FRAM_ADDRESS_PH_SENSOR_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,90);

	HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_value = pH_SensorCalibpoints_t.pH_Calculated_1_x1; //calculated pH values
	HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = pH_SensorCalibpoints_t.pH_Calculated_2_x2; //calculated pH values
	HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_value = pH_SensorCalibpoints_t.pH_Calculated_3_x3; //calculated pH values
	HoldingRegister_t.SensorCalibration_t.pH_PT.Y1 = pH_SensorCalibpoints_t.pH_Solution_1_y1;				//Simulation solution
	HoldingRegister_t.SensorCalibration_t.pH_PT.Y2 = pH_SensorCalibpoints_t.pH_Solution_2_y2;				//Simulation solution
	HoldingRegister_t.SensorCalibration_t.pH_Y3 = pH_SensorCalibpoints_t.pH_Solution_3_y3;				//Simulation solution
	HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;		//ADC counts
	HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_counts_2_x2;		//ADC counts
	HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_count = pH_SensorCalibpoints_t.pH_counts_3_x3;		//ADC counts
	HoldingRegister_t.SensorCalibration_t.pH_slope = pH_SensorCalibpoints_t.pH_Solpe;
	HoldingRegister_t.SensorCalibration_t.pH_intercept = pH_SensorCalibpoints_t.pH_Intercept;
	HoldingRegister_t.SensorCalibration_t.pH_slope_range_2 = pH_SensorCalibpoints_t.pH_slope_range_2;
	HoldingRegister_t.SensorCalibration_t.pH_intercept_range_2 = pH_SensorCalibpoints_t.pH_Intercept_range_2;
	//HoldingRegister_t.SensorCalibration_t.Calibration_Type = pH_SensorCalibpoints_t.measurement_Type;
	//HoldingRegister_t.SensorCalibration_t.pH_1pt_calib_point1 = pH_SensorCalibpoints_t.pH_Solution_1_y1;	//Simulation solution 1
	//HoldingRegister_t.SensorCalibration_t.pH_1pt_Cal_point_1_value = pH_SensorCalibpoints_t.

	//Read the PT-100 and PT-1000 Electronic calibration data
	FRAM_OperationRead(FRAM_ADDRESS_PT_ELEC_CALIB,(uint8_t*)&PT_ElectronicCalibration_t.bytes,16);

	//Read the COD Electronic Calibration Data
	//FRAM_OperationRead(FRAM_ADDRESS_COD_SENS_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,8);
	FRAM_OperationRead(FRAM_ADDRESS_COD_SENSOR_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,90);//Yet to make changes according to 3-pt calibration

	//Publish to modbus
	HoldingRegister_t.SensorCalibration_t.COD_CF = COD_SensorCalibration_t.slope;
	HoldingRegister_t.SensorCalibration_t.COD_Intercept = COD_SensorCalibration_t.intercept;
	HoldingRegister_t.SensorCalibration_t.COD_CF_RANGE_2 = COD_SensorCalibration_t.slope_range_2;
	HoldingRegister_t.SensorCalibration_t.COD_Intercept_RANGE_2 = COD_SensorCalibration_t.intercept_range_2;

	//Read the COD Factory 10-pt Calibration Data
	FRAM_OperationRead(FRAM_ADDRESS_COD_FACTORY_CALIB,(uint8_t*)&COD_10ptFactoryCalibrationHandle_t.bytes,96);

	for(int i = 0;i<10;i++)
	{
		HoldingRegister_t.ModeCommand_t.COD_X[i] = COD_10ptFactoryCalibrationHandle_t.x[i];
		HoldingRegister_t.ModeCommand_t.COD_Y[i] = COD_10ptFactoryCalibrationHandle_t.y[i];
	}

	for(int i = 0;i<3;i++)
	{
		HoldingRegister_t.ModeCommand_t.C[i] = COD_10ptFactoryCalibrationHandle_t.c[i];
	}


	//COD Factory 10pt calibration data store
	FRAM_OperationRead(FRAM_ADDRESS_COD_PD_ZERO,(uint8_t*)&framdata.bytes,8); //Yet to save the COD SF, total bytes 100

	InputRegister_t.PV_info.PD1_0 = framdata.fData[0];
	InputRegister_t.PV_info.PD2_0 = framdata.fData[1];

	//Read the TSS Factory 10-pt Calibration Data
	FRAM_OperationRead(FRAM_ADDRESS_TSS_FACTORY_CALIB,(uint8_t*)&TSS_10ptFactoryCalibrationHandle_t.bytes,96);

	for(int i = 0;i<10;i++)
	{
		HoldingRegister_t.ModeCommand_t.TSS_G[i] = TSS_10ptFactoryCalibrationHandle_t.y[i];
		HoldingRegister_t.ModeCommand_t.TSS_F[i] = TSS_10ptFactoryCalibrationHandle_t.x[i];
	}

	for(int i = 0;i<3;i++)
	{
		HoldingRegister_t.ModeCommand_t.TSS_K[i] = TSS_10ptFactoryCalibrationHandle_t.c[i];
	}

	memset(&framdata,'\0',sizeof(framdata));
	//COD Factory 10pt calibration data store
	FRAM_OperationRead(FRAM_ADDRESS_TSS_PD_ZERO,(uint8_t*)&framdata.bytes,4); //Yet to save the COD SF, total bytes 100

	InputRegister_t.PV_info.TSS_PD2_0 = framdata.fData[0];

	//Read the TSS Sensor Calibration Data
	//FRAM_OperationRead(FRAM_ADDRESS_TSS_SENS_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,8);
	FRAM_OperationRead(FRAM_ADDRESS_TSS_SENSOR_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,48);

	//Publish to modbus
	HoldingRegister_t.SensorCalibration_t.TSS_CF = TSS_SensorCalibration_t.slope;
	HoldingRegister_t.SensorCalibration_t.TSS_Intercept = TSS_SensorCalibration_t.intercept;
	HoldingRegister_t.SensorCalibration_t.TSS_CF_RANGE_2 = TSS_SensorCalibration_t.slope_range_2;
	HoldingRegister_t.SensorCalibration_t.TSS_Intercept_RANGE_2 = TSS_SensorCalibration_t.intercept_range_2;

	//Storing in Last calibration Space
	FRAM_OperationRead(FRAM_ADDRESS_LASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t)],164); //Storing only COD factory calibration with overflow flag
	/*<TODO: Store the data from Input registers to Linked List*/
	/*If the overflow flag is not set*/
	if(InputRegister_t.COD_lastCalibration.overflowFlag != SET)
	{
		/*
		 * Last calibration data is within 10
		 */
		for(int i = 0;i<10;i++)
		{
			if(InputRegister_t.COD_lastCalibration.epochtimestamp[i] != 0)
			{
				index_count += 1;
			}
		}

		/*Store the index count in the index*/
		AWALastCalibrationCount.factoryCOD_count = index_count;
	}
	/*If the overflow flag is SET*/
	else
	{
		index_count = 10;
	}

	for(int i = (index_count - 1);(InputRegister_t.COD_lastCalibration.epochtimestamp[i] != 0) & (i >=0 );i--)
	{
		/*Create new node and insert at the end*/
		factory_insertNode(&CODFactoryhead,
							InputRegister_t.COD_lastCalibration.C0[i],
							InputRegister_t.COD_lastCalibration.C1[i],
							InputRegister_t.COD_lastCalibration.C2[i],
							InputRegister_t.COD_lastCalibration.epochtimestamp[i]);
	}
	/*Read the overflow flag*/
	AWALastCalibrationCount.factoryCOD_overflowflag = InputRegister_t.COD_lastCalibration.overflowFlag;
	/*Reset the local variable*/
	index_count = 0;

	//Storing in Last calibration Space
	FRAM_OperationRead(FRAM_ADDRESS_TSSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 164],164); //Storing only COD factory calibration with overflow flag

	/*<TODO: Store the data from Input registers to Linked List*/
	/*If the overflow flag is not set*/
	if(InputRegister_t.TSS_lastCalibration.overflowFlag != SET)
	{
		/*
		 * Last calibration data is within 10
		 */
		for(int i = 0;i<10;i++)
		{
			if(InputRegister_t.TSS_lastCalibration.epochtimestamp[i] != 0)
			{
				index_count += 1;
			}
		}

		/*Store the index count in the index*/
		AWALastCalibrationCount.factoryTSS_count = index_count;
	}
	/*If the overflow flag is SET*/
	else
	{
		index_count = 10;
	}
	for(int i = (index_count - 1);(InputRegister_t.TSS_lastCalibration.epochtimestamp[i] != 0) & (i >=0 );i--)
	{
		/*Create new node and insert at the end*/
		factory_insertNode(&TSSFactoryhead,
							InputRegister_t.TSS_lastCalibration.C0[i],
							InputRegister_t.TSS_lastCalibration.C1[i],
							InputRegister_t.TSS_lastCalibration.C2[i],
							InputRegister_t.TSS_lastCalibration.epochtimestamp[i]);
	}
	/*Read the overflow flag*/
	AWALastCalibrationCount.factoryTSS_overflowflag = InputRegister_t.TSS_lastCalibration.overflowFlag;
	/*Reset the local variable*/
	index_count = 0;

	/*For COD Sensor last Calibration reading from FRAM*/
	//Storing in Last calibration Space
	//FRAM_OperationRead(FRAM_ADDRESS_CODSENSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 328],124); //Storing only COD factory calibration with overflow flag
	FRAM_OperationRead(FRAM_ADDRESS_CODSENSORLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 328],204); //Storing only COD factory calibration with overflow flag

	/*<TODO: Store the data from Input registers to Linked List*/
	/*If the overflow flag is not set*/
	if(InputRegister_t.COD_lastSensorCalibration.overflowFlag != SET)
	{
		/*
		 * Last calibration data is within 10
		 */
		for(int i = 0;i<10;i++)
		{
			if(InputRegister_t.COD_lastSensorCalibration.epochtimestamp[i] != 0)
			{
				index_count += 1;
			}
		}

		/*Store the index count in the index*/
		AWALastCalibrationCount.sensorCOD_count = index_count;
	}
	/*If the overflow flag is SET*/
	else
	{
		index_count = 10;
	}
	for(int i = (index_count - 1);(InputRegister_t.COD_lastSensorCalibration.epochtimestamp[i] != 0) & (i >=0 );i--)
	{
		/*Create new node and insert at the end*/
		sensor_3pt_insertNode(&CODSensorhead,
							InputRegister_t.COD_lastSensorCalibration.intercept[i],
							InputRegister_t.COD_lastSensorCalibration.slope[i],
							InputRegister_t.COD_lastSensorCalibration.intercept_range_2[i],
							InputRegister_t.COD_lastSensorCalibration.slope_range_2[i],
							InputRegister_t.COD_lastSensorCalibration.epochtimestamp[i]);
	}
	/*Read the overflow flag*/
	AWALastCalibrationCount.sensorCOD_overflowflag = InputRegister_t.COD_lastSensorCalibration.overflowFlag;
	/*Reset the local variable*/
	index_count = 0;

	/*For TSS Sensor last Calibration reading from FRAM*/
	//Storing in Last calibration Space
	//FRAM_OperationRead(FRAM_ADDRESS_TSSSENSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 452],124); //Storing only COD factory calibration with overflow flag
	FRAM_OperationRead(FRAM_ADDRESS_TSSSENSORLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 532],204); //Storing only COD factory calibration with overflow flag

	/*<TODO: Store the data from Input registers to Linked List*/
	/*If the overflow flag is not set*/
	if(InputRegister_t.TSS_lastSensorCalibration.overflowFlag != SET)
	{
		/*
		 * Last calibration data is within 10
		 */
		for(int i = 0;i<10;i++)
		{
			if(InputRegister_t.TSS_lastSensorCalibration.epochtimestamp[i] != 0)
			{
				index_count += 1;
			}
		}

		/*Store the index count in the index*/
		AWALastCalibrationCount.sensorTSS_count = index_count;
	}
	/*If the overflow flag is SET*/
	else
	{
		index_count = 10;
	}
	for(int i = (index_count - 1);(InputRegister_t.TSS_lastSensorCalibration.epochtimestamp[i] != 0) & (i >=0 );i--)
	{
		/*Create new node and insert at the end*/
		sensor_3pt_insertNode(&TSSSensorhead,
							InputRegister_t.TSS_lastSensorCalibration.intercept[i],
							InputRegister_t.TSS_lastSensorCalibration.slope[i],
							InputRegister_t.TSS_lastSensorCalibration.intercept_range_2[i],
							InputRegister_t.TSS_lastSensorCalibration.slope_range_2[i],
							InputRegister_t.TSS_lastSensorCalibration.epochtimestamp[i]);
	}
	/*Read the overflow flag*/
	AWALastCalibrationCount.sensorTSS_overflowflag = InputRegister_t.TSS_lastSensorCalibration.overflowFlag;


	/*
	 * Read the pH Sensor calibration last data
	 */

	/*Reset the local variable*/
	index_count = 0;

	//Storing in Last calibration Space
	//FRAM_OperationRead(FRAM_ADDRESS_pHSENSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 576],124); //Storing only COD factory calibration with overflow flag
	FRAM_OperationRead(FRAM_ADDRESS_PHSENSLASTCALIB_HISTORY,(uint8_t*)&InputRegister_t.bytes[sizeof(PVhandle_t) + 736],204); //Storing only COD factory calibration with overflow flag

	/*<TODO: Store the data from Input registers to Linked List*/
	/*If the overflow flag is not set*/
	if(InputRegister_t.pH_lastSensorCalibration.overflowFlag != SET)
	{
		/*
		 * Last calibration data is within 10
		 */
		for(int i = 0;i<10;i++)
		{
			if(InputRegister_t.pH_lastSensorCalibration.epochtimestamp[i] != 0)
			{
				index_count += 1;
			}
		}

		/*Store the index count in the index*/
		AWALastCalibrationCount.sensorpH_count = index_count;
	}
	/*If the overflow flag is SET*/
	else
	{
		index_count = 10;
	}
	for(int i = (index_count - 1);(InputRegister_t.pH_lastSensorCalibration.epochtimestamp[i] != 0) & (i >=0 );i--)
	{
		/*Create new node and insert at the end*/
		sensor_3pt_insertNode(&pHSensorhead,
							InputRegister_t.pH_lastSensorCalibration.intercept[i],
							InputRegister_t.pH_lastSensorCalibration.slope[i],
							InputRegister_t.pH_lastSensorCalibration.intercept_range_2[i],
							InputRegister_t.pH_lastSensorCalibration.slope_range_2[i],
							InputRegister_t.pH_lastSensorCalibration.epochtimestamp[i]);
	}
	/*Read the overflow flag*/
	AWALastCalibrationCount.sensorpH_overflowflag = InputRegister_t.pH_lastSensorCalibration.overflowFlag;
/**************************/

	/*Reading the default range select option*/
	uint8_t flag = 0;
	FRAM_OperationRead(FRAM_ADDRESS_RANGESELECT_FLAG,(uint8_t*)&flag,1);
	/*Check if flag is SET*/
	if(flag == SET)
	{
	  //if the flag is set then do not load with default analyzer configuration i.e; COD 300 mg/l
	  AWALastCalibrationCount.analyzerRangeSelectflag = SET;
		FRAM_OperationRead(FRAM_ADDRESS_MODBUSCONFIGURATION + (1 * sizeof(PORT_1)),(uint8_t*)&PORT_2,sizeof(PORT_1));
		FRAM_OperationRead(FRAM_ADDRESS_MODBUSCONFIGURATION + (2 * sizeof(PORT_1)),(uint8_t*)&PORT_3,sizeof(PORT_1));
		MODBUSInit();
	}else
	{
	  /*Booting first time or analyzer range is not selected by the user, load with the default configuration COD = 300 mg/l*/
		HoldingRegister_t.ModeCommand_t.RANGESELECT = MODEL_3011_3012;
		//Set the Default data to MODBUS and Live Data
		AWA_RangeSelect();
		//Store the data in FRAM
		AWADataStoreState.analyzerRangeSelect = SET;
		//Flag set as data not saved in the FRAM
		AWAOperationStatus_t.AWADataSave_Calibration = 0x01;


		PORT_2.baudrate = 38400;
		PORT_2.baudrate_selection = 4;
		PORT_2.data_length = 1;
		PORT_2.parity_bit = 0;
		PORT_2.slave_ID = 2;
		PORT_2.stop_bits = 1;

		PORT_3.baudrate = 38400;
		PORT_3.baudrate_selection = 4;
		PORT_3.data_length = 1;
		PORT_3.parity_bit = 0;
		PORT_3.slave_ID = 2;
		PORT_3.stop_bits = 1;
		MODBUSInit();


	}

	/*Get the upper limits for COD and TSS*/
	FRAM_OperationRead(FRAM_ADDRESS_COD_UPPERLIMIT,(uint8_t*)&COD_UpperLimit,sizeof(COD_UpperLimit));
	BOD_UpperLimit = COD_UpperLimit / 2.0f;
	FRAM_OperationRead(FRAM_ADDRESS_TSS_UPPERLIMIT,(uint8_t*)&TSS_UpperLimit,sizeof(TSS_UpperLimit));

	/*Read the process parameters*/
	FRAM_OperationRead(FRAM_ADDRESS_PROCESSVALUE,(uint8_t*)&InputRegister_t.bytes[0],28);

	/*Read the Check Screen parameters*/
	//FRAM_OperationRead(FRAM_ADDRESS_CHECKSCREENVALUES,(uint8_t*)&HoldingRegister_t.ModeCommand_t.CS_PD_1_MIN,16);
}


static inline void COD_SensorCalib_ypoint(uint8_t pt)
{

	HoldingRegister_t.SensorCalibration_t.COD_Messages = RESET;
#if 1
	if(HoldingRegister_t.SensorCalibration_t.RES_1[0] == 1)
	{
		HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_READACID, SET);
	//First operate the sample pump
		PumpOperation(0x01);
	}
	else if(HoldingRegister_t.SensorCalibration_t.RES_1[0] == 2)
	{
		HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_READSAMPLE, SET);
		PumpOperation(0x02);
	}
#else
	if(pt == 1 || pt == 4)
	{
		HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_READACID, SET);
	//First operate the sample pump
		PumpOperation(0x01);
	}
	else
	{
		HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_READSAMPLE, SET);
		PumpOperation(0x02);
	}
#endif

	//perform the ADC measurement action when the pump action is completed
	if(!PUMPControlHandle_t.u8Flag_measurement)
	{
		HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
		point = pt;
	}
}

void ParameterIOutProcess(void)
{
	for(int channel = 1;channel < 5; channel++)
	{
		//Check if the index of the parameter is non-zero
		if(HoldingRegister_t.IOUTConfig_t[channel-1].Current_OP_option <= 0)
			continue;
		else
		{
			float Parameter_min = HoldingRegister_t.IOUTConfig_t[channel-1].Current_OP_MIN_Value;
			float Parameter_max = HoldingRegister_t.IOUTConfig_t[channel-1].Current_OP_MAX_Value;
			float Parameter_Value = ParameterValues_t.value[HoldingRegister_t.IOUTConfig_t[channel-1].Current_OP_option-1];
			float OP_CurrentValue = 4.0f +(((Parameter_Value - Parameter_min)/(Parameter_max - Parameter_min)) * (20.0f - 4.0f));
			uint32_t PWM_count= (OP_CurrentValue - CurrentOutputCalibration_t[channel-1].AO_intercept)/CurrentOutputCalibration_t[channel-1].AO_slope;
			PWMCurrentCalibOutput(channel,PWM_count);
			InputRegister_t.PV_info.Iout_value[channel - 1] = OP_CurrentValue;
		}
	}
}


void ParameterRelayAlarmProcess(void)
{
	for(int relay = 0;relay <8; relay++)
	{
		float threshold = HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_Threshold;
		uint16_t parameterIndex = HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_Parameter - 1;
//		if(parameterIndex >	5)
//			return 0;
		float hysterisis =  HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_Hysterisis; /*<Percentage*/
		float parameterValue = ParameterValues_t.value[parameterIndex];
		hysterisis = threshold * hysterisis / 100.0f; /*<Value of hysteresis from percentage*/
		if(HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_HIGHLOW == 0x01)//high
		{
			if(parameterValue >= threshold)
				HoldingRegister_t.ModeCommand_t.RL[relay] = 0x01;//alarm high
			if(parameterValue <= threshold - hysterisis)
				HoldingRegister_t.ModeCommand_t.RL[relay] = 0x00;//alarm low
		}else if(HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_HIGHLOW == 0x02)//low
		{
			if(parameterValue <= threshold)
				HoldingRegister_t.ModeCommand_t.RL[relay] = 0x01;//alarm high
			if(parameterValue >= threshold + hysterisis)
				HoldingRegister_t.ModeCommand_t.RL[relay] = 0x00;//alarm low
		}
	}
	RelayToggle();
//	RelayToggleCoilInputUpdate();
}

/*
 * Function name :- Application_ReadAcid
 * Task :- Runs the acid pump and measures the raw absorbance.
 */
void Application_ReadAcid(void)
{
	//Run the acid pump
	PumpOperation(0x01);

	//perform the ADC measurement action when the pump action is completed
	if(!PUMPControlHandle_t.u8Flag_measurement)
	{
		HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
//		CODADCCapture(COD_Measure);
	}
}

/*
 * Function name :- Application_ReadSample
 * Task :- Runs the acid pump and measures the raw absorbance.
 */
void Application_ReadSample(void)
{
	//Run the acid pump
	PumpOperation(0x02);

	//perform the ADC measurement action when the pump action is completed
	if(!PUMPControlHandle_t.u8Flag_measurement)
	{
		HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
//		CODADCCapture(COD_Measure);
	}
}

/*
 * Function name :- Application_SetAsZero
 * Task :- Overwrite the existing PD1(0) and PD2(0) by taking the current PD1 and PD2 values.
 */
void Application_SetAsZero(uint8_t parameter)
{
	if(parameter == 1) /*< Parameter = COD*/
	{
		//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
		COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
		COD_MeasurementValues_t.PD2_Zero = COD_MeasurementValues_t.PD2_New;

		//Calculate the COD RAW
		float COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/COD_MeasurementValues_t.PD1_New) - log(COD_MeasurementValues_t.PD2_Zero/COD_MeasurementValues_t.PD2_New));
		//Publish the values to the MODBUS
		InputRegister_t.PV_info.COD_RAW = COD_RAW;
		InputRegister_t.PV_info.PD1_0 = COD_MeasurementValues_t.PD1_Zero;
		InputRegister_t.PV_info.PD2_0 = COD_MeasurementValues_t.PD2_Zero;
		//Reset the command
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
		//Flag set as data not saved in the FRAM
		AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
	}
	else
	{
		//set the current PD1 and PD2 mean values to the PD1(0) and PD2(0)
		//COD_MeasurementValues_t.PD1_Zero = COD_MeasurementValues_t.PD1_New;
		TSS_MeasurementValues_t.PD2_Zero = TSS_MeasurementValues_t.PD2_New;

		//Calculate the COD RAW
		float TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/TSS_MeasurementValues_t.PD2_New));
		//Publish the values to the MODBUS
		InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
		InputRegister_t.PV_info.TSS_PD2_0 = TSS_MeasurementValues_t.PD2_Zero;
		//Reset the command
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
		//Flag set as data not saved in the FRAM
		AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
	}
}




void sensor_insertNode(struct Sensornode **head,float intercept,float slope ,unsigned int timestamp)
{
	/*Create a new node and allocate the memory*/
    struct Sensornode *t;
    t = (struct Sensornode*)calloc(1,sizeof(struct Sensornode));

    /*Store the intercept, slope and epoch time stamp*/
    t->intercept = intercept;
    t->slope = slope;
    t->timestamp = timestamp;
    /*Point to NULL address*/
    t->next = NULL;

    /*Check if the the linked list has no nodes*/
    if(*head == NULL)
    {
    	/*If no nodes are affable then assign the new node address to the head.*/
        *head = t;
    }
    /*If nodes are present*/
    else{
    	/*Make the copy of the linked list by assigning the head address to the temporary nodes*/
        struct Sensornode *temp;
        temp = *head;
        /*Traverse until it reaches the last node*/
        while(temp->next != NULL)
        {
            temp = temp->next;
        }
        /*Assign the address of the new node to the last node.*/
        temp->next = t;
    }
}

void sensor_3pt_insertNode(struct Sensornode_3pt **head,float intercept,float slope ,float intercept_range_2,float slope_range_2,unsigned int timestamp)
{
	/*Create a new node and allocate the memory*/
    struct Sensornode_3pt *t;
    t = (struct Sensornode_3pt*)calloc(1,sizeof(struct Sensornode_3pt));

    /*Store the intercept, slope and epoch time stamp*/
    t->intercept = intercept;
    t->slope = slope;
    t->intercept_3pt = intercept_range_2;
    t->slope_3pt = slope_range_2;
    t->timestamp = timestamp;
    /*Point to NULL address*/
    t->next = NULL;

    /*Check if the the linked list has no nodes*/
    if(*head == NULL)
    {
    	/*If no nodes are affable then assign the new node address to the head.*/
        *head = t;
    }
    /*If nodes are present*/
    else{
    	/*Make the copy of the linked list by assigning the head address to the temporary nodes*/
        struct Sensornode_3pt *temp;
        temp = *head;
        /*Traverse until it reaches the last node*/
        while(temp->next != NULL)
        {
            temp = temp->next;
        }
        /*Assign the address of the new node to the last node.*/
        temp->next = t;
    }
}


void sensor_deleteNode(struct Sensornode **head)
{
    struct Sensornode *temp,*temp_next;
    temp = *head;
    temp_next = temp->next;
    free(temp);
    *head = temp_next;
}

void sensor_3pt_deleteNode(struct Sensornode_3pt **head)
{
    struct Sensornode_3pt *temp,*temp_next;
    temp = *head;
    temp_next = temp->next;
    free(temp);
    *head = temp_next;
}

void factory_insertNode(struct Factorynode **head,float c0,float c1,float c2, unsigned int timestamp)
{
	/*Create a new node and allocate the memory*/
    struct Factorynode *t;
    t = (struct Factorynode*)calloc(1,sizeof(struct Factorynode));

    /*Store the coefficients and epoch time stamp*/
    t->c0 = c0;
    t->c1 = c1;
    t->c2 = c2;
    t->timestamp = timestamp;
    /*Point to NULL address*/
    t->next = NULL;

    /*Check if the the linked list has no nodes*/
    if(*head == NULL)
    {
    	/*If no nodes are affable then assign the new node address to the head.*/
        *head = t;
    }
    /*If nodes are present*/
    else{
    	/*Make the copy of the linked list by assigning the head address to the temporary nodes*/
        struct Factorynode *temp;
        temp = *head;
        /*Traverse until it reaches the last node*/
        while(temp->next != NULL)
        {
            temp = temp->next;
        }
        /*Assign the address of the new node to the last node.*/
        temp->next = t;
    }
}

void factory_deleteNode(struct Factorynode **head)
{
    struct Factorynode *temp,*temp_next;
    temp = *head;
    temp_next = temp->next;
    free(temp);
    *head = temp_next;
}

void factory_dataTransfer(struct Factorynode **head,LastCalibrationFactoryHanlde_t *pLastCalibration_t,unsigned overflowFlag,uint8_t indexCount)
{
	struct Factorynode *temp;
	temp = *head;
	uint8_t index = 0;
	if(overflowFlag)
		index = 9;
	else
		index = indexCount - 1;
	while(temp->next != NULL)
	{
		/*Insert the data from Linked list to the Last Calibration buffer*/
		pLastCalibration_t->C0[index] = temp->c0;
		pLastCalibration_t->C1[index] = temp->c1;
		pLastCalibration_t->C2[index] = temp->c2;
		pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
		index -= 1; //Decrement the count
		temp = temp->next;
	}
    /*Insert the data from Linked list to the Last Calibration buffer*/
	pLastCalibration_t->C0[index] = temp->c0;
	pLastCalibration_t->C1[index] = temp->c1;
	pLastCalibration_t->C2[index] = temp->c2;
	pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
}

void sensor_dataTransfer(struct Sensornode **head,LastCalibrationSensorHandle_t *pLastCalibration_t,unsigned overflowFlag,uint8_t indexCount)
{
	struct Sensornode *temp;
	temp = *head;
	uint8_t index = 0;
	if(overflowFlag)
		index = 9;
	else
		index = indexCount - 1;
	while(temp->next != NULL)
	{
		/*Insert the data from Linked list to the Last Calibration buffer*/
		pLastCalibration_t->intercept[index] = temp->intercept;
		pLastCalibration_t->slope[index] = temp->slope;
		pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
		index -= 1; //Decrement the count
		temp = temp->next;
	}
    /*Insert the data from Linked list to the Last Calibration buffer*/
	pLastCalibration_t->intercept[index] = temp->intercept;
	pLastCalibration_t->slope[index] = temp->slope;
	pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
}

void sensor_3pt_dataTransfer(struct Sensornode_3pt **head,LastCalibrationSensorHandle_t *pLastCalibration_t,unsigned overflowFlag,uint8_t indexCount)
{
	struct Sensornode_3pt *temp;
	temp = *head;
	uint8_t index = 0;
	if(overflowFlag)
		index = 9;
	else
		index = indexCount - 1;
	while(temp->next != NULL)
	{
		/*Insert the data from Linked list to the Last Calibration buffer*/
		pLastCalibration_t->intercept[index] = temp->intercept;
		pLastCalibration_t->slope[index] = temp->slope;
		pLastCalibration_t->intercept_range_2[index] = temp->intercept_3pt;
		pLastCalibration_t->slope_range_2[index] = temp->slope_3pt;
		pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
		index -= 1; //Decrement the count
		temp = temp->next;
	}
    /*Insert the data from Linked list to the Last Calibration buffer*/
	pLastCalibration_t->intercept[index] = temp->intercept;
	pLastCalibration_t->slope[index] = temp->slope;
	pLastCalibration_t->intercept_range_2[index] = temp->intercept_3pt;
	pLastCalibration_t->slope_range_2[index] = temp->slope_3pt;
	pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
}

//void sensor_3pt_dataTransfer(struct Sensornode_3pt **head,LastCalibrationSensorHandle_t *pLastCalibration_t,unsigned overflowFlag,uint8_t indexCount)
//{
//	struct Sensornode_3pt *temp;
//	temp = *head;
//	uint8_t index = 0;
//	if(overflowFlag)
//		index = 9;
//	else
//		index = indexCount - 1;
//	while(temp->next != NULL)
//	{
//		/*Insert the data from Linked list to the Last Calibration buffer*/
//		pLastCalibration_t->intercept[index] = temp->intercept;
//		pLastCalibration_t->slope[index] = temp->slope;
//		pLastCalibration_t->intercept_range_2[index] = temp->intercept_3pt;
//		pLastCalibration_t->slope_range_2[index] = temp->slope_3pt;
//		pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
//		index -= 1; //Decrement the count
//		temp = temp->next;
//	}
//    /*Insert the data from Linked list to the Last Calibration buffer*/
//	pLastCalibration_t->intercept[index] = temp->intercept;
//	pLastCalibration_t->slope[index] = temp->slope;
//	pLastCalibration_t->intercept_range_2[index] = temp->intercept_3pt;
//	pLastCalibration_t->slope_range_2[index] = temp->slope_3pt;
//	pLastCalibration_t->epochtimestamp[index] = temp->timestamp;
//}
#if 0
void displayLinkedList(struct node **head)
{
	struct node *temp;
	temp = *head;
	while(temp->next != NULL)
	{
		printf("%f, ",temp->intercept);
		printf("%f, ",temp->slope);
		printf("%x\n",temp->timestamp);
		temp = temp->next;
	}
	printf("%f, ",temp->intercept);
	printf("%f, ",temp->slope);
	printf("%x",temp->timestamp);
}

void dataTransfer(struct node **head)
{
	struct node *temp;
	temp = *head;
	char count = 9;
	int i = 0;
	while(temp->next != NULL)
	{
		printf("%f, ",temp->intercept);
		printf("%f, ",temp->slope);
		printf("%x\n",temp->timestamp);
		/*Insert the data from Linked list to the Last Calibration buffer*/
		intercept[count] = temp->intercept;
		slope[count] = temp->slope;
		timestamp[count] = temp->timestamp;
		count -= 1; //Decrement the count
		temp = temp->next;
	}
	printf("%f, ",temp->intercept);
	printf("%f, ",temp->slope);
	printf("%x",temp->timestamp);
    /*Insert the data from Linked list to the Last Calibration buffer*/
		intercept[count] = temp->intercept;
		slope[count] = temp->slope;
		timestamp[count] = temp->timestamp;
    printf("\nRead from the buffer!!!\n");
    for(i = 0;i<=9;i++)
    {
        printf("%f, ",intercept[i]);
        printf("%f, ",slope[i]);
        printf("%x\n ",timestamp[i]);
    }
}
#endif

/*
 * Function name :- Application_LastCaldataToModbus
 *
 */
void Application_LastCaldataToModbus(void)
{
	uint8_t index = 0;
	uint32_t timestamp = HoldingRegister_t.ModeCommand_t.Epoch_Timestamp;
	HAL_Delay(20);
	if(AWADataStoreState.electronicpH)
	{
		index = AWALastCalibrationCount.electronicpH_count += 1;
		/*<TODO: Store intercept and slope in the Input register*/
		//InputRegister_t.PH_lastCalibration[index].PHCalibIntercept = pH_ElectronicCalibpoints_t.pH_Intercept;
		//InputRegister_t.PH_lastCalibration[index].PHCalibSlope = pH_ElectronicCalibpoints_t.pH_Slope;
	}
	if(AWADataStoreState.factoryCOD)
	{
		AWALastCalibrationCount.factoryCOD_count += 1;
		index = AWALastCalibrationCount.factoryCOD_count;
		/*If calibration data exceeds 10 pairs, set the overflow flag and reset the index count*/
		if(index >10)
		{
			AWALastCalibrationCount.factoryCOD_overflowflag = SET;/*Set the overflow flag*/
			AWALastCalibrationCount.factoryCOD_count = 0;/*Reset the count to 0*/
		}

		/*Create new node and insert at the end*/
		factory_insertNode(&CODFactoryhead,
				COD_10ptFactoryCalibrationHandle_t.c[0],
				COD_10ptFactoryCalibrationHandle_t.c[1],
				COD_10ptFactoryCalibrationHandle_t.c[2],
				timestamp);/*<TODO: Fetch the time stamp from Holding register*/

		/*If the overflow flag is set then delete the first node from the liked list.*/
		if(AWALastCalibrationCount.factoryCOD_overflowflag)
		{
			factory_deleteNode(&CODFactoryhead);
		}

		/*Write the into MODBUS*/
		factory_dataTransfer(&CODFactoryhead,
								&InputRegister_t.COD_lastCalibration,
								AWALastCalibrationCount.factoryCOD_overflowflag,
								AWALastCalibrationCount.factoryCOD_count);

		/*Store the overflow flag*/
		InputRegister_t.COD_lastCalibration.overflowFlag = AWALastCalibrationCount.factoryCOD_overflowflag;
	}
	if(AWADataStoreState.factoryTSS)
	{
		AWALastCalibrationCount.factoryTSS_count += 1;
		index = AWALastCalibrationCount.factoryTSS_count;
		/*If calibration data exceeds 10 pairs, set the overflow flag and reset the index count*/
		if(index >10)
		{
			AWALastCalibrationCount.factoryTSS_overflowflag = SET;/*Set the overflow flag*/
			AWALastCalibrationCount.factoryTSS_count = 0;/*Reset the count to 0*/
		}

		/*Create new node and insert at the end*/
		factory_insertNode(&TSSFactoryhead,
				TSS_10ptFactoryCalibrationHandle_t.c[0],
				TSS_10ptFactoryCalibrationHandle_t.c[1],
				TSS_10ptFactoryCalibrationHandle_t.c[2],
				timestamp);/*<TODO: Fetch the time stamp from Holding register*/

		/*If the overflow flag is set then delete the first node from the liked list.*/
		if(AWALastCalibrationCount.factoryTSS_overflowflag)
		{
			factory_deleteNode(&TSSFactoryhead);
		}

		/*Write the into MODBUS*/
		factory_dataTransfer(&TSSFactoryhead,
								&InputRegister_t.TSS_lastCalibration,
								AWALastCalibrationCount.factoryTSS_overflowflag,
								AWALastCalibrationCount.factoryTSS_count);

		/*Store the overflow flag*/
		InputRegister_t.TSS_lastCalibration.overflowFlag = AWALastCalibrationCount.factoryTSS_overflowflag;
	}
	if(AWADataStoreState.sensorCOD)
	{
		AWALastCalibrationCount.sensorCOD_count += 1;
		index = AWALastCalibrationCount.sensorCOD_count;
		/*If calibration data exceeds 10 pairs, set the overflow flag and reset the index count*/
		if(index >10)
		{
			AWALastCalibrationCount.sensorCOD_overflowflag = SET;/*Set the overflow flag*/
			AWALastCalibrationCount.sensorCOD_count = 0;/*Reset the count to 0*/
		}

		/*Create new node and insert at the end*/
		sensor_3pt_insertNode(&CODSensorhead,
							COD_SensorCalibration_t.intercept,
							COD_SensorCalibration_t.slope,
							COD_SensorCalibration_t.intercept_range_2,
							COD_SensorCalibration_t.slope_range_2,
							timestamp);/*<TODO: Fetch the time stamp from Holding register*/

		/*If the overflow flag is set then delete the first node from the liked list.*/
		if(AWALastCalibrationCount.sensorCOD_overflowflag)
		{
			sensor_3pt_deleteNode(&CODSensorhead);
		}

		/*Write the into MODBUS*/
		sensor_3pt_dataTransfer(&CODSensorhead,
							&InputRegister_t.COD_lastSensorCalibration,
							AWALastCalibrationCount.sensorCOD_overflowflag,
							AWALastCalibrationCount.sensorCOD_count);

		/*Store the overflow flag*/
		InputRegister_t.COD_lastSensorCalibration.overflowFlag = AWALastCalibrationCount.sensorCOD_overflowflag;
	}
	if(AWADataStoreState.sensorTSS)
	{
		AWALastCalibrationCount.sensorTSS_count += 1;
		index = AWALastCalibrationCount.sensorTSS_count;
		/*If calibration data exceeds 10 pairs, set the overflow flag and reset the index count*/
		if(index >10)
		{
			AWALastCalibrationCount.sensorTSS_overflowflag = SET;/*Set the overflow flag*/
			AWALastCalibrationCount.sensorTSS_count = 0;/*Reset the count to 0*/
		}

		/*Create new node and insert at the end*/
		sensor_3pt_insertNode(&TSSSensorhead,
							TSS_SensorCalibration_t.intercept,
							TSS_SensorCalibration_t.slope,
							TSS_SensorCalibration_t.intercept_range_2,
							TSS_SensorCalibration_t.slope_range_2,
							timestamp);/*<TODO: Fetch the time stamp from Holding register*/

		/*If the overflow flag is set then delete the first node from the liked list.*/
		if(AWALastCalibrationCount.sensorTSS_overflowflag)
		{
			sensor_3pt_deleteNode(&TSSSensorhead);
		}

		/*Write the into MODBUS*/
		sensor_3pt_dataTransfer(&TSSSensorhead,
							&InputRegister_t.TSS_lastSensorCalibration,
							AWALastCalibrationCount.sensorTSS_overflowflag,
							AWALastCalibrationCount.sensorTSS_count);

		/*Store the overflow flag*/
		InputRegister_t.TSS_lastSensorCalibration.overflowFlag = AWALastCalibrationCount.sensorTSS_overflowflag;
	}
	if(AWADataStoreState.sensorpH)
	{
		AWALastCalibrationCount.sensorpH_count += 1;
		index = AWALastCalibrationCount.sensorpH_count;
		/*If calibration data exceeds 10 pairs, set the overflow flag and reset the index count*/
		if(index >10)
		{
			AWALastCalibrationCount.sensorpH_overflowflag = SET;/*Set the overflow flag*/
			AWALastCalibrationCount.sensorpH_count = 0;/*Reset the count to 0*/
		}

		/*Create new node and insert at the end*/
//		sensor_insertNode(&pHSensorhead,
//							pH_SensorCalibpoints_t.pH_Intercept,
//							pH_SensorCalibpoints_t.pH_Solpe,
//							timestamp);

		sensor_3pt_insertNode(&pHSensorhead,
							pH_SensorCalibpoints_t.pH_Intercept,
							pH_SensorCalibpoints_t.pH_Solpe,
							pH_SensorCalibpoints_t.pH_Intercept_range_2,
							pH_SensorCalibpoints_t.pH_slope_range_2,
							timestamp);

		/*If the overflow flag is set then delete the first node from the liked list.*/
		if(AWALastCalibrationCount.sensorpH_overflowflag)
		{
			sensor_3pt_deleteNode(&pHSensorhead);
		}

		/*Write the into MODBUS*/
//		sensor_dataTransfer(&pHSensorhead,
//							&InputRegister_t.pH_lastSensorCalibration,
//							AWALastCalibrationCount.sensorpH_overflowflag,
//							AWALastCalibrationCount.sensorpH_count);

		sensor_3pt_dataTransfer(&pHSensorhead,
							&InputRegister_t.pH_lastSensorCalibration,
							AWALastCalibrationCount.sensorpH_overflowflag,
							AWALastCalibrationCount.sensorpH_count);

		/*Store the overflow flag*/
		InputRegister_t.pH_lastSensorCalibration.overflowFlag = AWALastCalibrationCount.sensorpH_overflowflag;
	}
}


void CODCalculateValue(void)
{
	//Quadratic equation - compensation (Factory Calibration)
	float c[4];//BOD

	//Calculations for COD raw
	float COD_RAW = HoldingRegister_t.ModeCommand_t.COD_SF *(log(COD_MeasurementValues_t.PD1_Zero/PD1_new) - log(COD_MeasurementValues_t.PD2_Zero/PD2_new));

	//COD coefficient
	for(int i = 0;i<4;i++)
		c[i] = HoldingRegister_t.ModeCommand_t.C[i];

	//COD_RAW = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];
	float factory_cod_value = pow(COD_RAW,3) *c[3] + pow(COD_RAW,2) *c[2] + pow(COD_RAW,1) *c[1] + c[0];

	//COD actual value
	COD_MeasurementValues_t.Cal_Value = factory_cod_value * COD_SensorCalibration_t.slope + COD_SensorCalibration_t.intercept;

	InputRegister_t.PV_info.COD_RAW = COD_RAW;
	if(COD_MeasurementValues_t.Cal_Value < 0)
	{
		InputRegister_t.PV_info.CODValue = 0.0f;	/*<Display to the Customer,only positive value*/
		//BOD value
		InputRegister_t.PV_info.BODValue = 0;		/*<Display to the Customer,only positive value*/
	}
	else
	{
		InputRegister_t.PV_info.CODValue = COD_MeasurementValues_t.Cal_Value;
		//BOD value
		InputRegister_t.PV_info.BODValue = HoldingRegister_t.ModeCommand_t.BOD_CF * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;
	}
	InputRegister_t.PV_info.CODValueUser = COD_MeasurementValues_t.Cal_Value; 	/*<Reference for the Developer, can be negative value*/
	InputRegister_t.PV_info.BODValueUser = HoldingRegister_t.ModeCommand_t.BOD_CF * COD_MeasurementValues_t.Cal_Value;



}

void TSScalculateValue(void)
{
	float k[3];//TSS
	//Calculation of TSS raw
	float TSS_RAW = HoldingRegister_t.ModeCommand_t.TSS_SF *(log(TSS_MeasurementValues_t.PD2_Zero/PD2_new));
	//TSS coefficient
	for(int i = 0;i<3;i++)
		k[i] = HoldingRegister_t.ModeCommand_t.TSS_K[i];
	//TSS factory equation
	float factory_tss_value = pow(TSS_RAW,2) *k[2] + pow(TSS_RAW,1) *k[1] + k[0];
	//TSS value
	TSS_MeasurementValues_t.Cal_Value = factory_tss_value * TSS_SensorCalibration_t.slope + TSS_SensorCalibration_t.intercept;

	//Publish the TSS value to the modbus
	InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
	if(TSS_MeasurementValues_t.Cal_Value < 0)
		InputRegister_t.PV_info.TSSValue = 0.0f;	/*<Display to the Customer,only positive value*/
	else
		InputRegister_t.PV_info.TSSValue = TSS_MeasurementValues_t.Cal_Value;
	InputRegister_t.PV_info.TSSValueUser = TSS_MeasurementValues_t.Cal_Value; 	/*<Reference for the Developer, can be negative value*/

}

void genericData(void)
{
	HoldingRegister_t.ModeCommand_t.CS_PUMP1_DELAY = 1;
	HoldingRegister_t.ModeCommand_t.CS_PUMP1_ONTIME=  20;
	HoldingRegister_t.ModeCommand_t.CS_PUMP2_DELAY = 1;
	HoldingRegister_t.ModeCommand_t.CS_PUMP2_ONTIME=  20;
	HoldingRegister_t.ModeCommand_t.CS_FLASH = 1;

	HoldingRegister_t.ModeCommand_t.PUMP1_DELAY = 1;
	HoldingRegister_t.ModeCommand_t.PUMP1_ONTIME = 10;
	HoldingRegister_t.ModeCommand_t.PUMP2_DELAY = 1;
	HoldingRegister_t.ModeCommand_t.PUMP2_ONTIME = 10;

	HoldingRegister_t.IOUTCalibandTest_t.temperatureSensorCable_resistance = 1.0f;

}

void AWA_RangeSelect(void)
{
	uint8_t AnalyzerRange = HoldingRegister_t.ModeCommand_t.RANGESELECT;

	switch(AnalyzerRange)
	{
	case MODEL_3021_3022:
		/*Load the default calibration and device configurations*/
		CalibrationDefaultValue(MODEL_3021_3022);
		/*Set the parameter display limit.*/
		COD_UpperLimit = (float)COD_800_UPPER;
		BOD_UpperLimit = COD_UpperLimit / 2.0f;
		TSS_UpperLimit = (float)TSS_750_UPPER;
		break;
	case MODEL_3011_3012:
		/*Load the default calibration and device configurations*/
		CalibrationDefaultValue(MODEL_3011_3012);
		/*Set the parameter display limit.*/
		COD_UpperLimit = (float)COD_300_UPPER;
		BOD_UpperLimit = COD_UpperLimit / 2.0f;
		TSS_UpperLimit = (float)TSS_450_UPPER;
		break;
	}

	//Set generic Data
	genericData();

	//Set the flags for the parameters to be reflected in calibration history
	AWADataStoreState.sensorCOD = SET;
	AWADataStoreState.sensorTSS = SET;
	AWADataStoreState.sensorpH = SET;
	AWADataStoreState.factoryCOD = SET;
	AWADataStoreState.factoryTSS = SET;

	//Store the Data into FRAM and reflect it in the Calibration history
	Application_LastCaldataToModbus();

	//Flag set as data not saved in the FRAM
	AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

	//Set the flag as range has been selected for the analyser
	AWADataStoreState.analyzerRangeSelect = SET;
}

void FlowSensorReadStatus(void)
{

	  float arrayADC [1000];
	  uint16_t samples = 100;
	  float avgs = (float)samples;

	  /*
	   * 500 samples - 1.7 ms
	   * with averaging - 2.7 ms
	   * for averaging - 1 ms
	   */
	  for(int i = 0;i<samples;i++)
	  {
		  //Set the data on the TOC HMI screen
		   arrayADC[i] = InternalADCRead() * (3.3f/4096.0f);
	  }
	  float avgADC = 0;
	  for(int i = 0;i<samples;i++)
		  avgADC += arrayADC[i] / avgs;

	  InputRegister_t.SlotParameter.FlowSensorVolatge = avgADC;
	  /*Cleaning tank is not empty*/
	  if(avgADC <= HoldingRegister_t.ModeCommand_t.FlowSensorCutoff) //Limit can by set from HMI.
	  {
		  AWAOperationStatus_t.CleaningTankEmpty = RESET;
		  CoilStatusRegister_t.CoilStatus_t.CleaningTankEmpty = RESET;
	  }
	  /* Cleaning tank is empty*/
	  else
	  {
		  AWAOperationStatus_t.CleaningTankEmpty = SET;
		  CoilStatusRegister_t.CoilStatus_t.CleaningTankEmpty = SET;/*Will display warning in HMI*/
	  }
}
void MILSwitchReadStatus(void)
{
	/*MIL switch*/
	uint32_t MIL_switch_status = HAL_GPIO_ReadPin(MIL_SWITCH_GPIO_Port, MIL_SWITCH_Pin);
	CoilStatusRegister_t.CoilStatus_t.MILSwitchState = MIL_switch_status;
	AWAOperationStatus_t.MILSwitchState = MIL_switch_status;
//	InputRegister_t.PV_info.TOC = MIL_switch_status;//Testing
}

void pumpOperationStop(void)
{
	/*TODO : Check which pump is in current operation*/
	uint8_t pumpNo = 0;
	uint8_t pumpAction = 0;

	pumpNo = PUMPControlHandle_t.u8PUMP_no;
	pumpAction = PUMPControlHandle_t.u8PUMP_action; // Running operation 3

	if(pumpAction == 3)
	{
		if(pumpNo == 2) //Sampling pump
		{
			HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
			//Turn of the pump 2
			PUMP2_OFF();
			//PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
			//Reset the HMI inter-locking Flag
			HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP,RESET);
			//Disable interrupt
//					htim6.Instance->DIER &= ~TIM_IT_UPDATE; // disable timer interrupt flag
			//htim6.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer
			//htim6.Instance->SR = 0;
			if(PUMPControlHandle_t.u8Flag_measurement == SET)
			{
				HAL_TIM_Base_Stop_IT(&htim6);
				HAL_TIM_Base_DeInit(&htim6);

				PUMPControlHandle_t.u8Flag_measurement = RESET;
			}
			//reset all the action
			PUMPControlHandle_t.u16TIM6_count = 0;
			PUMPControlHandle_t.u16TIM6_limit = 0;
			PUMPControlHandle_t.u8PUMP_action = 1;
			PUMPControlHandle_t.u8PUMP_delayaction = 0;
			PUMPControlHandle_t.u8PUMP_no = 0;
			//un-block the ADC measurement
			PUMPControlHandle_t.u8Flag_measurement = 0x00;

			HoldingRegister_t.ModeCommand_t.CommonCommandHMI = 0x0;
		}
		if(pumpNo == 1)
		{
			HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
			//Valve off
			VALVE_OFF();
			//Turn of the pump 1
			PUMP1_OFF();
			//PUMPControlHandle_t.u8PUMP_action = 0x02;//move to the pump delay time
			//Reset the HMI inter-locking Flag
			HMIInterlockingFlag(HMI_INTERLOCK_SAMPLE_PUMP,RESET);
			//Disable interrupt
//					htim6.Instance->DIER &= ~TIM_IT_UPDATE; // disable timer interrupt flag
			//htim6.Instance->CR1 &= ~TIM_CR1_CEN; // disable timer
			//htim6.Instance->SR = 0;
			if(PUMPControlHandle_t.u8Flag_measurement == SET)
			{
				HAL_TIM_Base_Stop_IT(&htim6);
				HAL_TIM_Base_DeInit(&htim6);

				PUMPControlHandle_t.u8Flag_measurement = RESET;
			}
			//reset all the action
			PUMPControlHandle_t.u16TIM6_count = 0;
			PUMPControlHandle_t.u16TIM6_limit = 0;
			PUMPControlHandle_t.u8PUMP_action = 1;
			PUMPControlHandle_t.u8PUMP_delayaction = 0;
			PUMPControlHandle_t.u8PUMP_no = 0;
			//un-block the ADC measurement
			PUMPControlHandle_t.u8Flag_measurement = 0x00;

			HoldingRegister_t.ModeCommand_t.CommonCommandHMI = 0x0;
		}
	}else
	{
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = 0x0;
	}

	//HMI bits for MIMIC
	CoilStatusRegister_t.CoilStatus_t.measure = RESET;
	CoilStatusRegister_t.CoilStatus_t.read_acid = RESET;
	CoilStatusRegister_t.CoilStatus_t.read_sample = RESET;
	CoilStatusRegister_t.CoilStatus_t.NoProcess = SET;
}

void Application_commandCheckandProcess(void)
{
	  /*Compare the command from HMI with the already executing command*/
	  if(HoldingRegister_t.ModeCommand_t.CommonCommand == RESET
			  && CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG == SET)
	  {
		  /*
		   * If the the already executing command has expired.
		   * Then, push the command from HMI into the state machine command variable.
		   */

		  //Main command
		  HoldingRegister_t.ModeCommand_t.ModeCommand_H = HoldingRegister_t.ModeCommand_t.ModeCommandHMI_H;
		  HoldingRegister_t.ModeCommand_t.ModeCommand_L = HoldingRegister_t.ModeCommand_t.ModeCommandHMI_L;

		  //Common command
		  HoldingRegister_t.ModeCommand_t.CommonCommand = HoldingRegister_t.ModeCommand_t.CommonCommandHMI;
		  //HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

		  //Reset the new command flag
		  CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG = RESET;
	  }
	  /*For cleaning cycle*/
	  else if(HoldingRegister_t.ModeCommand_t.CommonCommandHMI == AUTO_COD_ZERO
	  			  && CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG == SET)
	  {
		  /*Perform Auto Zero after the current action is over*/
		  performAUTOZERO = SET;	//Action will be performed in main loop

		  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

		  //Reset the new command flag
		  CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG = RESET;

	  }
	  /*For batch cycle*/
	  else if(HoldingRegister_t.ModeCommand_t.CommonCommandHMI == AUTO_COD_MEASURE
	  			  && CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG == SET)
	  {
		  /*Perform batch mode after the current action is over*/
		  performBatchCycle = SET;	//Action will be performed in main loop

		  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

		  //Reset the new command flag
		  CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG = RESET;

	  }
	  /*For STOP pump action*/
	  else if(HoldingRegister_t.ModeCommand_t.CommonCommandHMI == STOP_RUNNING_PUMP
			  && HoldingRegister_t.ModeCommand_t.CommonCommand != COD_Measure
			  && CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG == SET)
	  {
		  //Reset the new command flag
		  //CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG = RESET;

		  //Main command
		  HoldingRegister_t.ModeCommand_t.ModeCommand_H = HoldingRegister_t.ModeCommand_t.ModeCommandHMI_H;
		  HoldingRegister_t.ModeCommand_t.ModeCommand_L = HoldingRegister_t.ModeCommand_t.ModeCommandHMI_L;

		  //Common command
		  HoldingRegister_t.ModeCommand_t.CommonCommand = HoldingRegister_t.ModeCommand_t.CommonCommandHMI;

		  //Reset the new command flag
		  CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG = RESET;
	  }
	  /*Check if the command from HMI is similar to that of the already executing/executed command*/
	  else if(HoldingRegister_t.ModeCommand_t.CommonCommand != RESET
			  && CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG == SET)
	  {
		  /*
		   * If there is a new command from the HMI while the old common command is still in use,
		   * then the common command from the HMI should be forced RESET, but the main command (Page ID)
		   * should be retained.
		   */

		  //Reset the common command from HMI
		  HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;

		  //Reset the new command flag
		  CoilStatusRegister_t.CoilStatus_t.NEW_COMMAND_FLAG = RESET;
	  }

	  if(performAUTOZERO && HoldingRegister_t.ModeCommand_t.CommonCommand == RESET)
	  {
		  //Set the main page command
		  HoldingRegister_t.ModeCommand_t.ModeCommand_H = 0x22;
		  HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x01;

		  //Set the common command for Auto Zeroing
		  HoldingRegister_t.ModeCommand_t.CommonCommand = AUTO_COD_ZERO;

		  //Reset the Auto Zero command flag.
		  performAUTOZERO = RESET;

	  }else if(performBatchCycle && HoldingRegister_t.ModeCommand_t.CommonCommand == RESET)
	  {
		  //Set the main page command
		  HoldingRegister_t.ModeCommand_t.ModeCommand_H = 0x22;
		  HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x01;

		  //Set the common command for batch mode
		  HoldingRegister_t.ModeCommand_t.CommonCommand = AUTO_COD_MEASURE;

		  //Reset the Auto Zero command flag.
		  performBatchCycle = RESET;
	  }
}


void Application_AWAIOProcess(void)
{
	  if(AWAOperationStatus_t.FactoryMode != 1 && AWAOperationStatus_t.CalibrationMode != 1 && AWAOperationStatus_t.CODFlashOperation != 1)
	  {
		  CardAction(0x01);
		  if(AWAOperationStatus_t.ElectronicCal_AO == 0)
		  {
			ParameterValues_t.pH_Value = InputRegister_t.PV_info.pH_value;
			ParameterValues_t.COD_Value = InputRegister_t.PV_info.CODValue;
			ParameterValues_t.BOD_Value = InputRegister_t.PV_info.BODValue;
			ParameterValues_t.TSS_Value = InputRegister_t.PV_info.TSSValue;

			//Analog Output
			ParameterIOutProcess();

			//Relay Output
			ParameterRelayAlarmProcess();
		  }

	  }
}


void Application_MODBUSRXprocess(void)
{
	/****TEST***/
	if(uart_rx_process_query)
	{
		uart_rx_process_query = RESET;

		HAL_TIM_Base_Stop_IT(&htim7);
		/*3. if the Slave ID is not correct the flush the RX buffer*/
		memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		//Reset the rx byte counter
		uart_rx_bytes = 0;
		//Reset the query processing flag.
		uart_rx_process_query = RESET;
		//re-start the uart reception interrupt
		HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);
		HAL_TIM_Base_Start_IT(&htim7);
	}
	/*******/
	  //RTU
	/***************DEPRICATED************************/
	  if(DMA_TX_FLAG == 1)
	  {
#if MODBUS_MULTI_DROP
		HAL_TIM_Base_Stop_IT(&htim7);
		/*3. if the Slave ID is not correct the flush the RX buffer*/
		//memset(MOD2_Rxbuff,'\0',sizeof(MOD2_Rxbuff));
		memset((uint16_t*)Rxbuff,'\0',sizeof(Rxbuff));
		//Reset the rx byte counter
		uart_rx_bytes = 0;
		//Reset the query processing flag.
		uart_rx_process_query = RESET;
		//re-start the uart reception interrupt

		ADM_2_CLTR_LOW();
		HAL_UART_Receive_IT(&huart6, &uart_rx_buffer, 1);
		HAL_TIM_Base_Start_IT(&htim7);

		//ADM_2_CLTR_LOW();
		DMA_TX_FLAG = 0;
#else
		/*Clear the UART6 Transfer complete flag*/
		while(!(huart6.Instance->SR & USART_SR_TC));
		huart6.Instance->SR &= !USART_SR_TC;

		/*Disable the UART DMA Tx*/
		huart6.Instance->CR3 &= !USART_CR3_DMAT;

		/*Enable the UART DMA Rx*/
		huart6.Instance->CR3 |= USART_CR3_DMAR;

		/*Enable the Stream 2 IRQ*/
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

		/*Enable the Stream 2*/
		DMAPeripheralEnable(DMA2_Stream2,ENABLE);
		ADM_2_CLTR_LOW();
		DMA_TX_FLAG = 0;
#endif
	  }
	  /**************************************************************************/
	  //HMI
	  if(DMA_TX_FLAG_HMI == 1)
	  {
		/*Clear the UART1 Transfer complete flag*/
		while(!(huart1.Instance->SR & USART_SR_TC));
		huart1.Instance->SR &= !USART_SR_TC;

		/*Disable the UART DMA Tx*/
		huart1.Instance->CR3 &= !USART_CR3_DMAT;

		/*Enable the UART DMA Rx*/
		huart1.Instance->CR3 |= USART_CR3_DMAR;

		/*Configure the Stream 5 Rx with DMA_FIRST_TRANSACTION_NO*/
#if QUERRY_RX_INIT_LEN_6
		DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[0]),DMA_FIRST_TRANSACTION_NO);
#else
		DMA_UART1_RX_Init((uint32_t*)(&Rxbuff[0]),8);
#endif

		/*Enable the Stream 5 IRQ*/
		//HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

		/*Enable the Stream 5*/
		DMAPeripheralEnable(DMA2_Stream5,ENABLE);

		ADM_CLTR_LOW();
		DMA_TX_FLAG_HMI = 0;
	  }
}


void Application_StoreData(void)
{
	  //Check for the save calibration data flag, if the flag is set then save the calibration data into FRAM
	  if(AWAOperationStatus_t.AWADataSave_Calibration)
	  {
		  ModbusSaveConfiguration(SAVE_CALIBRATION_DATA);
	  }

	  //Check for the save process parameter data flag, if the flag is set then save the calibration data into FRAM
	  if(AWAOperationStatus_t.AWADataSave_ProcessValues)
	  {
		  ModbusSaveConfiguration(SAVE_PROCESSPARAMETER_DATA);
	  }
}

void Application_MODBUSParseQuery(void)
{
	  if(RxFlag == 0x01)
	  {
		  RxFlag = 0;
//		  ProcessModbusQuery();
		  ProcessMOD2_ModbusQuery_DMA();//RTU
	  }

	  //Comparing current command and new command
	  Application_commandCheckandProcess();

	  if(MOD2_RxFlag == 0x01)
	  {
		  //Copy the data from Analyzer to RTU buffer
		  //setRTUData();

		  MOD2_RxFlag = 0;
		  //ProcessMOD2_ModbusQuery();
//		  ProcessMOD2_ModbusQuery_DMA();

		  ProcessModbusQuery_ISR();//HMI
	  }
	  if(MOD3_RxFlag)
	  {
		  MOD3_RxFlag = RESET;
		  ProcessModbusQuery_ISR_3();//CPCB
	  }
}
uint32_t adjust_time_zone_to_epoch(int tz, uint32_t epoch){
    return epoch - ((5 * 60 * 60) + (30 * 60));
}

uint32_t adjust_time_zone_to_epoch_add(int tz, uint32_t epoch){
    return epoch + ((5 * 60 * 60) + (30 * 60));
}

uint32_t date_time_to_epoch(date_time_t* date_time)
{

    unsigned int second = date_time->second;  // 0-59
    unsigned int minute = date_time->minute;  // 0-59
    unsigned int hour   = date_time->hour;    // 0-23
    unsigned int day    = date_time->day-1;   // 0-30
    unsigned int month  = date_time->month-1; // 0-11
    unsigned int year   = date_time->year;    // 0-99
    /*
    unsigned int second = in_sec;  // 0-59
    unsigned int minute = in_min;  // 0-59
    unsigned int hour   = in_hour;    // 0-23
    unsigned int day    = in_day;   // 0-30
    unsigned int month  = in_month; // 0-11
    unsigned int year   = in_year;    // 0-99
*/
    uint32_t epoch = 946684800 + (((year/4*(365*4+1)+days[year%4][month]+day)*24+hour)*60+minute)*60+second;
    epoch = adjust_time_zone_to_epoch(22,epoch) ;
    return epoch ;
}

void epoch_to_date_time(date_time_t* date_time, uint32_t epoch)
{
    epoch = epoch - 946684800 ;
    epoch = adjust_time_zone_to_epoch_add(22,epoch);
    date_time->second = epoch%60; epoch /= 60;
    date_time->minute = epoch%60; epoch /= 60;
    date_time->hour   = epoch%24; epoch /= 24;
    uint32_t years = epoch/(365*4+1)*4; epoch %= 365*4+1;
    uint32_t year;
    for (year=3; year>0; year--)
    {
        if (epoch >= days[year][0])
            break;
    }
    uint32_t month;
    for (month=11; month>0; month--)
    {
        if (epoch >= days[year][month])
            break;
    }
    date_time->year  = years+year;
    date_time->month = month+1;
    date_time->day   = epoch-days[year][month]+1;
}


void calculate_Next_ProcessTime(void)
{
	uint32_t epoch = 0;
	epoch = HoldingRegister_t.SensorCalibration_t.LastBatchTime;
	//Next process time
	int next_batch_hr = HoldingRegister_t.SensorCalibration_t.NextProcessTime_Hr;
	int next_batch_min = HoldingRegister_t.SensorCalibration_t.NextProcessTime_Min;
	int next_batch_sec = HoldingRegister_t.SensorCalibration_t.NextProcessTime_Sec;

	next_batch_hr *= 3600;
	next_batch_min *= 60;
	epoch += (next_batch_hr + next_batch_min + next_batch_sec);

	HoldingRegister_t.SensorCalibration_t.NextProcessTime = epoch;
}

void startupMessages(void)
{
	HoldingRegister_t.SensorCalibration_t.pHSensMessages = SIMULATE_POS_414_mV;
	HoldingRegister_t.SensorCalibration_t.TempElectronicCalibMessages = SIMULATE_100_OHMS;
	HoldingRegister_t.SensorCalibration_t.COD_Messages = ENTER_BUFFER_SOL_1;
	HoldingRegister_t.SensorCalibration_t.TSS_Messages = ENTER_BUFFER_SOL_1;
}
//DEPRICATED
void resetMODBUS(void)
{
//	  UHolding_Modbus_3.confirm = RESET;
//
//	  HAL_UART_Abort_IT(&huart3);
//	  HAL_TIM_Base_Stop_IT(&htim12);
//	  UHolding_Modbus_3.baudrate = 9610;
//
//	  HAL_UART_DeInit(&huart3);
//	  HAL_UART_IRQHandler(&huart3);
}

void resetMODBUSComm(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
	  HAL_UART_Abort_IT(huart);
	  HAL_TIM_Base_Stop_IT(htim);
	  //UHolding_Modbus_3.baudrate = 9610;

	  HAL_UART_DeInit(huart);
	  HAL_UART_IRQHandler(huart);
}

//DEPRICATED
void beginMODBUS(void)
{

	  uart_rx_timeout_counter_uart3 = 0;
	  //Reset the rx byte counter
	  uart_rx_bytes_uart3 = 0;
	  MOD3_RxFlag = RESET;
	  ADM_3_CLTR_LOW();
	  HAL_TIM_Base_Start_IT(&htim12);
}
void beginMODBUSComm(TIM_HandleTypeDef *htim,volatile uint16_t *timeour_counter,volatile uint8_t *rx_bytes,volatile uint8_t *querry_process,int dir_line)
{
	//Reset the timer timeout counter
	*timeour_counter = 0;
	//Reset the rx byte counter
	*rx_bytes = 0;
	*querry_process = RESET;
	//ADM_3_CLTR_LOW();
	if(dir_line == 3)
		ADM_3_CLTR_LOW();
	if(dir_line == 2)
		ADM_CLTR_LOW();
	//Start the MODBUS timer
	HAL_TIM_Base_Start_IT(htim);
}
void MODBUSConfiguration(UART_HandleTypeDef *huart,MODBUS_config_t pConfig)
{
	/*
	 * UART configuration
	 */


	  huart->Init.BaudRate = pConfig.baudrate;
	  huart->Init.WordLength = ( pConfig.data_length == 1 ) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
	  huart->Init.StopBits = ( pConfig.stop_bits == 1 ) ? UART_STOPBITS_1 : UART_STOPBITS_2;
	  huart->Init.Parity = ( pConfig.parity_bit == 0 ) ? UART_PARITY_NONE : ( pConfig.parity_bit == 1 ) ? UART_PARITY_EVEN : UART_PARITY_ODD;

	  if (HAL_UART_Init(huart) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void baudrateSelection(MODBUS_config_t *PORT)
{
	(PORT->baudrate_selection == 1)?PORT->baudrate = 9600:
			(PORT->baudrate_selection == 2)?PORT->baudrate = 19200:
					(PORT->baudrate_selection == 3)?PORT->baudrate = 38400:
							(PORT->baudrate_selection == 4)?PORT->baudrate = 115200:0;
}

void initqueue(void)
{
	queue_ = (struct queue *)malloc(sizeof(struct queue));
	queue_->maxSize = 5;
	queue_->data = (int *)malloc(queue_->maxSize * sizeof(int)); // allocating the memory for the pointer to store the data
	queue_->front = -1;
	queue_->rear = -1;
	//printf("Init!!!\n");
}

void enqueue(struct queue *queue_,int da)
{

	if(queue_->front == -1){
		++(queue_->front);
	}
	if(queue_->rear == queue_->maxSize - 1)
	{
		//printf("Queue is full!!");
	}else{
		++(queue_->rear);
		//printf("%d\n",queue_->rear);
		int in = queue_->rear;
		queue_->data[in] = da;
		//printf("%d Enqueued!!\n",da);
		//printf("front :%d\n",queue_->front);
		//printf("rear :%d\n",queue_->rear);
	}

}

void dequeue(void)
{
	if(queue_->rear == -1)
	{
		//printf("Queue is Empty!!");
	}else if(queue_->front == queue_->maxSize - 1){
		queue_->front = -1;
		queue_->rear = -1;
		//printf("for last element front :%d\n",queue_->front);
		//printf("for last element rear :%d\n",queue_->rear);
	}else{
		++(queue_->front);
		//printf("Deququed!!\n");
		//printf("front :%d\n",queue_->front);
		//printf("rear :%d\n",queue_->rear);
	}
}

