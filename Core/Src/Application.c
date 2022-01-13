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
extern UART_HandleTypeDef huart3;
uint8_t cod_flash_operation;
uint16_t dataptr1;
uint16_t dataptr;
uint16_t ArrayPtr;


void ProcessModesCommands(void)
{
	if(HoldingRegister_t.ModeCommand_t.ModeCommand_H == RUN_MODE)
	{
		switch(HoldingRegister_t.ModeCommand_t.ModeCommand_L)
		{
			//Will change the commands, this just for testing
			case Batch:
			{
				AWAOperationStatus_t.ElectronicCal_AO = 0;
				AWAOperationStatus_t.CalibrationMode = 0;
				AWAOperationStatus_t.FactoryMode = 0;
				//Auto measurement, background process
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == AUTO_COD_MEASURE)
				{
					//First operate the sample pump
					PumpOperation(0x02);

					//perform the ADC measurement action when the pump action is completed
					if(!PUMPControlHandle_t.u8Flag_measurement)
						HoldingRegister_t.ModeCommand_t.CommonCommand = COD_Measure;
				}
				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CODADCCapture(COD_Measure);
				}
				//Perform the Auto Zero process
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == AUTO_COD_ZERO)
				{
					//First operate the sample pump
					PumpOperation(0x01);

					//perform the ADC measurement action when the pump action is completed
					if(!PUMPControlHandle_t.u8Flag_measurement)
						HoldingRegister_t.ModeCommand_t.CommonCommand = COD_ZERO_MEAS;
				}
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_ZERO_MEAS && !PUMPControlHandle_t.u8Flag_measurement)
				{
					CODADCCapture(AUTO_COD_ZERO);
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
				HoldingRegister_t.ModeCommand_t.ModeCommand_H = 0x23;
				HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x10;
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
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
					AWAOperationStatus_t.CalibrationMode = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
					HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x50;
				}
				break;
			}
			case Calibrate_COD:
			{
				//Measure Zero
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt1)
					COD_SensorCalib_ypoint(1);
				//Measure Sample 1
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt2)
					COD_SensorCalib_ypoint(2);
				//Measure sample 2
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt3)
					COD_SensorCalib_ypoint(3);

				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					//CODADCCapture(COD_Measure);
					CODADCCapture_Sensor(COD_Measure,point);
#endif
				}
				//For Two point calibration operation
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Sensor_Calibrate_COD)
				{
					COD_SensorCalibration();
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}
				break;
			}
			case Calibrate_TSS:
			{
				//Measure Zero
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt1)
					COD_SensorCalib_ypoint(4);
				//Measure Sample 1
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt2)
					COD_SensorCalib_ypoint(5);
				//Measure sample 2
				else if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_SENSOR_MEASURE_pt3)
					COD_SensorCalib_ypoint(3);

				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					//CODADCCapture(COD_Measure);
					CODADCCapture_Sensor(COD_Measure,point);
#endif
				}
				//For Two point calibration operation
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == Sensor_Calibrate_TSS)
				{
					TSS_SensorCalibration();
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
				}
				break;
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
			case Electronics_Calibration_mV: //page
			{
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
				pH_ElectronicCalibpoints_t.pH_ADCCounts = ADS1115_OperationpHMeasurement(0x01);//(float)adc_temp / (float)sample_no;
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
					AWAOperationStatus_t.FactoryMode = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
					HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x50;
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
					break;
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
					/*18-8-2021*/
					HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
					//calFlag = 0;/*30/8/2021*/
					AWAOperationStatus_t.ElectronicCal_AO = 0x0;
					//New calibration data is not saved
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
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
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
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
					AWAOperationStatus_t.FactoryMode = 0x0;
					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;
					HoldingRegister_t.ModeCommand_t.ModeCommand_L = 0x50;
					HoldingRegister_t.ModeCommand_t.CommonCommand = 0x0;
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

					gaussEliminationLS(x_mat,y_mat,10,3,1);

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}

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
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == COD_FACTORY_SETASZERO)
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

					//Flag set as data not saved in the FRAM
					AWAOperationStatus_t.AWADataSave_Calibration = 0x01;

					HoldingRegister_t.ModeCommand_t.CommonCommand = 0;
				}

				//perform the COD ADC measurement action if command is received and none of the pump actions are taking place.
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_Measure && !PUMPControlHandle_t.u8Flag_measurement)
				{
#ifdef HMI_TEST
					CODADCCapture(AUTO_COD_ZERO);
#else
					CODADCCapture(TSS_Measure);
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
				if(HoldingRegister_t.ModeCommand_t.CommonCommand == TSS_FACTORY_SETASZERO)
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
				break;
			}
			case Relay_test:
			{
				RelayToggle();
				//RelayToggleCoilInputUpdate();
			}
		}
	}
}

void CardAction(uint8_t CardID)
{

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
				//pH value with temperature compensation
				InputRegister_t.PV_info.pH_value = (adc * pH_SensorCalibpoints_t.pH_Solpe * (1.0 / (1.0 + 0.003351686 * (InputRegister_t.PV_info.temp_pH - 25))) + pH_SensorCalibpoints_t.pH_Intercept);

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
		//Software PT100 select SEL1 pin HIGH
		if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)//PT100
			SEL1_PT100();
		//Software PT1000 select SEL1 pin LOW
		else
			SEL1_PT1000();

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
			adc = AdCounts_pH * PT_ElectronicCalibration_t.PT100_slope + PT_ElectronicCalibration_t.PT100_intercept;
			volatge = adc;
			adc *= 10.0f;

		}
		//If PT-1000 is selected
		else if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT1000)
		{
			adc = AdCounts_pH * PT_ElectronicCalibration_t.PT1000_slope + PT_ElectronicCalibration_t.PT1000_intercept;
			volatge = adc;
		}

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

				//COD value with Factory Calibration not sensor
				//COD_MeasurementValues_t.Cal_Value = factory_cod_value;

				//TSS value
				TSS_MeasurementValues_t.Cal_Value = factory_tss_value;

				//BOD value
				InputRegister_t.PV_info.BODValue = 0.5f * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;

				//TOC Value
				InputRegister_t.PV_info.TOC = 0.3f * COD_MeasurementValues_t.Cal_Value;//factory_cod_value;

				//Publish the COD_RAW on modbus
				//InputRegister_t.PV_info.COD_RAW = COD_MeasurementValues_t.RAW_Value;
				InputRegister_t.PV_info.COD_RAW = COD_RAW;
				InputRegister_t.PV_info.CODValue = COD_MeasurementValues_t.Cal_Value;

				//Publish the TSS value to the modbus
				InputRegister_t.PV_info.TSS_RAW = TSS_RAW;
				InputRegister_t.PV_info.TSSValue = TSS_MeasurementValues_t.Cal_Value;

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
			huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
			huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			//reset the common command flag
			HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;

			//set the flashing operation flag to 2 as flashing operation is completed successfully
			cod_flash_operation = 2;

			//Reset the COD flashing operation to perform other card action
			AWAOperationStatus_t.CODFlashOperation = 0x00;

			//Reset the HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_MEASURE,RESET);
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
			HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_MEASURE,SET);
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
			HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;

			//set the flashing operation flag to 2 as flashing operation is completed successfully
			cod_flash_operation = 2;

			//Reset the COD flashing operation to perform other card action
			AWAOperationStatus_t.CODFlashOperation = 0x00;

			//Reset the HMI inter-locking flag
			HMIInterlockingFlag(HMI_INTERLOCK_SENSOR_MEASURE,RESET);
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
			CoilStatusRegister_t.CoilStatus_t.acid_pump = state;
			break;
		case HMI_INTERLOCK_SAMPLE_PUMP:
			CoilStatusRegister_t.CoilStatus_t.sample_pump = state;
			break;
		case HMI_INTERLOCK_MEASURE:
			CoilStatusRegister_t.CoilStatus_t.measure = state;
			break;
	}

}

void RelayToggle(void)
{

#if 1
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
#if 0
	if(HoldingRegister_t.ModeCommand_t.relay[0] == 1)
	{
		RELAY_1_ON();
	}else{
		RELAY_1_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[1] == 1)
	{
		RELAY_2_ON();
	}else{
		RELAY_2_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[2] == 1)
	{
		RELAY_3_ON();
	}else{
		RELAY_3_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[3] == 1)
	{
		RELAY_4_ON();
	}else{
		RELAY_4_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[4] == 1)
	{
		RELAY_5_ON();
	}else{
		RELAY_5_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[5] == 1)
	{
		RELAY_6_ON();
	}else{
		RELAY_6_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[6] == 1)
	{
		RELAY_7_ON();
	}else{
		RELAY_7_OFF();
	}
	if(HoldingRegister_t.ModeCommand_t.relay[7] == 1)
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
			HAL_Delay(1000);
			//turn on pump 1
			PUMP1_ON();
			//Set the HMI inter-locking flag for sample pump
			HMIInterlockingFlag(HMI_INTERLOCK_ACID_PUMP, SET);
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
		}
		//Enable Interrupt
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
	}
}

void currentOutputTest(uint8_t CurrentChannel)
{

	uint32_t PWM_count= (CurrentTestValues_mA[HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Test_Command - 1] - CurrentOutputCalibration_t[CurrentChannel-1].AO_intercept)/CurrentOutputCalibration_t[CurrentChannel-1].AO_slope;
	PWMCurrentCalibOutput(CurrentChannel,PWM_count);
}

void ModbusSaveConfiguration(uint8_t data)
{
	  HAL_Delay(10);

	  if(data == SAVE_CONFIGURATION_DATA)//Configuration data
	  {
		  //Complete MODBUS data storage
		  FRAM_OperationWrite(FRAM_ADDRESS_MIN, (uint8_t*)&HoldingRegister_t.bytes, sizeof(HoldingRegister_t.bytes));
	  }
	  else if(data == SAVE_CALIBRATION_DATA)//Calibration data storage
	  {
		  //AO Electronic calibration data store
	//	  FRAM_OperationWrite(FRAM_ADDRESS_MIN + sizeof(HoldingRegister_t.bytes),(uint8_t*)&CurrentOutputCalibration_t,sizeof(CurrentOutputCalibration_t));
		  FRAM_OperationWrite(FRAM_ADDRESS_AO_ELEC_CALIB,(uint8_t*)&CurrentOutputCalibration_t,sizeof(CurrentOutputCalibration_t) * 8);

		  //pH Electronic calibration storage
		  FRAM_OperationWrite(FRAM_ADDRESS_pH_ELEC_CALIB,(uint8_t*)&pH_ElectronicCalibpoints_t.bytes[8],8);

		  //PH Sensor calibration store
		  FRAM_OperationWrite(FRAM_ADDRESS_pH_SENS_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,8);

		  //PT temperature sensor electronic calibration storage (PT-100 AND PT-1000)
		  FRAM_OperationWrite(FRAM_ADDRESS_PT_ELEC_CALIB,(uint8_t*)&PT_ElectronicCalibration_t.bytes,16);

		  //COD Sensor calibration data store
		  FRAM_OperationWrite(FRAM_ADDRESS_COD_SENS_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,8);

		  //COD Factory 10pt calibration data store
		  FRAM_OperationWrite(FRAM_ADDRESS_COD_FACTORY_CALIB,(uint8_t*)&COD_10ptFactoryCalibrationHandle_t.bytes,96); //Yet to save the COD SF, total bytes 100

		  framdata.fData[0] = InputRegister_t.PV_info.PD1_0;
		  framdata.fData[1] = InputRegister_t.PV_info.PD2_0;
		  //COD PD1(0) and PD2(0)
		  FRAM_OperationWrite(FRAM_ADDRESS_COD_PD_ZERO,(uint8_t*)&framdata.bytes,8);

		  //TSS Factory 10pt calibration data store
		  FRAM_OperationWrite(FRAM_ADDRESS_TSS_FACTORY_CALIB,(uint8_t*)&TSS_10ptFactoryCalibrationHandle_t.bytes,96); //Yet to save the COD SF, total bytes 100

		  memset(&framdata,'\0',sizeof(framdata));

		  framdata.fData[0] = InputRegister_t.PV_info.TSS_PD2_0;
		  //TSS PD2(0)
		  FRAM_OperationWrite(FRAM_ADDRESS_TSS_PD_ZERO,(uint8_t*)&framdata.bytes,4);

		  //TSS Sensor calibration data store
		  FRAM_OperationWrite(FRAM_ADDRESS_TSS_SENS_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,8);

		  //Reset the FRAM save Calibration flag
		  AWAOperationStatus_t.AWADataSave_Calibration = 0x00;
	  }


}

void ModbusReadConfiguration(void)
{
	//Read the complete MODBUS configuration from FRAM
	FRAM_OperationRead(FRAM_ADDRESS_MIN, (uint8_t*)&HoldingRegister_t.bytes, sizeof(HoldingRegister_t.bytes));

	//Read the AO electronic calibration data
	FRAM_OperationRead(FRAM_ADDRESS_AO_ELEC_CALIB, (uint8_t*)&CurrentOutputCalibration_t, sizeof(CurrentOutputCalibration_t));

	//Read the pH electronic calibration data
	FRAM_OperationRead(FRAM_ADDRESS_pH_ELEC_CALIB, (uint8_t*)&pH_ElectronicCalibpoints_t.bytes[8], 8);

	//Read the pH sensor calibration data
	FRAM_OperationRead(FRAM_ADDRESS_pH_SENS_CALIB,(uint8_t*)&pH_SensorCalibpoints_t.byte,8);

	//Read the PT-100 and PT-1000 Electronic calibration data
	FRAM_OperationRead(FRAM_ADDRESS_PT_ELEC_CALIB,(uint8_t*)&PT_ElectronicCalibration_t.bytes,16);

	//Read the COD Electronic Calibration Data
	FRAM_OperationRead(FRAM_ADDRESS_COD_SENS_CALIB,(uint8_t*)&COD_SensorCalibration_t.byte,8);
	//Publish to modbus
	HoldingRegister_t.SensorCalibration_t.COD_CF = COD_SensorCalibration_t.slope;
	HoldingRegister_t.SensorCalibration_t.COD_Intercept = COD_SensorCalibration_t.intercept;

	//Read the COD Factory 10-pt Calibration Data
	FRAM_OperationRead(FRAM_ADDRESS_COD_FACTORY_CALIB,(uint8_t*)&COD_10ptFactoryCalibrationHandle_t.bytes,96);

	for(int i = 0;i<10;i++)
	{
		HoldingRegister_t.ModeCommand_t.COD_X[i] = COD_10ptFactoryCalibrationHandle_t.x[i];
		HoldingRegister_t.ModeCommand_t.COD_Y[i] = COD_10ptFactoryCalibrationHandle_t.y[i];
	}

	for(int i = 0;i<4;i++)
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
	FRAM_OperationRead(FRAM_ADDRESS_TSS_SENS_CALIB,(uint8_t*)&TSS_SensorCalibration_t.byte,8);
	//Publish to modbus
	HoldingRegister_t.SensorCalibration_t.TSS_CF = TSS_SensorCalibration_t.slope;
	HoldingRegister_t.SensorCalibration_t.TSS_Intercept = TSS_SensorCalibration_t.intercept;
}


static inline void COD_SensorCalib_ypoint(uint8_t pt)
{
	if(pt == 1 || pt == 4)
	//First operate the sample pump
		PumpOperation(0x01);
	else
		PumpOperation(0x02);

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
		}
	}
}


void ParameterRelayAlarmProcess(void)
{
	for(int relay = 0;relay <7; relay++)
	{
		float threshold = HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_Threshold;
		uint16_t parameterIndex = HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_Parameter - 1;
//		if(parameterIndex >	5)
//			return 0;
		float hysterisis =  HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_Hysterisis;
		float parameterValue = ParameterValues_t.value[parameterIndex];
		if(HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_HIGHLOW == 0x01)//high
		{
			if(parameterValue >= threshold)
				CoilStatusRegister_t.CoilStatus_t.relay[relay] = 0x01;//alarm high
			if(parameterValue <= threshold - hysterisis)
				CoilStatusRegister_t.CoilStatus_t.relay[relay] = 0x00;//alarm low
		}else if(HoldingRegister_t.RelayOUTConfig_t[relay].Relay_OP_HIGHLOW == 0x02)//low
		{
			if(parameterValue <= threshold)
				CoilStatusRegister_t.CoilStatus_t.relay[relay] = 0x01;//alarm high
			if(parameterValue >= threshold + hysterisis)
				CoilStatusRegister_t.CoilStatus_t.relay[relay] = 0x00;//alarm low
		}
	}
	RelayToggle();
}