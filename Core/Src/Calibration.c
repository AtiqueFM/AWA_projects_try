/*
 * Calibration.c
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */


#include "Calibration.h"
#include "stm32f4xx_dma.h"
#include "AD7682.h"
#include "ADS1115.h"
#include "Application.h"
#include "math.h"

/*Static variables*/
/*
 * 1 -> display y1
 * 2 -> display y2
 */
static uint8_t display_flag = 0;

typedef struct{
	float pt[3];
	float x[3];
}three_pt;

float middle_point = 0;

//test
float COD_3_ptcal[2];
float TSS_3_ptcal[2];
/*External variable*/
extern TIM_HandleTypeDef htim1;

void pHElectronicCalibrationmV(void)
{
	float y1 = 0.414f;
	float x1 = (float)pH_ElectronicCalibpoints_t.pH_ElectronicCalib_p414V_count - 65535.0f;
	float y2 = -0.414f;
	float x2 = (float)pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count;
	pH_ElectronicCalibpoints_t.pH_Slope = (y2-y1)/(x2-x1);
	pH_ElectronicCalibpoints_t.pH_Intercept = y1 - pH_ElectronicCalibpoints_t.pH_Slope*x1;
	HoldingRegister_t.SensorCalibration_t.pHElectronicCalibMessages = CALIBRATION_SUCCESSFULL;
}

three_pt sort_ph_values(float y1,float y2, float y3,float x1,float x2, float x3)
{
	three_pt pH_Values;
	if (y1>y2) {
		pH_Values.pt[1]=y1;
		pH_Values.pt[0]=y2;

		pH_Values.x[1] = x1;
		pH_Values.x[0] = x2;
	} else {
		pH_Values.pt[1]=y2;
		pH_Values.pt[0]=y1;

		pH_Values.x[1]=x2;
		pH_Values.x[0]=x1;
	}
	if (pH_Values.pt[1]>y3) {
		pH_Values.pt[2]=pH_Values.pt[1];

		pH_Values.x[2]=pH_Values.x[1];
	  if(pH_Values.pt[0]>y3){
		  pH_Values.pt[1]=pH_Values.pt[0];
		  pH_Values.pt[0]=y3;

		  pH_Values.x[1]=pH_Values.x[0];
		  pH_Values.x[0]=x3;
	  }else {
		  pH_Values.pt[1]=y3;

		  pH_Values.x[1]=x3;
	  }
	}else {
		pH_Values.pt[2]=y3;

		pH_Values.x[2]=x3;
	}

	return pH_Values;
}
void pHSensorCalibrationmV(void)
{

	uint8_t calib_error = RESET;
	float slope;
	float intercept;
	float slope_2;
	float intercept_2;

	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == TWO_POINT)
	{
		float x1_count = 0;
		float x2_count = 0;
		pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y1;
		pH_SensorCalibpoints_t.pH_Solution_2_y2 =  HoldingRegister_t.SensorCalibration_t.pH_PT.Y2;
		float y1 = pH_SensorCalibpoints_t.pH_Solution_1_y1;
		float y2 = pH_SensorCalibpoints_t.pH_Solution_2_y2;
		if(pH_SensorCalibpoints_t.pH_counts_1_x1 > 0x7fff)
			x1_count = (float)pH_SensorCalibpoints_t.pH_counts_1_x1 - 65535.0f;
		else
			x1_count = (float)pH_SensorCalibpoints_t.pH_counts_1_x1;
		if(pH_SensorCalibpoints_t.pH_counts_2_x2 > 0x7fff)
			x2_count = (float)pH_SensorCalibpoints_t.pH_counts_2_x2 - 65535.0f;
		else
			x2_count = (float)pH_SensorCalibpoints_t.pH_counts_2_x2;
		//Voltage 1
		float x1 = pH_ElectronicCalibpoints_t.pH_Slope * x1_count + pH_ElectronicCalibpoints_t.pH_Intercept;
		//Voltage 2
		float x2 = pH_ElectronicCalibpoints_t.pH_Slope * x2_count + pH_ElectronicCalibpoints_t.pH_Intercept;

		pH_SensorCalibpoints_t.pH_Solpe = (y2-y1)/(x2-x1);
		pH_SensorCalibpoints_t.pH_Intercept = y1 - pH_SensorCalibpoints_t.pH_Solpe * x1;
		float inv = 1/pH_SensorCalibpoints_t.pH_Solpe;
		pH_SensorCalibpoints_t.pH_Solpe = (inv/-0.05916f)*100.0f;
		//if(pH_SensorCalibpoints_t.pH_Solpe < 0)
			//pH_SensorCalibpoints_t.pH_Solpe *= -1;
		pH_SensorCalibpoints_t.measurement_Type = TWO_POINT;

	}
	else if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == SINGLE_POINT)
	{
		float x1_count = 0;
		float x2_count = 0;
		pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_1pt_calib_point1;
		pH_SensorCalibpoints_t.pH_Solution_2_y2 = 7.0f;// HoldingRegister_t.SensorCalibration_t.pH_1pt_calib_point1;
		float y1 = pH_SensorCalibpoints_t.pH_Solution_1_y1;
		float y2 = pH_SensorCalibpoints_t.pH_Solution_2_y2;
		if(pH_SensorCalibpoints_t.pH_counts_1_x1 > 0x7fff)
			x1_count = pH_SensorCalibpoints_t.pH_counts_1_x1 - 65535.0f;
		else
			x1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;
		//x2_count = 0;
		x2_count =  - pH_ElectronicCalibpoints_t.pH_Intercept / pH_ElectronicCalibpoints_t.pH_Slope;
		//volatge 1
		float x1 = pH_ElectronicCalibpoints_t.pH_Slope * x1_count + pH_ElectronicCalibpoints_t.pH_Intercept;
		//volatge 2
		float x2 = pH_ElectronicCalibpoints_t.pH_Slope * x2_count + pH_ElectronicCalibpoints_t.pH_Intercept;

		pH_SensorCalibpoints_t.pH_Solpe = (y2-y1)/(x2-x1);
		pH_SensorCalibpoints_t.pH_Intercept = y1 - pH_SensorCalibpoints_t.pH_Solpe * x1;
		float inv = 1/pH_SensorCalibpoints_t.pH_Solpe;
		pH_SensorCalibpoints_t.pH_Solpe = (inv/-0.05916f)*100.0f;
		//if(pH_SensorCalibpoints_t.pH_Solpe < 0)
			//pH_SensorCalibpoints_t.pH_Solpe *= -1;
		pH_SensorCalibpoints_t.measurement_Type = SINGLE_POINT;
	}
	else if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == THREE_POINT)
	{
		float x1_count = 0;
		float x2_count = 0;
		float x3_count = 0;
#if 0
		float y1 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y1;
		float y2 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y2;
		float y3 = HoldingRegister_t.SensorCalibration_t.pH_Y3;
#else
		pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y1;
		pH_SensorCalibpoints_t.pH_Solution_2_y2 =  HoldingRegister_t.SensorCalibration_t.pH_PT.Y2;
		pH_SensorCalibpoints_t.pH_Solution_3_y3 =  HoldingRegister_t.SensorCalibration_t.pH_Y3;
		float y1 = pH_SensorCalibpoints_t.pH_Solution_1_y1;
		float y2 = pH_SensorCalibpoints_t.pH_Solution_2_y2;
		float y3 = pH_SensorCalibpoints_t.pH_Solution_3_y3;
#endif

		if(pH_SensorCalibpoints_t.pH_counts_1_x1 > 0x7fff)
			x1_count = (float)pH_SensorCalibpoints_t.pH_counts_1_x1 - 65535.0f;
		else
			x1_count = (float)pH_SensorCalibpoints_t.pH_counts_1_x1;
		if(pH_SensorCalibpoints_t.pH_counts_2_x2 > 0x7fff)
			x2_count = (float)pH_SensorCalibpoints_t.pH_counts_2_x2 - 65535.0f;
		else
			x2_count = (float)pH_SensorCalibpoints_t.pH_counts_2_x2;
		if(pH_SensorCalibpoints_t.pH_counts_3_x3 > 0x7fff)
			x3_count = (float)pH_SensorCalibpoints_t.pH_counts_3_x3 - 65535.0f;
		else
			x3_count = (float)pH_SensorCalibpoints_t.pH_counts_3_x3;

		three_pt temp_3_pt = sort_ph_values(y1,y2,y3,x1_count,x2_count,x3_count);
		x1_count = temp_3_pt.x[0];
		x2_count = temp_3_pt.x[1];
		x3_count = temp_3_pt.x[2];

		y1 = temp_3_pt.pt[0];
		y2 = temp_3_pt.pt[1];middle_point = y2;pH_SensorCalibpoints_t.middle_value = middle_point;
		y3 = temp_3_pt.pt[2];

		//Voltage 1
		float x1 = pH_ElectronicCalibpoints_t.pH_Slope * x1_count + pH_ElectronicCalibpoints_t.pH_Intercept;
		//Voltage 2
		float x2 = pH_ElectronicCalibpoints_t.pH_Slope * x2_count + pH_ElectronicCalibpoints_t.pH_Intercept;
		//voltage 3
		float x3 = pH_ElectronicCalibpoints_t.pH_Slope * x3_count + pH_ElectronicCalibpoints_t.pH_Intercept;


#if 1
		slope = (y2-y1)/(x2-x1);
		intercept = y1 - slope * x1;
		//Calculating percentage
		float inv = 1/slope;
		slope = (inv/-0.05916f)*100.0f;
#else
		pH_SensorCalibpoints_t.pH_Solpe = (y2-y1)/(x2-x1);
		pH_SensorCalibpoints_t.pH_Intercept = y1 - pH_SensorCalibpoints_t.pH_Solpe * x1;
		//Calculating percentage
		float inv = 1/pH_SensorCalibpoints_t.pH_Solpe;
		pH_SensorCalibpoints_t.pH_Solpe = (inv/-0.05916f)*100.0f;
#endif
		/*
		 * Procedure to follow
		 * 1. reset complete calibration process.
		 * 2. reset the flags involved for that process.
		 * 3. show the following messages.
		 */

		//Checking if the intercept and slope is with in +- 2pH width.
#if 1
		if((slope < PH_SLOPE_LOW_LIMIT || slope > PH_SLOPE_HIGH_LIMIT) ||
						(intercept < PH_INTERCEPT_LOW_LIMIT || intercept > PH_INTERCEPT_HIGH_LIMIT) )
#else
		if((pH_SensorCalibpoints_t.pH_Solpe < PH_SLOPE_LOW_LIMIT || pH_SensorCalibpoints_t.pH_Solpe > PH_SLOPE_HIGH_LIMIT) ||
				(pH_SensorCalibpoints_t.pH_Intercept < PH_INTERCEPT_LOW_LIMIT || pH_SensorCalibpoints_t.pH_Intercept < PH_INTERCEPT_LOW_LIMIT > PH_INTERCEPT_HIGH_LIMIT) )
#endif
		{
			/*
			 * pH slope not within limits.
			 */

			//1. TODO: set the error flag, should not proceed further.
			calib_error = SET;

			//2. TODO: Reset all the variables for the calibration process.
			display_flag = 0;

			//Display Messages
			HoldingRegister_t.SensorCalibration_t.pHSensMessages = CALIBRATION_FAILED;
			return;

		}
		if(!calib_error)
		{
#if 1
			slope_2 = (y3-y2)/(x3-x2);
			intercept_2 = y2 - slope_2 * x2;
			//Calculating percentage
			inv = 1/slope_2;
			slope_2 = (inv/-0.05916f)*100.0f;
#else
			pH_SensorCalibpoints_t.pH_slope_range_2 = (y3-y2)/(x3-x2);
			pH_SensorCalibpoints_t.pH_Intercept_range_2 = y2 - pH_SensorCalibpoints_t.pH_slope_range_2 * x2;
			//Calculating percentage
			inv = 1/pH_SensorCalibpoints_t.pH_slope_range_2;
			pH_SensorCalibpoints_t.pH_slope_range_2 = (inv/-0.05916f)*100.0f;
#endif

			//Checking if the intercept and slope is with in +- 2pH width.
#if 1
			if((slope_2 < PH_SLOPE_LOW_LIMIT || slope_2 > PH_SLOPE_HIGH_LIMIT) ||
								(intercept_2 < PH_INTERCEPT_LOW_LIMIT || intercept_2 > PH_INTERCEPT_HIGH_LIMIT) )
#else
			if((pH_SensorCalibpoints_t.pH_slope_range_2 < PH_SLOPE_LOW_LIMIT || pH_SensorCalibpoints_t.pH_slope_range_2 > PH_SLOPE_HIGH_LIMIT) ||
					(pH_SensorCalibpoints_t.pH_Intercept_range_2 < PH_INTERCEPT_LOW_LIMIT || pH_SensorCalibpoints_t.pH_Intercept_range_2 < PH_INTERCEPT_LOW_LIMIT > PH_INTERCEPT_HIGH_LIMIT) )
#endif
			{
				/*
				 * pH slope not within limits.
				 */

				//1. TODO: set the error flag, should not proceed further.
				calib_error = SET;

				//2. TODO: Reset all the variables for the calibration process.
				display_flag = 0;

				//Display Messages
				HoldingRegister_t.SensorCalibration_t.pHSensMessages = CALIBRATION_FAILED;
				return;

			}

		}
		/*Calibration is successful*/
		pH_SensorCalibpoints_t.pH_Solpe = slope;
		pH_SensorCalibpoints_t.pH_Intercept = intercept;
		pH_SensorCalibpoints_t.pH_slope_range_2 = slope_2;
		pH_SensorCalibpoints_t.pH_Intercept_range_2 = intercept_2;

		HoldingRegister_t.SensorCalibration_t.pH_slope = pH_SensorCalibpoints_t.pH_Solpe;
		HoldingRegister_t.SensorCalibration_t.pH_intercept = pH_SensorCalibpoints_t.pH_Intercept;
		HoldingRegister_t.SensorCalibration_t.pH_slope_range_2 = pH_SensorCalibpoints_t.pH_slope_range_2;
		HoldingRegister_t.SensorCalibration_t.pH_intercept_range_2 = pH_SensorCalibpoints_t.pH_Intercept_range_2;

		//Display Messages
		HoldingRegister_t.SensorCalibration_t.pHSensMessages = CALIBRATION_SUCCESSFULL;

		pH_SensorCalibpoints_t.measurement_Type = THREE_POINT;
	}
	//Push to MODBUS
	HoldingRegister_t.SensorCalibration_t.pH_slope = pH_SensorCalibpoints_t.pH_Solpe;
	HoldingRegister_t.SensorCalibration_t.pH_intercept = pH_SensorCalibpoints_t.pH_Intercept;

}


void Current_OP_Calibrate(uint16_t AO_Channel, uint16_t Calib_command)
{
	if(Calib_command == 1)
		PWMCurrentCalibOutput(AO_Channel,(uint16_t)PWM_5mA_COUNTS);
	else if(Calib_command == 2)
		PWMCurrentCalibOutput(AO_Channel,(uint16_t)PWM_19mA_COUNTS);
}

void PWMCurrentCalibOutput(uint8_t CurrentChannel,uint32_t counts)
{
	if(CurrentChannel == AO1)
	{
		htim1.Instance->CCR1 = PWM_TIM_FS - counts - 1;
	}
	if(CurrentChannel == AO2)
	{
		htim1.Instance->CCR2 = PWM_TIM_FS - counts - 1;
	}
	if(CurrentChannel == AO3)
	{
		htim1.Instance->CCR3 = PWM_TIM_FS - counts - 1;
	}
	if(CurrentChannel == AO4)
	{
		htim1.Instance->CCR4 = PWM_TIM_FS - counts - 1;
	}

}

void CurrentOutputCalibration(uint8_t CurrentChannel)
{

		float y1 = HoldingRegister_t.IOUTCalibandTest_t.CurrentOPPoints_t[CurrentChannel - 1].Current_OP_Calib_Point1;
		float x1 = (float)PWM_5mA_COUNTS;//2070.0f;
		float y2 = HoldingRegister_t.IOUTCalibandTest_t.CurrentOPPoints_t[CurrentChannel - 1].Current_OP_Calib_Point2;
		float x2 = (float)PWM_19mA_COUNTS;//6993.0f;
		float AO1_slope = (y2-y1)/(x2-x1);
		float AO1_intercept = y1 - AO1_slope*x1;
		CurrentOutputCalibration_t[CurrentChannel - 1].AO_slope = AO1_slope;
		CurrentOutputCalibration_t[CurrentChannel - 1].AO_intercept = AO1_intercept;
}

void PT_ElectronicCalibration(void)
{
	float y1 = 0;
	float y2 = 0;
	float x1 = 0;
	float x2 = 0;
	if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)
	{
		y1 = 100.0f;
		x1 = (float)PT_ElectronicCalibration_t.PT_100_Counts;
		y2 = 150.0f;
		x2 = (float)PT_ElectronicCalibration_t.PT_150_Counts;
		PT_ElectronicCalibration_t.PT100_slope = (y2-y1)/(x2-x1);
		PT_ElectronicCalibration_t.PT100_intercept = y1 - PT_ElectronicCalibration_t.PT100_slope*x1;
	}else if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT1000)
	{
		y1 = 1000.0f;
		x1 = (float)PT_ElectronicCalibration_t.PT_1000_Counts;
		y2 = 1500.0f;
		x2 = (float)PT_ElectronicCalibration_t.PT_1500_Counts;
		PT_ElectronicCalibration_t.PT1000_slope = (y2-y1)/(x2-x1);
		PT_ElectronicCalibration_t.PT1000_intercept = y1 - PT_ElectronicCalibration_t.PT1000_slope*x1;
	}

}

void pH_SensorCalibrationGetValues(void)
{
	float milli_volt = 0.0f;
//	/*
//	 * 1 -> display y1
//	 * 2 -> display y2
//	 */
//	static uint8_t display_flag = 0;
	//-------------------------1 point calibration--------------------------------------//
	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == SINGLE_POINT)
	{
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x01)//set sample
		{
			pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_1pt_calib_point1;
			pH_SensorCalibpoints_t.flag_display_count = 0x01;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x02)//set ADC counts
		{
			pH_SensorCalibpoints_t.pH_counts_1_x1 = pH_SensorCalibpoints_t.pH_ADCCounts;//used in calibration
			HoldingRegister_t.SensorCalibration_t.pH_1pt_Cal_point_1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;
			pH_SensorCalibpoints_t.flag_display_count = 0x03;//no action on display value
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		//Reset command
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x04)
		{
			pH_SensorCalibpoints_t.pH_counts_1_x1 = 0;
			pH_SensorCalibpoints_t.pH_Solution_1_y1 = 0;
			pH_SensorCalibpoints_t.flag_display_count = 0;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		/*For HMI display*/
		if(pH_SensorCalibpoints_t.flag_display_count == 0x01)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/

			//Read by HMI (Display only)
			HoldingRegister_t.SensorCalibration_t.pH_1pt_Cal_point_1_value = pH_calculated;

			//For slope and intercept calculation.
			HoldingRegister_t.SensorCalibration_t.pH_1pt_Cal_point_1_count = pH_SensorCalibpoints_t.pH_ADCCounts;

		}
	}
	//----------------------------------------------------------------------------------//
	//------------------------ 2 point calibration--------------------------------------//
	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == TWO_POINT)
	{
#if 1
		//For x1 and y1
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x01)//grab y1
		{
			display_flag = 1;//display x1
			pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y1;//used in calibration
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x02)//grab y1
		{
			display_flag = 0;//FREEZE x1
			pH_SensorCalibpoints_t.pH_counts_1_x1 = pH_SensorCalibpoints_t.pH_ADCCounts;//used in calibration
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x03)//grab y1
		{
			/*
			 * Reset the X1 and Y1
			 */

			display_flag = 0;//FREEZE x1

			pH_SensorCalibpoints_t.pH_Solution_1_y1 = 0;
			pH_SensorCalibpoints_t.pH_counts_1_x1 = 0;

			//reset the holding register
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_value = 0;
			//HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = 0;
			//HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;

			HoldingRegister_t.SensorCalibration_t.pH_PT.Y1 = 0;


			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		//for x2 y2
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x04)//grab y2
		{
			display_flag = 2;//display x2
			pH_SensorCalibpoints_t.pH_Solution_2_y2 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y2;//used in calibration
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x05)//grab y2
		{
			display_flag = 0;//FREEZE x2
			pH_SensorCalibpoints_t.pH_counts_2_x2 = pH_SensorCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_counts_2_x2;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x06)//reset x2,y2
		{
			/*
			 * Reset the X2 and Y2
			 */
			display_flag = 0;

			pH_SensorCalibpoints_t.pH_Solution_2_y2 = 0;
			pH_SensorCalibpoints_t.pH_counts_2_x2 = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;

			HoldingRegister_t.SensorCalibration_t.pH_PT.Y2 = 0;

			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		/*Display calculated pH*/
		if(display_flag == 1)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/


			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_1_x1 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_value = pH_calculated;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;
		}else if(display_flag == 2)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/

			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_2_x2 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = pH_calculated;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_ADCCounts;
		}
#else
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
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/


			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_1_x1 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_value = pH_calculated;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;
		}else if(pH_SensorCalibpoints_t.flag_display_count == 0x02)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/

			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_2_x2 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = pH_calculated;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_ADCCounts;
		}
#endif
	}
	//---------------------------- 2 point calibration end--------------------------------------------//

	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == THREE_POINT)
	{
		//For x1 and y1
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x01)//grab y1
		{
			display_flag = 1;//display x1
			pH_SensorCalibpoints_t.pH_Solution_1_y1 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y1;//used in calibration
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
			//Display Messages
			HoldingRegister_t.SensorCalibration_t.pHSensMessages = CAPTURE_STABLE_ADC_SOL_1;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x02)//grab y1
		{
			display_flag = 0;//FREEZE x1
			pH_SensorCalibpoints_t.pH_counts_1_x1 = pH_SensorCalibpoints_t.pH_ADCCounts;//used in calibration
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_counts_1_x1;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
			//Display Messages
			//HoldingRegister_t.SensorCalibration_t.pHSensMessages = ENTER_BUFFER_SOL_2;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x03)//grab y1
		{
			/*
			 * Reset the X1 and Y1
			 */

			display_flag = 0;//FREEZE x1

			pH_SensorCalibpoints_t.pH_Solution_1_y1 = 0;
			pH_SensorCalibpoints_t.pH_counts_1_x1 = 0;

			//reset the holding register
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_value = 0;
			//HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = 0;
			//HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;

			HoldingRegister_t.SensorCalibration_t.pH_PT.Y1 = 0;


			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		//for x2 y2
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x04)//grab y2
		{
			display_flag = 2;//display x2
			pH_SensorCalibpoints_t.pH_Solution_2_y2 = HoldingRegister_t.SensorCalibration_t.pH_PT.Y2;//used in calibration
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
			//Display Messages
			HoldingRegister_t.SensorCalibration_t.pHSensMessages = CAPTURE_STABLE_ADC_SOL_2;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x05)//grab y2
		{
			display_flag = 0;//FREEZE x2
			pH_SensorCalibpoints_t.pH_counts_2_x2 = pH_SensorCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_counts_2_x2;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x06)//reset x2,y2
		{
			/*
			 * Reset the X2 and Y2
			 */
			display_flag = 0;

			pH_SensorCalibpoints_t.pH_Solution_2_y2 = 0;
			pH_SensorCalibpoints_t.pH_counts_2_x2 = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;

			HoldingRegister_t.SensorCalibration_t.pH_PT.Y2 = 0;

			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		//for x3 y3
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x07)//grab y2
		{
			display_flag = 3;//display x3
			pH_SensorCalibpoints_t.pH_Solution_3_y3 = HoldingRegister_t.SensorCalibration_t.pH_Y3;//used in calibration
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x08)//grab y2
		{
			display_flag = 0;//FREEZE x2
			pH_SensorCalibpoints_t.pH_counts_3_x3 = pH_SensorCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_count = pH_SensorCalibpoints_t.pH_counts_3_x3;
			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
			//Display Messages
			HoldingRegister_t.SensorCalibration_t.pHSensMessages = CAPTURE_STABLE_ADC_SOL_3;
		}
		if(HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command == 0x09)//reset x2,y2
		{
			/*
			 * Reset the X2 and Y2
			 */
			display_flag = 0;

			pH_SensorCalibpoints_t.pH_Solution_3_y3 = 0;
			pH_SensorCalibpoints_t.pH_counts_3_x3 = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_value = 0;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_count = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Y3 = 0;

			HoldingRegister_t.SensorCalibration_t.Sensor_Calib_Command = 0;
		}
		/*Display calculated pH*/
		if(display_flag == 1)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/


			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_1_x1 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_value = pH_calculated;
			//HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = 0;

			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_1_count = pH_SensorCalibpoints_t.pH_ADCCounts;
			//HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = 0;
		}else if(display_flag == 2)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/

			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_2_x2 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_value = pH_calculated;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_2_count = pH_SensorCalibpoints_t.pH_ADCCounts;
		}else if(display_flag == 3)
		{
			//convert the mV to Calculated pH
			float pH_calculated = (-16.908f * InputRegister_t.SlotParameter.pH_Live_Volatge) + 7.0f; /*<To be sent to the HMI, (414,14) and (-414,0) line equation*/

			//Store in the live buffer
			pH_SensorCalibpoints_t.pH_Calculated_3_x3 = pH_calculated;
			//Read by HMI
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_value = pH_calculated;
			HoldingRegister_t.SensorCalibration_t.pH_Cal_Point_3_count = pH_SensorCalibpoints_t.pH_ADCCounts;
		}
	}
}

void pH_ElectronicCalibrationGetValues(void)
{
	if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command == 9)
	{
		if(!pH_ElectronicCalibpoints_t.flag_p414)
		{
			pH_ElectronicCalibpoints_t.pH_ElectronicCalib_p414V_count = pH_ElectronicCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_p414mA = pH_ElectronicCalibpoints_t.pH_ElectronicCalib_p414V_count;
			pH_ElectronicCalibpoints_t.flag_p414 = 1;
//			HoldingRegister_t.SensorCalibration_t.pHElecMeaages = 2;//simulate -414mV
			HoldingRegister_t.SensorCalibration_t.pHElectronicCalibMessages = SIMULATE_NEG_414_mV;
			HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
		}
		else if(pH_ElectronicCalibpoints_t.flag_p414)
		{
			pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count = pH_ElectronicCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_n414mA = pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count;
			pH_ElectronicCalibpoints_t.flag_p414 = 0;
//			HoldingRegister_t.SensorCalibration_t.pHElecMeaages = 3;//Press calibrate
			HoldingRegister_t.SensorCalibration_t.pHElectronicCalibMessages = PRESS_CALIBRATE;
			HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
		}
	}else if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command == 11)
	{
		pH_ElectronicCalibpoints_t.flag_p414 = RESET;

		/*Reset buffers*/
		pH_ElectronicCalibpoints_t.pH_ElectronicCalib_p414V_count = 0;
		HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_p414mA = 0;

		pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count = 0;
		HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_n414mA = 0;

		/*Simulate +414 mV messages*/
		HoldingRegister_t.SensorCalibration_t.pHElectronicCalibMessages = SIMULATE_POS_414_mV;

		HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
	}
}

void PT_ElectronicCalibrationGetValues(void)
{
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
	}else if(HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command == 12)
	{
		PT_ElectronicCalibration_t.flag_15 = RESET;

		/*Reset buffers*/
		if(HoldingRegister_t.IOUTCalibandTest_t.CalibrationType == TEMP_TYPE_PT100)
		{
			PT_ElectronicCalibration_t.PT_100_Counts = 0;
			HoldingRegister_t.IOUTCalibandTest_t.PT100_ADC_Counts_100  = 0;

			PT_ElectronicCalibration_t.PT_150_Counts = 0;
			HoldingRegister_t.IOUTCalibandTest_t.PT100_ADC_Counts_150  = 0;

			//Simulate 100 ohm
			HoldingRegister_t.SensorCalibration_t.TempElectronicCalibMessages = SIMULATE_100_OHMS;

		}
		else{
			PT_ElectronicCalibration_t.PT_100_Counts = 0;
			HoldingRegister_t.IOUTCalibandTest_t.PT100_ADC_Counts_100  = 0;

			PT_ElectronicCalibration_t.PT_150_Counts = 0;
			HoldingRegister_t.IOUTCalibandTest_t.PT100_ADC_Counts_150  = 0;

			//Simulate 1000 ohm
			HoldingRegister_t.SensorCalibration_t.TempElectronicCalibMessages = SIMULATE_1000_OHMS;
		}
		HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;

	}
}

void COD_SensorCalibration(void)
{
	//2-point calibration
	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x03)
	{
		float x1 = 0.0f;
		float y1 = HoldingRegister_t.SensorCalibration_t.COD_Y1;//COD_SensorCalibration_t.y1;
		float x2 = HoldingRegister_t.SensorCalibration_t.COD_X2;//COD_SensorCalibration_t.x2;
		float y2 = HoldingRegister_t.SensorCalibration_t.COD_Y2;//COD_SensorCalibration_t.y2;
		//float slope = (y2-y1)/(x2-x1);
		//float intercept = (y2 - slope * x2);
		float slope = (x2-x1)/(y2-y1);
		float intercept = (x2 - slope * y2);
		COD_SensorCalibration_t.slope = slope;
		COD_SensorCalibration_t.intercept = intercept;
		//Publish to the MODBUS
		HoldingRegister_t.SensorCalibration_t.COD_CF = slope;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept = intercept;

		HoldingRegister_t.SensorCalibration_t.COD_Messages = CALIBRATION_SUCCESSFULL;
		COD_SensorCalibration_t.measurementType = 0x03;
		//Reset the Common command Flag
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
	}
	//3-point calibration
	else if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x04)
	{
//		float x1 = HoldingRegister_t.SensorCalibration_t.COD_X1;
//		float y1 = HoldingRegister_t.SensorCalibration_t.COD_Y1;//COD_SensorCalibration_t.y1;
//		float x2 = HoldingRegister_t.SensorCalibration_t.COD_X2;COD_SensorCalibration_t.middle_value = x2;//Middle value //COD_SensorCalibration_t.x2;
//		float y2 = HoldingRegister_t.SensorCalibration_t.COD_Y2;//COD_SensorCalibration_t.y2;
//		float x3 = HoldingRegister_t.SensorCalibration_t.COD_X3;//COD_SensorCalibration_t.x2;
//		float y3 = HoldingRegister_t.SensorCalibration_t.COD_Y3;//COD_SensorCalibration_t.y2;

		double  x[3],y[3];
		int degree = 1;
		int no_of_points = 3;
		int data_transfer_type = 3;

		x[0] = HoldingRegister_t.SensorCalibration_t.COD_X1;
		y[0] = HoldingRegister_t.SensorCalibration_t.COD_Y1;//COD_SensorCalibration_t.y1;
		x[1] = HoldingRegister_t.SensorCalibration_t.COD_X2;//COD_SensorCalibration_t.x2;
		y[1] = HoldingRegister_t.SensorCalibration_t.COD_Y2;//COD_SensorCalibration_t.y2;
		x[2] = HoldingRegister_t.SensorCalibration_t.COD_X3;//COD_SensorCalibration_t.x2;
		y[2] = HoldingRegister_t.SensorCalibration_t.COD_Y3;//COD_SensorCalibration_t.y2;

		//test
//		x[1-1] = 98;//HoldingRegister_t.SensorCalibration_t.COD_X1;
//		y[1-1] = 100;//HoldingRegister_t.SensorCalibration_t.COD_Y1;//COD_SensorCalibration_t.y1;
//		x[2-1] = 255;//HoldingRegister_t.SensorCalibration_t.COD_X2;//COD_SensorCalibration_t.x2;
//		y[2-1] = 270;//HoldingRegister_t.SensorCalibration_t.COD_Y2;//COD_SensorCalibration_t.y2;
//		x[3-1] = 306;//HoldingRegister_t.SensorCalibration_t.COD_X3;//COD_SensorCalibration_t.x2;
//		y[3-1] = 300;//HoldingRegister_t.SensorCalibration_t.COD_Y3;//COD_SensorCalibration_t.y2;

		//sorting the points in ascending order
		//three_pt cod_pts = sort_ph_values(y1,y2,y3,x1,x2,x3);

#if 1
		gaussEliminationLS(y, x, no_of_points, degree, data_transfer_type);

		COD_SensorCalibration_t.slope = COD_3_ptcal[1];
		COD_SensorCalibration_t.intercept = COD_3_ptcal[0];
		HoldingRegister_t.SensorCalibration_t.COD_CF = COD_SensorCalibration_t.slope;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept = COD_SensorCalibration_t.intercept;

#else
		/*
		 * Error margin is not defined.
		 */
		float slope_range_1 = (cod_pts.x[1] - cod_pts.x[0])/((cod_pts.pt[1] - cod_pts.pt[0]));
		float intercept_range_1 = cod_pts.x[0] - slope_range_1*cod_pts.pt[0];
		float slope_range_2 = (cod_pts.x[2] - cod_pts.x[1])/((cod_pts.pt[2] - cod_pts.pt[1]));
		float intercept_range_2 = cod_pts.x[1] - slope_range_2*cod_pts.pt[1];

		COD_SensorCalibration_t.slope = slope_range_1;
		COD_SensorCalibration_t.intercept = intercept_range_1;
		COD_SensorCalibration_t.slope_range_2 = slope_range_1;
		COD_SensorCalibration_t.intercept_range_2 = intercept_range_1;
		//Publish to the MODBUS
		HoldingRegister_t.SensorCalibration_t.COD_CF = slope_range_1;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept = intercept_range_1;
		HoldingRegister_t.SensorCalibration_t.COD_CF_RANGE_2 = slope_range_2;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept_RANGE_2 = intercept_range_2;

		float slope = (x2-x1)/(y2-y1);
		float intercept = (x2 - slope * y2);
		COD_SensorCalibration_t.slope = slope;
		COD_SensorCalibration_t.intercept = intercept;
		//Publish to the MODBUS
		HoldingRegister_t.SensorCalibration_t.COD_CF = slope;
		HoldingRegister_t.SensorCalibration_t.COD_Intercept = intercept;
#endif

		HoldingRegister_t.SensorCalibration_t.COD_Messages = CALIBRATION_SUCCESSFULL;
		COD_SensorCalibration_t.measurementType = 0x04;

		//Reset the Common command Flag
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
	}


}

void TSS_SensorCalibration(void)
{
	//2-point calibration
	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x03)
	{
		float x1 = 0.0f;
		float y1 = HoldingRegister_t.SensorCalibration_t.TSS_Y1;//COD_SensorCalibration_t.y1;
		float x2 = HoldingRegister_t.SensorCalibration_t.TSS_X2;//COD_SensorCalibration_t.x2;
		float y2 = HoldingRegister_t.SensorCalibration_t.TSS_Y2;//COD_SensorCalibration_t.y2;
		//float slope = (y2-y1)/(x2-x1);
		//float intercept = (y2 - slope * x2);
		float slope = (x2-x1)/(y2-y1);
		float intercept = (x2 - slope * y2);
		TSS_SensorCalibration_t.slope = slope;
		TSS_SensorCalibration_t.intercept = intercept;
		//Publish to the MODBUS
		HoldingRegister_t.SensorCalibration_t.TSS_CF = slope;
		HoldingRegister_t.SensorCalibration_t.TSS_Intercept = intercept;
		//Reset the Common command Flag
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
	}
	//3-point calibration
	else if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x04)
	{
//		float x1 = HoldingRegister_t.SensorCalibration_t.TSS_X1;
//		float y1 = HoldingRegister_t.SensorCalibration_t.TSS_Y1;//COD_SensorCalibration_t.y1;
//		float x2 = HoldingRegister_t.SensorCalibration_t.TSS_X2;TSS_SensorCalibration_t.middle_value = x2;//Middle value //COD_SensorCalibration_t.x2;
//		float y2 = HoldingRegister_t.SensorCalibration_t.TSS_Y2;//COD_SensorCalibration_t.y2;
//		float x3 = HoldingRegister_t.SensorCalibration_t.TSS_X3;//COD_SensorCalibration_t.x2;
//		float y3 = HoldingRegister_t.SensorCalibration_t.TSS_Y3;//COD_SensorCalibration_t.y2;

		double  x[3],y[3];
		int degree = 1;
		int no_of_points = 3;
		int data_transfer_type = 4;

		x[0] = HoldingRegister_t.SensorCalibration_t.TSS_X1;
		y[0] = HoldingRegister_t.SensorCalibration_t.TSS_Y1;//COD_SensorCalibration_t.y1;
		x[1] = HoldingRegister_t.SensorCalibration_t.TSS_X2;//COD_SensorCalibration_t.x2;
		y[1] = HoldingRegister_t.SensorCalibration_t.TSS_Y2;//COD_SensorCalibration_t.y2;
		x[2] = HoldingRegister_t.SensorCalibration_t.TSS_X3;//COD_SensorCalibration_t.x2;
		y[2] = HoldingRegister_t.SensorCalibration_t.TSS_Y3;//COD_SensorCalibration_t.y2;


		gaussEliminationLS(y, x, no_of_points, degree, data_transfer_type);

		TSS_SensorCalibration_t.slope = TSS_3_ptcal[1];
		TSS_SensorCalibration_t.intercept = TSS_3_ptcal[0];
		HoldingRegister_t.SensorCalibration_t.TSS_CF = TSS_SensorCalibration_t.slope;
		HoldingRegister_t.SensorCalibration_t.TSS_Intercept = TSS_SensorCalibration_t.intercept;
#if 0
		//sorting the points in ascending order
		three_pt tss_pts = sort_ph_values(x1,x2,x3,y1,y2,y3);
		/*
		 * Error margin is not defined.
		 */
		float slope_range_1 = (tss_pts.pt[1] - tss_pts.pt[0])/((tss_pts.x[1] - tss_pts.x[0]));
		float intercept_range_1 = tss_pts.pt[0] - slope_range_1*tss_pts.x[0];
		float slope_range_2 = (tss_pts.pt[2] - tss_pts.pt[1])/((tss_pts.x[2] - tss_pts.x[1]));
		float intercept_range_2 = tss_pts.pt[1] - slope_range_2*tss_pts.x[1];

		TSS_SensorCalibration_t.slope = slope_range_1;
		TSS_SensorCalibration_t.intercept = intercept_range_1;
		TSS_SensorCalibration_t.slope_range_2 = slope_range_1;
		TSS_SensorCalibration_t.intercept_range_2 = intercept_range_1;
		//Publish to the MODBUS
		HoldingRegister_t.SensorCalibration_t.TSS_CF = slope_range_1;
		HoldingRegister_t.SensorCalibration_t.TSS_Intercept = intercept_range_1;
		HoldingRegister_t.SensorCalibration_t.TSS_CF_RANGE_2 = slope_range_2;
		HoldingRegister_t.SensorCalibration_t.TSS_Intercept_RANGE_2 = intercept_range_2;
#endif
		HoldingRegister_t.SensorCalibration_t.TSS_Messages = CALIBRATION_SUCCESSFULL;
		//Reset the Common command Flag
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
	}


}


void gaussEliminationLS(double x[],double y[],int len,int degree,uint8_t type){

	int i,j,k;
	int n = degree;
	int N = len;
	int g = n+2; //n (old function)
	int m = n+1;

	double X[2*n+1];
	    for(i=0;i<=2*n;i++){
	        X[i]=0;
	        for(j=0;j<N;j++){
	            X[i]=X[i]+pow(x[j],i);
	        }
	    }

	    //the normal augmented matrix
	    double B[n+1][n+2];
	    // rhs
	    double Y[n+1];
	    for(i=0;i<=n;i++){
	        Y[i]=0;
	        for(j=0;j<N;j++){
	            Y[i]=Y[i]+pow(x[j],i)*y[j];
	        }
	    }
	    for(i=0;i<=n;i++){
	        for(j=0;j<=n;j++){
	            B[i][j]=X[i+j];
	        }
	    }
	    for(i=0;i<=n;i++){
	        B[i][n+1]=Y[i];
	    }
	    double A[n+1];

    for(i=0;i<m-1;i++){
        //Partial Pivoting
        for(k=i+1;k<m;k++){
            //If diagonal element(absolute vallue) is smaller than any of the terms below it
            if(fabs(B[i][i])<fabs(B[k][i])){
                //Swap the rows
                for(j=0;j<g;j++){
                    double temp;
                    temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }
            }
        }
        //Begin Gauss Elimination
        for(k=i+1;k<m;k++){
            double  term=B[k][i]/ B[i][i];
            for(j=0;j<g;j++){
                B[k][j]=B[k][j]-term*B[i][j];
            }
        }

    }
    //Begin Back-substitution
    for(i=m-1;i>=0;i--){
        A[i]=B[i][g-1];
        for(j=i+1;j<g-1;j++){
            A[i]=A[i]-B[i][j]*A[j];
        }
        A[i]=A[i]/B[i][i];
    }

    if(type == 1)
    {
        //publish the data on the modbus
        for(int i = 0;i<4;i++)
        {
        	HoldingRegister_t.ModeCommand_t.C[i] = A[i];
        	COD_10ptFactoryCalibrationHandle_t.c[i] = A[i];
        }
    }else if(type == 2)
    {
    	for(int i = 0;i<3;i++)
		{
			HoldingRegister_t.ModeCommand_t.TSS_K[i] = A[i];
			TSS_10ptFactoryCalibrationHandle_t.c[i] = A[i];
		}
    }
    /*COD 3-point calibration*/
    else if(type == 3)
    {
    	for(int i = 0;i<2;i++)
		{
    		COD_3_ptcal[i] = A[i];
			//TSS_10ptFactoryCalibrationHandle_t.c[i] = A[i];
		}
    }
    /*TSS 3-point calibration*/
    else if(type == 4)
    {
    	for(int i = 0;i<2;i++)
		{
    		TSS_3_ptcal[i] = A[i];
			//TSS_10ptFactoryCalibrationHandle_t.c[i] = A[i];
		}
    }

}
