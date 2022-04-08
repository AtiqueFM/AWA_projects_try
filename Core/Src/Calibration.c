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
}

void pHSensorCalibrationmV(void)
{
	if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x02)
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
	}
	else if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x01)
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
	}
	//---------------------------- 2 point calibration end--------------------------------------------//
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
			HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
		}
		else if(pH_ElectronicCalibpoints_t.flag_p414)
		{
			pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count = pH_ElectronicCalibpoints_t.pH_ADCCounts;
			HoldingRegister_t.IOUTCalibandTest_t.pH_ADC_Counts_n414mA = pH_ElectronicCalibpoints_t.pH_ElectronicCalib_n414V_count;
			pH_ElectronicCalibpoints_t.flag_p414 = 0;
//			HoldingRegister_t.SensorCalibration_t.pHElecMeaages = 3;//Press calibrate
			HoldingRegister_t.IOUTCalibandTest_t.Current_OP_Calib_Command = 0;
		}
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
		//Reset the Common command Flag
		HoldingRegister_t.ModeCommand_t.CommonCommand = 0x00;
		HoldingRegister_t.ModeCommand_t.CommonCommandHMI = RESET;
	}
	//3-point calibration
	else if(HoldingRegister_t.SensorCalibration_t.Calibration_Type == 0x04)
	{
		//Logic yet to be defined
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
		//Logic yet to be defined
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

}
