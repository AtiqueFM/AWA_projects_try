/*
 * Calibration.h
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "Stdint.h"

void pHElectronicCalibrationmV(void);

void pHSensorCalibrationmV(void);

void Current_OP_Calibrate(uint16_t AO_Channel, uint16_t Calib_command);

void PWMCurrentCalibOutput(uint8_t CurrentChannel,uint32_t counts);

void CurrentOutputCalibration(uint8_t CurrentChannel);

void PT_ElectronicCalibration(void);

void pH_SensorCalibrationGetValues(void);

void pH_ElectronicCalibrationGetValues(void);

void PT_ElectronicCalibrationGetValues(void);

void COD_SensorCalibration(void);

void TSS_SensorCalibration(void);

void gaussEliminationLS(double x[],double y[],int len,int degree,uint8_t type);

#define PH_SLOPE_LOW_LIMIT			70.0f
#define PH_SLOPE_HIGH_LIMIT			130.0f
#define PH_INTERCEPT_LOW_LIMIT		6.0f
#define PH_INTERCEPT_HIGH_LIMIT		8.0f

typedef enum{
	ENTER_BUFFER_SOL_1 = 1,		/*<Enter the 1st pH buffer solution*/
	CAPTURE_STABLE_ADC_SOL_1,	/*<Before pressing the capture ADC button*/
	ENTER_BUFFER_SOL_2,			/*<Enter the 1st pH buffer solution*/
	CAPTURE_STABLE_ADC_SOL_2,	/*<Before pressing the capture ADC button*/
	ENTER_BUFFER_SOL_3,			/*<Enter the 1st pH buffer solution*/
	CAPTURE_STABLE_ADC_SOL_3,	/*<Before pressing the capture ADC button*/
	CALIBRATION_SUCCESSFULL,	/*<Calibration successful*/
	CALIBRATION_FAILED,			/*<Calibration failed*/

	/*pH Electronic calibration*/
	SIMULATE_POS_414_mV,		//9
	SIMULATE_NEG_414_mV,
	PRESS_CALIBRATE,

	/*PT-100 Temperature Electronic calibration*/
	SIMULATE_100_OHMS,			//12
	SIMULATE_150_OHMS,

	/*PT-1000 Temperature Electronic calibration*/
	SIMULATE_1000_OHMS,			//14
	SIMULATE_1500_OHMS,

	/*Sample Pump Stopped*/
	SAMPLING_PUMP_STOPPED,		//16

	/*Cleaning Pump Stopped*/
	CLEANING_PUMP_STOPPED,		//17
}AWAMessages;
#endif /* INC_CALIBRATION_H_ */
