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
#endif /* INC_CALIBRATION_H_ */
