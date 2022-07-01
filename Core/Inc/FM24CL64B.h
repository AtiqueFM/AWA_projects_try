/*
 * FM24CL64B.h
 *
 *  Created on: Jun 17, 2021
 *      Author: atiqueshaikh
 */

#ifndef INC_FM24CL64B_H_
#define INC_FM24CL64B_H_

#include "main.h"

#define FRAM_ADDRESS_MIN			0
#define FRAM_ADDRESS_MAX			0x1FFF
#define FRAM_SLAVE_ADDRESS			(uint8_t)0xAE

#define FRAM_ADDRESS_AO_ELEC_CALIB			FRAM_ADDRESS_MIN + sizeof(HoldingRegister_t.bytes)//uses 64 bytes
//#define FRAM_ADDRESS_pH_ELEC_CALIB			FRAM_ADDRESS_AO_ELEC_CALIB + (sizeof(CurrentOutputCalibration_t)*4)
//AO 1 - AO 8
#define FRAM_ADDRESS_pH_ELEC_CALIB			FRAM_ADDRESS_AO_ELEC_CALIB + (sizeof(CurrentOutputCalibration_t)*8) //uses 8 bytes slope and intercept
//1-pt calibration or 2-pt calibration
#define FRAM_ADDRESS_pH_SENS_CALIB			FRAM_ADDRESS_pH_ELEC_CALIB + 8 //used 32 bytes //uses 8 bytes //DEPRICATED ADDRESS
#define	FRAM_ADDRESS_PT_ELEC_CALIB  		FRAM_ADDRESS_pH_SENS_CALIB + 32 //uses 16 bytes PT-100 slope and intercept, PT-1000 slope and intercept
#define FRAM_ADDRESS_COD_SENS_CALIB			FRAM_ADDRESS_PT_ELEC_CALIB + 16//uses 8 bytes stores slope and intercept
#define FRAM_ADDRESS_COD_FACTORY_CALIB 		FRAM_ADDRESS_COD_SENS_CALIB + 8 // takes up 100 bytes
//PD1(0) and PD2(0)
#define FRAM_ADDRESS_COD_PD_ZERO			FRAM_ADDRESS_COD_FACTORY_CALIB + 100 //takes 8 bytes
#define FRAM_ADDRESS_TSS_FACTORY_CALIB		FRAM_ADDRESS_COD_PD_ZERO + 8 //takes 100 bytes
//PD2(0)
#define FRAM_ADDRESS_TSS_PD_ZERO			FRAM_ADDRESS_TSS_FACTORY_CALIB + 100 // takes 4 bytes
#define FRAM_ADDRESS_TSS_SENS_CALIB			FRAM_ADDRESS_TSS_PD_ZERO + 4 //take 8 bytes
//BOD single point calibration
#define FRAM_ADDRESS_BOD_SENS_CALIB			FRAM_ADDRESS_TSS_SENS_CALIB + 8 //takes 8 bytes
//AI 1 electronic calibration
#define FRAM_ADDRESS_AI1_ELEC_CALIB			FRAM_ADDRESS_BOD_SENS_CALIB + 8 //takes 8 bytes, slope and intercept
//AI 2 electronic calibration
#define FRAM_ADDRESS_AI2_ELEC_CALIB			FRAM_ADDRESS_AI1_ELEC_CALIB + 8 //takes 8 bytes, slope and intercept
//AI 1 sensor calibration
#define FRAM_ADDRESS_AI1_SENS_CALIB			FRAM_ADDRESS_AI2_ELEC_CALIB + 8 //takes 8 bytes
//AI 2 sensor calibration
#define FRAM_ADDRESS_AI2_SENS_CALIB			FRAM_ADDRESS_AI1_SENS_CALIB + 8 //takes 8 bytes
//Last Calibration data
#define FRAM_ADDRESS_LASTCALIB_HISTORY		FRAM_ADDRESS_AI2_SENS_CALIB + 8 //takes 164 bytes (c- 0:2 and time stamp)
//TSS factory Last Calibration data
#define FRAM_ADDRESS_TSSLASTCALIB_HISTORY	FRAM_ADDRESS_LASTCALIB_HISTORY + 164 //takes 164 bytes (c- 0:2 and time stamp)
//COD sensor calibration data
#define FRAM_ADDRESS_CODSENSLASTCALIB_HISTORY	FRAM_ADDRESS_TSSLASTCALIB_HISTORY + 164 //take 124 bytes
//TSS sensor calibration data
#define FRAM_ADDRESS_TSSSENSLASTCALIB_HISTORY	FRAM_ADDRESS_CODSENSLASTCALIB_HISTORY + 124 //take 124 bytes
//TSS sensor calibration data
#define FRAM_ADDRESS_pHSENSLASTCALIB_HISTORY	FRAM_ADDRESS_TSSSENSLASTCALIB_HISTORY + 124 //take 124 bytes
//Range select flag
#define FRAM_ADDRESS_RANGESELECT_FLAG			FRAM_ADDRESS_pHSENSLASTCALIB_HISTORY + 124 //takes 1 byte
//Upper ranges for COD and TSS
#define FRAM_ADDRESS_COD_UPPERLIMIT				FRAM_ADDRESS_RANGESELECT_FLAG + 1 //takes 4 byte
//Upper ranges for COD and TSS
#define FRAM_ADDRESS_TSS_UPPERLIMIT				FRAM_ADDRESS_COD_UPPERLIMIT + 4 //takes 4 byte
//Process Values
#define FRAM_ADDRESS_PROCESSVALUE			FRAM_ADDRESS_TSS_UPPERLIMIT + 4 //takes 16 byte add buffer for 10 process parameters (28 bytes)
//Check Screen Variables
#define FRAM_ADDRESS_CHECKSCREENVALUES		FRAM_ADDRESS_PROCESSVALUE + 100//NOT USED, reserved 50 bytes
//pH sensor calibration
#define FRAM_ADDRESS_PH_SENSOR_CALIB		FRAM_ADDRESS_CHECKSCREENVALUES + 50//in use 50 bytes + 40 bytes reserved (10 float values)
//COD sensor calibration
#define FRAM_ADDRESS_COD_SENSOR_CALIB		FRAM_ADDRESS_PH_SENSOR_CALIB + 90 //Used 30 bytes


void FRAM_OperationWrite(uint16_t u16_address,uint8_t *u8_pData,uint16_t u16_noByte);
void FRAM_OperationRead(uint16_t u16_address,uint8_t *u8_pData,uint16_t u16_noByte);

#endif /* INC_FM24CL64B_H_ */
