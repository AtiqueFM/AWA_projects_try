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
#define FRAM_ADDRESS_pH_SENS_CALIB			FRAM_ADDRESS_pH_ELEC_CALIB + 8 //uses 8 bytes
#define	FRAM_ADDRESS_PT_ELEC_CALIB  		FRAM_ADDRESS_pH_SENS_CALIB + 8 //uses 16 bytes PT-100 slope and intercept, PT-1000 slope and intercept
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




void FRAM_OperationWrite(uint16_t u16_address,uint8_t *u8_pData,uint16_t u16_noByte);
void FRAM_OperationRead(uint16_t u16_address,uint8_t *u8_pData,uint16_t u16_noByte);

#endif /* INC_FM24CL64B_H_ */