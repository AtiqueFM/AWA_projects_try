/*
 * Application.h
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include<stdint.h>

void ProcessModesCommands(void);

void CardAction(uint8_t CardID);

uint8_t CODADCCapture(uint8_t command);

uint8_t CODADCCapture_Sensor(uint8_t command,uint8_t sens_calib_point);

/*Helper functions*/
void bubble_sort(void);

void HMIInterlockingFlag(uint8_t lock,uint8_t state);

void RelayToggle(void);

void RelayToggleCoilInputUpdate(void);

void PumpOperation(uint8_t PumpNo);

void currentOutputTest(uint8_t CurrentChannel);

void ModbusSaveConfiguration(uint8_t data);

void ModbusReadConfiguration(void);

static inline void COD_SensorCalib_ypoint(uint8_t pt);

void ParameterIOutProcess(void);

void ParameterRelayAlarmProcess(void);
#endif /* INC_APPLICATION_H_ */
