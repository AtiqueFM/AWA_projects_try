/*
 * Application.h
 *
 *  Created on: Jan 3, 2022
 *      Author: ATIQUE
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include<stdint.h>
#include "main.h"

/*Node for sensor calibration*/
struct Sensornode{
	float intercept;
	float slope;
	uint32_t timestamp;		/*<Epoch time stamp*/
	struct Sensornode *next;
};

struct Sensornode_3pt{
	float intercept;
	float slope;
	float intercept_3pt;
	float slope_3pt;
	uint32_t timestamp;		/*<Epoch time stamp*/
	struct Sensornode_3pt *next;
};

/*Node for factory calibration*/
struct Factorynode{
	float c0;
	float c1;
	float c2;
	uint32_t timestamp;		/*<Epoch time stamp*/
	struct Factorynode *next;
};

typedef struct{
	uint32_t baudrate;	//14
	uint32_t parity_bit;//15
	uint32_t stop_bits;//10
	uint32_t data_length;//11
	uint32_t slave_ID;//12
	uint32_t baudrate_selection;//13
}MODBUS_config_t;

typedef struct queue
{
	int maxSize;
	int front;
	int rear;
	int *data;
}QueueHandle_t;

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

//Check screen
void CheckScreen_PumpOperation(uint8_t PumpNo);

uint8_t CheckScreen_CODADCCapture(uint8_t command);

void CheckScreen_bubble_sort(uint16_t filterLimit);

void currentOutputTest(uint8_t CurrentChannel);

void ModbusSaveConfiguration(uint8_t data);

void ModbusReadConfiguration(void);

static inline void COD_SensorCalib_ypoint(uint8_t pt);

void ParameterIOutProcess(void);

void ParameterRelayAlarmProcess(void);

/*Helper functions*/
/*
 * Function name :- Application_ReadAcid
 * Task :- Runs the acid pump and measures the raw absorbance.
 */
void Application_ReadAcid(void);

/*
 * Function name :- Application_ReadSample
 * Task :- Runs the acid pump and measures the raw absorbance.
 */
void Application_ReadSample(void);

/*
 * Function name :- Application_SetAsZero
 * Task :- Overwrite the existing PD1(0) and PD2(0) by taking the current PD1 and PD2 values.
 */
void Application_SetAsZero(uint8_t parameter);
/*
 * Function name :- Application_LastCaldataToModbus
 *
 */
void Application_LastCaldataToModbus(void);

void sensor_insertNode(struct Sensornode **head,float intercept,float slope, unsigned int timestamp);
void sensor_3pt_insertNode(struct Sensornode_3pt **head,float intercept,float slope ,float intercept_range_2,float slope_range_2,unsigned int timestamp);
/*
 * Function name :- factory_insertNode
 * Arguments :-
 * 				1. Head of the respective linked list.
 * 				2. c0-c2 = Factory Calibration.
 * 				3. epoch time stamp.
 * Return	:-
 * 				None.
 * Description :-
 * 				Inserts node at the end of the linked list of the respective linked list head provided.
 */
void factory_insertNode(struct Factorynode **head,float c0,float c1,float c2, unsigned int timestamp);
//void displayLinkedList(struct Sensornode **head);
void sensor_deleteNode(struct Sensornode **head);
/*
 * Function name	:- facroty_deleteNode
 * Arguments		:-
 * 						1. Head of the respective linked list.
 * 	Return			:-
 * 						None
 * 	Description		:-
 * 						1. Deletes the node from the last of the respective head provided.
 */
void factory_deleteNode(struct Factorynode **head);
/*
 * Function name	:- factory_dataTransfer
 * Arguments		:-
 * 						1. Head of the respective linked list.
 * 						2. pointer to LastCalibrationFactoryHanlde_t.
 * 						3. unsigned data for overflow flag.
 * 						4. index count for the data that is to be stored in the linked list.
 * Return			:-
 * 						None.
 * Description		:-
 * 						New factory calibration coefficient is stored in the linked list and then transfer to the Input register by sorting
 * 						it from recent value to old value.
 */
void factory_dataTransfer(struct Factorynode **head,LastCalibrationFactoryHanlde_t *pLastCalibration_t,unsigned overflowFlag,uint8_t indexCount);

void sensor_dataTransfer(struct Sensornode **head,LastCalibrationSensorHandle_t *pLastCalibration_t,unsigned overflowFlag,uint8_t indexCount);

void CODCalculateValue(void);

void TSScalculateValue(void);

void AWA_RangeSelect(void);

void FlowSensorReadStatus(void);

void MILSwitchReadStatus(void);

void pumpOperationStop(void);

void Application_commandCheckandProcess(void);

void Application_AWAIOProcess(void);	//relays and AO

void Application_MODBUSRXprocess(void);

void Application_StoreData(void);		//Stores the process variables and calibration data if the flag is raised

void Application_MODBUSParseQuery(void);	//Will parse the modbus query from Master and also perform the action on the command received

uint32_t adjust_time_zone_to_epoch(int tz, uint32_t epoch);

uint32_t adjust_time_zone_to_epoch_add(int tz, uint32_t epoch);

uint32_t date_time_to_epoch(date_time_t* date_time);

void epoch_to_date_time(date_time_t* date_time, uint32_t epoch);

void calculate_Next_ProcessTime(void);

void startupMessages(void);

void MODBUSConfiguration(UART_HandleTypeDef *huart,MODBUS_config_t pConfig);

void resetMODBUS(void);
void resetMODBUSComm(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);

void beginMODBUS(void);
void beginMODBUSComm(TIM_HandleTypeDef *htim,volatile uint16_t *timeour_counter,volatile uint8_t *rx_bytes,volatile uint8_t *querry_process,int dir_line);


void baudrateSelection(MODBUS_config_t *PORT);

void MODBUSInit(void);

void initqueue(void);

void enqueue(struct queue *,int);

void dequeue(void);
#endif /* INC_APPLICATION_H_ */
