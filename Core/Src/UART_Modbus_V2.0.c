/*
 * UART_Modbus.c
 *
 *  Created on: Apr 27, 2020
 *      Author: ppingale
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_dma.h"


#define CRC_POLY_16         0xA001
#define CRC_START_MODBUS    0xFFFF

extern uint8_t CycleStart;

union InputRegister UInputReg;
union CoilDtata UCoil;
union HoldReg UHoldReg;

volatile uint8_t Rxbuff[310];
volatile uint16_t Rxptr = 0, Txptr = 0, RxBytes = 0, TxBytes = 0;
volatile uint8_t Txbuff[310];
volatile uint8_t SlaveID = 1;
volatile uint8_t MOD2_SlaveID = 2;
uint8_t TimerStart = 0;

//For RTU MODBUS
volatile uint8_t MOD2_Rxbuff[310];
volatile uint8_t MOD2_Txbuff[310];
volatile uint16_t MOD2_Rxptr = 0, MOD2_Txptr = 0, MOD2_RxBytes = 0, MOD2_TxBytes = 0;
volatile uint16_t DMA_Transaction_no_Tx_uart6 = 0;//RTU
volatile uint16_t DMA_Transaction_no_Tx_uart1 = 0;//HMI

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
uint16_t crc_calc(char* input_str, int len);
void ProcessModbusQuery(void);
void ProcessMOD2_ModbusQuery_DMA(void);

/*External function from Initialization*/
extern void DMA_UART6_RX_Init(void);
extern void DMA_UART6_TX_Init(void);
extern void DMA_UART1_RX_Init(void);
extern void DMA_UART1_TX_Init(void);
//void MX_USART3_UART_Init(void);

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
/*void MX_USART3_UART_Init(void)
{


  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
   // Error_Handler();
  }

 // Rxptr = huart3.pRxBuffPtr;
 // Txptr = huart3.pRxBuffPtr;


}*/

uint16_t crc_calc(char* input_str, int len )
{
    int pos = 0,i = 0;
    uint16_t crc1 = CRC_START_MODBUS;
    for (pos = 0; pos < len; pos++)
    {
        crc1 ^= (uint16_t)input_str[pos];            // XOR byte into least significant byte of CRC
        for (i = 8; i != 0; i--)
        {                                               // Loop over each bit
            if ((crc1 & 0x0001) != 0)
            {                                           // If the LSB is set
                crc1 >>= 1;                              // Shift right and XOR 0xA001
                crc1 ^= CRC_POLY_16;
            }
            else
            {                                           // Else LSB is not set
                crc1 >>= 1;
            }                                           // Just shift right
        }
    }
    return crc1;
}  /* crc_modbus */


// Process Modbus data

//UART transmit
void ProcessModbusQuery(void)
{
	uint16_t Temp1 = 0, Temp2 = 0, startadd = 0, length = 0, Temp = 0;


	//huart3.Instance->CR1 |= UART_IT_TXE;
	if(Rxbuff[0] == SlaveID) // check slave ID
	{
		Temp1 = Rxbuff[7] << 8;
		Temp1 |= Rxbuff[6];
		// calculate CRC of received packet
		Temp2 = crc_calc((char*)Rxbuff, 6);
		if(Temp2 == Temp1) // verify CRC
		{
			TxBytes = 0;
			Txbuff[TxBytes++] = Rxbuff[0]; // slave ID
			Txbuff[TxBytes++] = Rxbuff[1]; // Function code
			startadd = Rxbuff[2] << 8;
			startadd |= Rxbuff[3];
			switch(Rxbuff[1])
			{
				case 0x01: // read coil status
				{
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5]; // number of coils to read
					if((startadd + length) < 45)//26)
					{
						if(((length >> 3) << 3) == length)
							Temp1 = length >> 3; // even byte count;
						else
							Temp1 = (length >> 3) + 1; // odd byte count;
						Txbuff[TxBytes++] = Temp1;

						length = length + startadd;
						for(Temp2 = 0; Temp2 < Temp1; Temp2 ++) //  byte count
						{
							Txbuff[TxBytes] = 0;
							Temp = 0;
							while((startadd < length) && (Temp < 8))
							{
								Txbuff[TxBytes] |= (CoilStatusRegister_t.bytes[startadd++] << Temp);
								Temp++;
							}

							TxBytes++;
						}
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}
#if 0
					if((startadd + length) < 17)
					{
						if(((length >> 3) << 3) == length)
							Temp1 = length >> 3; // even byte count;
						else
							Temp1 = (length >> 3) + 1; // odd byte count;
						Txbuff[TxBytes++] = Temp1;

						length = length + startadd;
						for(Temp2 = 0; Temp2 < Temp1; Temp2 ++) //  byte count
						{
							Txbuff[TxBytes] = 0;
							Temp = 0;
							while((startadd < length) && (Temp < 8))
							{
								Txbuff[TxBytes] |= (UCoil.ByteArr[startadd++] << Temp);
								Temp++;
							}

							TxBytes++;
						}
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}
#endif
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				case 0x02:
				{
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5]; // number of coils to read
					if((startadd + length) < 45)//26)
					{
						if(((length >> 3) << 3) == length)
							Temp1 = length >> 3; // even byte count;
						else
							Temp1 = (length >> 3) + 1; // odd byte count;
						Txbuff[TxBytes++] = Temp1;

						length = length + startadd;
						for(Temp2 = 0; Temp2 < Temp1; Temp2 ++) //  byte count
						{
							Txbuff[TxBytes] = 0;
							Temp = 0;
							while((startadd < length) && (Temp < 8))
							{
								Txbuff[TxBytes] |= (CoilInputRegister_t.bytes[startadd++] << Temp);
								Temp++;
							}

							TxBytes++;
						}
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}
					break;
				}
				case 0x03:// read holding register
				{
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];
					//startadd = startadd - 40000;
					uint16_t index = 0;
					if(startadd < 5000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = HOLDING_REGISTER_ADDRESS_41000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = HOLDING_REGISTER_ADDRESS_42000;
							startadd -= 2000;
						}else if(startadd >= 3000 && startadd < 4000)
						{
							index = HOLDING_REGISTER_ADDRESS_43000;
							startadd -= 3000;
						}else if(startadd >= 4000 && startadd < 5000)
						{
							index = HOLDING_REGISTER_ADDRESS_44000;
							startadd -= 4000;
						}
							startadd = startadd * 2;
							Temp1 = length * 2;
							DMA_Transaction_no_Tx_uart1 = Temp1;//for the DMA
							Txbuff[TxBytes++] = Temp1; // byte count

							for(Temp2 = 0; Temp2 < length; Temp2++)
							{
								Txbuff[TxBytes++] = HoldingRegister_t.bytes[index + startadd + 1];//UHoldReg.ByteArr[startadd + 1]; // High byte
								Txbuff[TxBytes++] = HoldingRegister_t.bytes[index + startadd];//UHoldReg.ByteArr[startadd]; // low byte
								startadd += 2;
							}

							Temp2 = crc_calc((char*)Txbuff, TxBytes);
							Txbuff[TxBytes++] = Temp2;
							Txbuff[TxBytes++] = Temp2 >> 8;
					}
#if 0
					if((startadd + length) < 6)
					{
						startadd = startadd * 2;
						Temp1 = length * 2;
						Txbuff[TxBytes++] = Temp1; // byte count

						for(Temp2 = 0; Temp2 < length; Temp2++)
						{
							Txbuff[TxBytes++] = UHoldReg.ByteArr[startadd + 1]; // High byte
							Txbuff[TxBytes++] = UHoldReg.ByteArr[startadd]; // low byte
							startadd += 2;
						}

						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}
#endif
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				case 0x04:// read input register
				{
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];
					//startadd = startadd - 30000;
					uint16_t index = 0;
					if(startadd < 4000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = INPUT_REGISTER_ADDRESS_31000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = INPUT_REGISTER_ADDRESS_32000;
							startadd -= 2000;
						}
						startadd = startadd * 2;
						Temp1 = length * 2;
						Txbuff[TxBytes++] = Temp1; // byte count
//						if((startadd + length) >= 500)
//						{
//							UInputReg.SInputReg.TemperatureCh5 = ReadADC(CH5);
//							UInputReg.SInputReg.WDetectCh6 = ReadADC(CH6);
//							UInputReg.SInputReg.TestCh7 = ReadADC(CH7);
//						}
						for(Temp2 = 0; Temp2 < length; Temp2++)
						{
							Txbuff[TxBytes++] = InputRegister_t.bytes[index + startadd + 1];//UInputReg.ByteArr[index + startadd + 1]; // High byte
							Txbuff[TxBytes++] = InputRegister_t.bytes[index + startadd];//UInputReg.ByteArr[index + startadd]; // low byte
							startadd += 2;
						}

						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}
#if 0
					if((startadd + length) < 504)
					{
						startadd = startadd * 2;
						Temp1 = length * 2;
						Txbuff[TxBytes++] = Temp1; // byte count
						if((startadd + length) >= 500)
						{
							UInputReg.SInputReg.TemperatureCh5 = ReadADC(CH5);
							UInputReg.SInputReg.WDetectCh6 = ReadADC(CH6);
							UInputReg.SInputReg.TestCh7 = ReadADC(CH7);
						}
						for(Temp2 = 0; Temp2 < length; Temp2++)
						{
							Txbuff[TxBytes++] = UInputReg.ByteArr[startadd + 1]; // High byte
							Txbuff[TxBytes++] = UInputReg.ByteArr[startadd]; // low byte
							startadd += 2;
						}

						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;

					}
#endif
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}


				  break;
				}
				case 0x05:// Force single coil
				{
					if(startadd < 16)
					{
						CoilStatusRegister_t.bytes[startadd] = Rxbuff[4] & 0x01;
						for(Temp1 = 2; Temp1 < RxBytes; Temp1++) // echo response
							Txbuff[TxBytes++] = Rxbuff[Temp1];
					}
#if 0
					if(startadd < 16)
					{
						UCoil.ByteArr[startadd] = Rxbuff[4] & 0x01;
						for(Temp1 = 2; Temp1 < RxBytes; Temp1++) // echo response
							Txbuff[TxBytes++] = Rxbuff[Temp1];
					}
#endif
					else
					{
						// error response
						Txbuff[1] |= 0x80;
						for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}

				  break;
				}
				case 0x06:// Preset single register(holding)
				{
					//startadd = startadd - 40000;
					uint16_t index = 0;
					if(startadd < 5000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = HOLDING_REGISTER_ADDRESS_41000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = HOLDING_REGISTER_ADDRESS_42000;
							startadd -= 2000;
						}else if(startadd >= 3000 && startadd < 4000)
						{
							index = HOLDING_REGISTER_ADDRESS_43000;
							startadd -= 3000;
						}else if(startadd >= 4000 && startadd < 5000)
						{
							index = HOLDING_REGISTER_ADDRESS_44000;
							startadd -= 4000;
						}

						startadd = startadd << 1;
						HoldingRegister_t.bytes[index + startadd++] = Rxbuff[5];
						HoldingRegister_t.bytes[index + startadd] = Rxbuff[4];
						for(Temp1 = 2; Temp1 < RxBytes; Temp1++) // echo response
							Txbuff[TxBytes++] = Rxbuff[Temp1];
						if(UHoldReg.SHoldReg.ADCReadStart == 0x01)
						{
							if(CycleStart == 0)
							{
								TimerStart = 0x01;
							}
						}

					}
#if 0
					if(startadd < 5)
					{
						startadd = startadd << 1;
						UHoldReg.ByteArr[startadd++] = Rxbuff[5];
						UHoldReg.ByteArr[startadd] = Rxbuff[4];
						for(Temp1 = 2; Temp1 < RxBytes; Temp1++) // echo response
							Txbuff[TxBytes++] = Rxbuff[Temp1];
						if(UHoldReg.SHoldReg.ADCReadStart == 0x01)
						{
							if(CycleStart == 0)
							{
								TimerStart = 0x01;
							}
						}
							//TimerStart = 0x01;
					}
#endif
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				case 0x0F:// Force multiple coils
				{
					Temp = 6;
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];
					if((startadd + length) < 45)//17) //number of coils
					{
						for(Temp1 = 0; Temp1 < Rxbuff[6]; Temp1++) // byte count
						{
							Temp2 = 0;
							Temp++;
#if 0
							while(Temp2 < 8)
							{
								UCoil.ByteArr[startadd++] = (Rxbuff[Temp] >> Temp2) & 0x01;
								Temp2++;
							}
#endif
							while(Temp2 < 8)
							{
								CoilStatusRegister_t.bytes[startadd++] = (Rxbuff[Temp] >> Temp2) & 0x01;
								Temp2++;
							}
						}
						for(Temp1 = 2; Temp1 < 6; Temp1++) // response
						Txbuff[TxBytes++] = Rxbuff[Temp1];
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;

#if 0
						//Test code start 04.03.2021 Sagar
							if(UCoil.ByteArr[0] == 1) 	//P5
							{
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
							}

							if(UCoil.ByteArr[1] == 1)	//P6
							{
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
							}

							if(UCoil.ByteArr[2] == 1)	//P7
							{
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
							}

							if(UCoil.ByteArr[3] == 1)	//P8
							{
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
							}


							if(UCoil.ByteArr[4] == 1)	//P9   if(UCoil.ByteArr[1] == 1)	//P6   //temperer //
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[5] == 1)	//P10
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[6] == 1)	//P11
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[7] == 1)	//P12
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
							}



							if(UCoil.ByteArr[8] == 1)	//RLY1 (P31)
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[9] == 1)	//RLY2 (P32)
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[10] == 1)	//RLY3 (P33)
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[11] == 1)	//RLY4 (P34)
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[12] == 1)	//RLY5 (P35)
							{
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[13] == 1)	//RLY6 (P36)
							{
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[14] == 1)	//RLY7 (P29)
							{
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
							}
							if(UCoil.ByteArr[15] == 1)	//RLY8 (P30)
							{
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
							}else
							{
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
							}
#endif
							Temp1 = 0;
						//Test code end 04.03.2021 Sagar
					}
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}
				  break;
				}
				case 0x10:// Preset multiple registers
				{
					//startadd = startadd - 40000;
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];

					uint16_t index = 0;
					if(startadd < 5000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = HOLDING_REGISTER_ADDRESS_41000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = HOLDING_REGISTER_ADDRESS_42000;
							startadd -= 2000;
						}else if(startadd >= 3000 && startadd < 4000)
						{
							index = HOLDING_REGISTER_ADDRESS_43000;
							startadd -= 3000;
						}else if(startadd >= 4000 && startadd < 5000)
						{
							index = HOLDING_REGISTER_ADDRESS_44000;
							startadd -= 4000;
						}

						startadd = startadd << 1;
						Temp2 = Rxbuff[6] + 7;
						Temp1 = 7;
						while( Temp1 < Temp2 ) // byte count
						{
							HoldingRegister_t.bytes[index + startadd++] = Rxbuff[Temp1 + 1]; // low byte
							HoldingRegister_t.bytes[index + startadd++] = Rxbuff[Temp1]; // high byte
							Temp1 += 2;
						}

						for(Temp1 = 2; Temp1 < 6; Temp1++)
							Txbuff[TxBytes++] = Rxbuff[Temp1];
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;

						if(UHoldReg.SHoldReg.ADCReadStart == 0x01)
						{
							if(CycleStart == 0)
							{
								TimerStart = 0x01;
							}
						}
							//TimerStart = 0x01;
					}



#if 0
					if((startadd + length) < 6)
					{
						startadd = startadd << 1;
						Temp2 = Rxbuff[6] + 7;
						Temp1 = 7;
						while( Temp1 < Temp2 ) // byte count
						{
							UHoldReg.ByteArr[startadd++] = Rxbuff[Temp1 + 1]; // low byte
							UHoldReg.ByteArr[startadd++] = Rxbuff[Temp1]; // high byte
							Temp1 += 2;
						}

						for(Temp1 = 2; Temp1 < 6; Temp1++)
							Txbuff[TxBytes++] = Rxbuff[Temp1];
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;

						if(UHoldReg.SHoldReg.ADCReadStart == 0x01)
						{
							if(CycleStart == 0)
							{
								TimerStart = 0x01;
							}
						}
							//TimerStart = 0x01;
					}
#endif
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				default:
					break;
			}

			for(Temp1 = 0; Temp1 < 310; Temp1++)
				Rxbuff[Temp1] = 0;
			Txptr = 0;
//			huart3.Instance->CR1 |= UART_MODE_TX;// Tx mode enable
//			huart3.Instance->CR1 |= UART_IT_TXE; // tx empty interrupt enable

			//huart1.Instance->CR1 |= UART_MODE_TX;// Tx mode enable
			//huart1.Instance->CR1 |= UART_IT_TXE; // tx empty interrupt enable
			//Enable UART 6 DMAT
			huart1.Instance->CR3 &= !USART_CR3_DMAR;
			huart1.Instance->CR3 |= USART_CR3_DMAT;
			//Disable the interrupt for UART RX
			//DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,DISABLE);

			/*Configure the DMA 2 Stream 6 and enable the IRQ*/
			DMA_UART1_TX_Init();

			/*Enable the DMA2 Stream 7*/
			DMAPeripheralEnable(DMA2_Stream7, ENABLE);

			/*Direction pin to HIGH for Tx*/
			ADM_CLTR_HIGH();

		}
		else
		{
//			huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
//			huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

			//huart1.Instance->CR1 |= UART_MODE_RX; // Rx enable
			//huart1.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
			huart1.Instance->CR3 |= USART_CR3_DMAR;
			huart1.Instance->CR3 &= !USART_CR3_DMAT;
			//Enable the interrupt for UART TX
			DMA_IRQITConfig(IRQ_NO_DMA2_Stream7,DISABLE);
			//Disable the interrupt for UART RX
			DMA_IRQITConfig(IRQ_NO_DMA2_Stream5,ENABLE);

			//Enable the DMA 2 Stream for UART 6 RX
			DMAPeripheralEnable(DMA2_Stream5, ENABLE);
			ADM_CLTR_LOW();
		}
	}
	else // invalid slave ID
	{
//		huart3.Instance->CR1 |= UART_MODE_RX; // Rx enable
//		huart3.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable

		//huart1.Instance->CR1 |= UART_MODE_RX; // Rx enable
		//huart1.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
		huart1.Instance->CR3 |= USART_CR3_DMAR;
		huart1.Instance->CR3 &= !USART_CR3_DMAT;
		//Enable the interrupt for UART TX
		DMA_IRQITConfig(IRQ_NO_DMA2_Stream7,DISABLE);
		//Disable the interrupt for UART RX
		DMA_IRQITConfig(IRQ_NO_DMA2_Stream5,ENABLE);

		//Enable the DMA 2 Stream for UART 6 RX
		DMAPeripheralEnable(DMA2_Stream5, ENABLE);
		ADM_CLTR_LOW();
	}




}

void ProcessMOD2_ModbusQuery_DMA(void)
{
	uint16_t Temp1 = 0, Temp2 = 0, startadd = 0, length = 0, Temp = 0;


	//huart3.Instance->CR1 |= UART_IT_TXE;
	if(MOD2_Rxbuff[0] == MOD2_SlaveID) // check slave ID
	{
		Temp1 = MOD2_Rxbuff[7] << 8;
		Temp1 |= MOD2_Rxbuff[6];
		// calculate CRC of received packet
		Temp2 = crc_calc((char*)MOD2_Rxbuff, 6);
		if(Temp2 == Temp1) // verify CRC
		{
			MOD2_TxBytes = 0;
			MOD2_Txbuff[MOD2_TxBytes++] = MOD2_Rxbuff[0]; // slave ID
			MOD2_Txbuff[MOD2_TxBytes++] = MOD2_Rxbuff[1]; // Function code
			startadd = MOD2_Rxbuff[2] << 8;
			startadd |= MOD2_Rxbuff[3];
			switch(MOD2_Rxbuff[1])
			{
				case 0x01: // read coil status
				{
					length = MOD2_Rxbuff[4] << 8;
					length |= MOD2_Rxbuff[5]; // number of coils to read
					if((startadd + length) < 45)//26)
					{
						if(((length >> 3) << 3) == length)
							Temp1 = length >> 3; // even byte count;
						else
							Temp1 = (length >> 3) + 1; // odd byte count;
						Txbuff[TxBytes++] = Temp1;

						length = length + startadd;
						for(Temp2 = 0; Temp2 < Temp1; Temp2 ++) //  byte count
						{
							Txbuff[TxBytes] = 0;
							Temp = 0;
							while((startadd < length) && (Temp < 8))
							{
								Txbuff[TxBytes] |= (CoilStatusRegister_t.bytes[startadd++] << Temp);
								Temp++;
							}

							TxBytes++;
						}
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				case 0x02:
				{
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5]; // number of coils to read
					if((startadd + length) < 45)//26)
					{
						if(((length >> 3) << 3) == length)
							Temp1 = length >> 3; // even byte count;
						else
							Temp1 = (length >> 3) + 1; // odd byte count;
						Txbuff[TxBytes++] = Temp1;

						length = length + startadd;
						for(Temp2 = 0; Temp2 < Temp1; Temp2 ++) //  byte count
						{
							Txbuff[TxBytes] = 0;
							Temp = 0;
							while((startadd < length) && (Temp < 8))
							{
								Txbuff[TxBytes] |= (CoilInputRegister_t.bytes[startadd++] << Temp);
								Temp++;
							}

							TxBytes++;
						}
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}
					break;
				}
				case 0x03:// read holding register
				{
					length = MOD2_Rxbuff[4] << 8;
					length |= MOD2_Rxbuff[5];
					//startadd = startadd - 40000;
					uint16_t index = 0;
					if(startadd < 5000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = HOLDING_REGISTER_ADDRESS_41000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = HOLDING_REGISTER_ADDRESS_42000;
							startadd -= 2000;
						}else if(startadd >= 3000 && startadd < 4000)
						{
							index = HOLDING_REGISTER_ADDRESS_43000;
							startadd -= 3000;
						}else if(startadd >= 4000 && startadd < 5000)
						{
							index = HOLDING_REGISTER_ADDRESS_44000;
							startadd -= 4000;
						}
							startadd = startadd * 2;
							Temp1 = length * 2;//Set this as global variable
							DMA_Transaction_no_Tx_uart6 = Temp1;
							MOD2_Txbuff[MOD2_TxBytes++] = Temp1; // byte count

							for(Temp2 = 0; Temp2 < length; Temp2++)
							{
								MOD2_Txbuff[MOD2_TxBytes++] = UHolding_Modbus_2.bytes[index + startadd + 1];//UHoldReg.ByteArr[startadd + 1]; // High byte
								MOD2_Txbuff[MOD2_TxBytes++] = UHolding_Modbus_2.bytes[index + startadd];//UHoldReg.ByteArr[startadd]; // low byte
								startadd += 2;
							}

							Temp2 = crc_calc((char*)MOD2_Txbuff, MOD2_TxBytes);
							MOD2_Txbuff[MOD2_TxBytes++] = Temp2;
							MOD2_Txbuff[MOD2_TxBytes++] = Temp2 >> 8;
					}
					else
					{
						// error
						MOD2_Txbuff[1] |= 0x80;
						MOD2_Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)MOD2_Txbuff, 3);
						MOD2_Txbuff[3] = Temp2;
						MOD2_Txbuff[4] = Temp2 >> 8;
						MOD2_TxBytes = 5;
					}

				  break;
				}
				case 0x04:// read input register
				{
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];
					//startadd = startadd - 30000;
					uint16_t index = 0;
					if(startadd < 4000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = INPUT_REGISTER_ADDRESS_31000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = INPUT_REGISTER_ADDRESS_32000;
							startadd -= 2000;
						}
						startadd = startadd * 2;
						Temp1 = length * 2;
						Txbuff[TxBytes++] = Temp1; // byte count
//						if((startadd + length) >= 500)
//						{
//							UInputReg.SInputReg.TemperatureCh5 = ReadADC(CH5);
//							UInputReg.SInputReg.WDetectCh6 = ReadADC(CH6);
//							UInputReg.SInputReg.TestCh7 = ReadADC(CH7);
//						}
						for(Temp2 = 0; Temp2 < length; Temp2++)
						{
							Txbuff[TxBytes++] = InputRegister_t.bytes[index + startadd + 1];//UInputReg.ByteArr[index + startadd + 1]; // High byte
							Txbuff[TxBytes++] = InputRegister_t.bytes[index + startadd];//UInputReg.ByteArr[index + startadd]; // low byte
							startadd += 2;
						}

						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}


				  break;
				}
				case 0x05:// Force single coil
				{
					if(startadd < 16)
					{
						CoilStatusRegister_t.bytes[startadd] = Rxbuff[4] & 0x01;
						for(Temp1 = 2; Temp1 < RxBytes; Temp1++) // echo response
							Txbuff[TxBytes++] = Rxbuff[Temp1];
					}
					else
					{
						// error response
						Txbuff[1] |= 0x80;
						for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
					}

				  break;
				}
				case 0x06:// Preset single register(holding)
				{
					//startadd = startadd - 40000;
					uint16_t index = 0;
					if(startadd < 5000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = HOLDING_REGISTER_ADDRESS_41000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = HOLDING_REGISTER_ADDRESS_42000;
							startadd -= 2000;
						}else if(startadd >= 3000 && startadd < 4000)
						{
							index = HOLDING_REGISTER_ADDRESS_43000;
							startadd -= 3000;
						}else if(startadd >= 4000 && startadd < 5000)
						{
							index = HOLDING_REGISTER_ADDRESS_44000;
							startadd -= 4000;
						}

						startadd = startadd << 1;
						HoldingRegister_t.bytes[index + startadd++] = Rxbuff[5];
						HoldingRegister_t.bytes[index + startadd] = Rxbuff[4];
						for(Temp1 = 2; Temp1 < RxBytes; Temp1++) // echo response
							Txbuff[TxBytes++] = Rxbuff[Temp1];
						if(UHoldReg.SHoldReg.ADCReadStart == 0x01)
						{
							if(CycleStart == 0)
							{
								TimerStart = 0x01;
							}
						}

					}
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				case 0x0F:// Force multiple coils
				{
					Temp = 6;
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];
					if((startadd + length) < 45)//17) //number of coils
					{
						for(Temp1 = 0; Temp1 < Rxbuff[6]; Temp1++) // byte count
						{
							Temp2 = 0;
							Temp++;
							while(Temp2 < 8)
							{
								CoilStatusRegister_t.bytes[startadd++] = (Rxbuff[Temp] >> Temp2) & 0x01;
								Temp2++;
							}
						}
						for(Temp1 = 2; Temp1 < 6; Temp1++) // response
						Txbuff[TxBytes++] = Rxbuff[Temp1];
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;
							Temp1 = 0;
						//Test code end 04.03.2021 Sagar
					}
					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}
				  break;
				}
				case 0x10:// Preset multiple registers
				{
					//startadd = startadd - 40000;
					length = Rxbuff[4] << 8;
					length |= Rxbuff[5];

					uint16_t index = 0;
					if(startadd < 5000)
					{
						if(startadd < 1000)
						{
							index = 0;
						}else if(startadd >= 1000 && startadd < 2000)
						{
							index = HOLDING_REGISTER_ADDRESS_41000;
							startadd -= 1000;
						}else if(startadd >= 2000 && startadd < 3000)
						{
							index = HOLDING_REGISTER_ADDRESS_42000;
							startadd -= 2000;
						}else if(startadd >= 3000 && startadd < 4000)
						{
							index = HOLDING_REGISTER_ADDRESS_43000;
							startadd -= 3000;
						}else if(startadd >= 4000 && startadd < 5000)
						{
							index = HOLDING_REGISTER_ADDRESS_44000;
							startadd -= 4000;
						}

						startadd = startadd << 1;
						Temp2 = Rxbuff[6] + 7;
						Temp1 = 7;
						while( Temp1 < Temp2 ) // byte count
						{
							HoldingRegister_t.bytes[index + startadd++] = Rxbuff[Temp1 + 1]; // low byte
							HoldingRegister_t.bytes[index + startadd++] = Rxbuff[Temp1]; // high byte
							Temp1 += 2;
						}

						for(Temp1 = 2; Temp1 < 6; Temp1++)
							Txbuff[TxBytes++] = Rxbuff[Temp1];
						Temp2 = crc_calc((char*)Txbuff, TxBytes);
						Txbuff[TxBytes++] = Temp2;
						Txbuff[TxBytes++] = Temp2 >> 8;

						if(UHoldReg.SHoldReg.ADCReadStart == 0x01)
						{
							if(CycleStart == 0)
							{
								TimerStart = 0x01;
							}
						}
							//TimerStart = 0x01;
					}

					else
					{
						// error
						Txbuff[1] |= 0x80;
						Txbuff[2] = 0x02;
						//for(Temp1 = 2; Temp1 < RxBytes - 2; Temp1++) // echo response
						//Txbuff[TxBytes++] = Rxbuff[Temp1];

						Temp2 = crc_calc((char*)Txbuff, 3);
						Txbuff[3] = Temp2;
						Txbuff[4] = Temp2 >> 8;
						TxBytes = 5;
					}

				  break;
				}
				default:
					break;
			}

			for(Temp1 = 0; Temp1 < 310; Temp1++)
				MOD2_Rxbuff[Temp1] = 0;
			MOD2_Txptr = 0;
			//huart6.Instance->CR1 |= UART_MODE_TX;// Tx mode enable
			//huart6.Instance->CR1 |= UART_IT_TXE; // tx empty interrupt enable
			//Enable UART 6 DMAT
			huart6.Instance->CR3 &= !USART_CR3_DMAR;
			huart6.Instance->CR3 |= USART_CR3_DMAT;
			//Disable the interrupt for UART RX
			//DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,DISABLE);

			/*Configure the DMA 2 Stream 6 and enable the IRQ*/
			DMA_UART6_TX_Init();

			/*Enable the DMA2 Stream 6*/
			DMAPeripheralEnable(DMA2_Stream6, ENABLE);

			/*Direction pin to HIGH for Tx*/
			ADM_2_CLTR_HIGH();

		}
		else
		{
			//huart6.Instance->CR1 |= UART_MODE_RX; // Rx enable
			//huart6.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
			huart6.Instance->CR3 |= USART_CR3_DMAR;
			huart6.Instance->CR3 &= !USART_CR3_DMAT;
			//Enable the interrupt for UART TX
			DMA_IRQITConfig(IRQ_NO_DMA2_Stream6,DISABLE);
			//Disable the interrupt for UART RX
			DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,ENABLE);

			//Enable the DMA 2 Stream for UART 6 RX
			DMAPeripheralEnable(DMA2_Stream2, ENABLE);
			ADM_2_CLTR_LOW();
		}
	}
	else // invalid slave ID
	{
		//huart1.Instance->CR1 |= UART_MODE_RX; // Rx enable
		//huart6.Instance->CR1 |= UART_IT_RXNE; // UART RX not empty interrupt enable
		huart6.Instance->CR3 |= USART_CR3_DMAR;
		huart6.Instance->CR3 &= !USART_CR3_DMAT;
		//Enable the interrupt for UART TX
		DMA_IRQITConfig(IRQ_NO_DMA2_Stream6,DISABLE);
		//Disable the interrupt for UART RX
		DMA_IRQITConfig(IRQ_NO_DMA2_Stream2,ENABLE);
		//Enable the DMA 2 Stream for UART 6 RX
		DMAPeripheralEnable(DMA2_Stream2, ENABLE);
		ADM_2_CLTR_LOW();
	}
}
