/**
  ******************************************************************************
  * File Name          : Modbus.h
  * Description        : This file contains the defines of the Modbus protocol
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_H
#define __MODBUS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "Command.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

//----------------------------------------------
typedef struct ModbusCmd //Read Holding Regs
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x03
  uint8_t regHi;//Reg Hi number addr
	uint8_t regLo;//Reg Lo number addr
  uint8_t regNumH;//Data Hi number
	uint8_t regNumL;//Data Lo number
}ModbusCmd;
typedef struct ModbusReply03
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x03
  uint8_t len;//Bytes len
  uint8_t data;//Data Hi MSB first
}ModbusReply03;

typedef struct ModbusCmd06 //Write Reg
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x06
  uint32_t reg;//Reg addr
  uint8_t data;//Data to write
}ModbusCmd06;
typedef struct ModbusReply06 //
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x06
  uint32_t reg;//Reg addr
  uint8_t data;//Data written
}ModbusReply06;

typedef struct ModbusCmd10 //Write Regs
{
  uint8_t addr;		//Device addr
  uint8_t cmd;		//Cmd=0x10
  uint8_t regHi; 	//Reg MSB byte addr
	uint8_t regLo; 	//Reg LSB byte addr
  uint8_t numHi;	//Regs num to write, usually 0x0001, MSB first
	uint8_t numLo;
  uint8_t numByts;		//	Bytes len
  uint8_t data;//Data to write
}ModbusCmd10;
typedef struct ModbusReply10
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x10
  uint8_t regHi; //Reg MSB byte addr
	uint8_t regLo; //Reg LSB byte addr
  uint8_t numHi;//Regs num written, usually 0x0001, MSB first
	uint8_t numLo;
}ModbusReply10;

/* Private function prototypes -----------------------------------------------*/
//***********ModbusPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize MU port for data reception, will signal from TelevisScan() after connected controller address found
void ModbusPortRxInit(void);
//***********ModbusPortReception***************
// returns none
// Inputs: none
// Outputs: none
// Wait for data reception from MU trough USART3, will signal from USART3 ISR trough USART3_IDLE_Callback()
void ModbusPortReception(void);
//***********ModbusPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize variables and fill out buffers for MU port transmittion
void ModbusPortTxInit(void);
//***********ModbusPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// transmit data to MU device through USART3
void ModbusPortSendMsg(void);
//***********ModbusHndlReceive***************
// returns none
// Inputs: none
// Outputs: none
// Handle received from MU port Modbus message, will signal from ModbusPortReception()
void ModbusHndlReceive(void);
//***********ModbusSend***************
// returns none
// Inputs: none
// Outputs: none
// Prepare Modbus message to send, thread will signal from TelevisHndlReceive()
void ModbusSend(void);
#endif /* __MODBUS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
