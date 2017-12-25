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
//	uint8_t lenLo;
  uint8_t data;//Data Hi MSB first
//	uint8_t dataLo;
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
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x10
  uint32_t reg;//Reg addr
  uint32_t num;//Regs num to write, usually 0x0001, MSB first
  uint8_t len;//Bytes len
  uint8_t data;//Data to write
}ModbusCmd10;
typedef struct ModbusReply10
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x10
  uint32_t reg;//Reg addr
  uint32_t num;//Regs num written, usually 0x0001, MSB first
}ModbusReply10;

/* Private function prototypes -----------------------------------------------*/

void ModbusPortRxInit(void);

void ModbusPortReception(void);

void ModbusPortTxInit(void);

void ModbusPortSendMsg(void);

void ModbusHndlReceive(void);

void ModbusSend(void);



#endif /* __MODBUS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
