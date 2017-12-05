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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define CMD03    0x03
#define CMD06    0x06
#define CMD10    0x10
#define CMD43    0x2B
#define CMD73    0x73
#define CMD76    0x76
#define SCAN     0xD6
#define DEBUG    0xD6
#define ADDR     0xDD
#define FARLOOP1 1
#define FARLOOP2 2
#define NEARLOOP 3
#define BROADCAST 254
#define BUFLEN 32
#define REPLYCNTR 10

//----------------------------------------------
typedef struct ModbusCmd //Read Holding Regs
{
  uint8_t addr;//Device addr
  uint8_t cmd;//Cmd=0x03
  uint32_t reg;//Reg addr
  uint32_t num;//Regs num
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

bool ModbusReceive(uint8_t *data);

bool TModbusReceive(TEvent *ev);
#endif /* __MODBUS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
