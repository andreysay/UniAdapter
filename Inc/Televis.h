/**
  ******************************************************************************
  * File Name          : Televis.h
  * Description        : This file contains the defines of the Televis protocol
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELEVIS_H
#define __TELEVIS_H

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

//-----------------------------------------------
typedef struct TelevisReadRequest
{
  uint8_t header;//0x82
  uint8_t cmd;//0x92 or 0x12
  uint8_t sender;//any, 00 - possible
  uint8_t receiver;//fridge controller addr
  uint8_t len;//msg len, usually 5
  uint8_t regH;//Reg addr Hi
  uint8_t regL;//Reg addr Low
  uint8_t num;//number bytes to read
  uint32_t id;//transaction ident, 00 - possible
  uint32_t crc;
}TelevisReadRequest;
typedef struct TelevisReadReply
{
  uint8_t header;//0x82
  uint8_t cmd;//0x13
  uint8_t sender;//Equal to receiver in request
  uint8_t receiver;//Equal to sender in request
  uint8_t len;
  uint8_t idH;//transaction ident, same as in request
  uint8_t idL;
  uint8_t data;//MSB first
}TelevisReadReply;

typedef struct TelevisWriteRequest
{
  uint8_t header;//0x82
  uint8_t cmd;//0x93
  uint8_t sender;//any, 00 - possible
  uint8_t receiver;//fridge controller addr
  uint8_t len;//msg len, usually 4
  uint8_t regH;//Reg addr
  uint8_t regL;
  uint8_t data;//Data to write to Reg MSB first
}TelevisWriteRequest;
typedef struct TelevisWriteReply
{
  uint8_t header;//0x82
  uint8_t cmd;//0x05
  uint8_t sender;//Equal to receiver in request
  uint8_t receiver;//Equal to sender in request
  uint8_t len;//usually 0
}TelevisWriteReply;

typedef struct TEvent
{
  uint8_t addr;
  uint8_t cmd;
  uint32_t reg;
  uint8_t *dataptr;
  uint8_t len;
  uint8_t debug;
  uint8_t size;
} TEvent;

bool TelevisReceive(TEvent *ev);

void TelevisSend(TEvent *ev);

#endif /* __TELEVIS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
