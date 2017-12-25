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
#include "Command.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
typedef struct TelevisReadRequest
{
  uint8_t t_header;//0x82
  uint8_t t_cmd;//0x92 or 0x12
  uint8_t t_sender;//any, 00 - possible
  uint8_t t_receiver;//fridge controller addr
  uint8_t t_len;//msg len, usually 5
  uint8_t t_regH;//Reg addr Hi
  uint8_t t_regL;//Reg addr Low
  uint8_t t_num;//number bytes to read
  uint32_t t_id;//transaction ident, 00 - possible
  uint32_t t_crc;
}TelevisReadRequest;
typedef struct TelevisReadReply
{
  uint8_t t_header;//0x82
  uint8_t t_cmd;//0x13
  uint8_t t_sender;//Equal to receiver in request
  uint8_t t_receiver;//Equal to sender in request
  uint8_t t_len;
  uint8_t t_idH;//transaction ident, same as in request
  uint8_t t_idL;
  uint8_t t_data;//MSB first
}TelevisReadReply;

typedef struct TelevisWriteRequest
{
  uint8_t t_header;//0x82
  uint8_t t_cmd;//0x93
  uint8_t t_sender;//any, 00 - possible
  uint8_t t_receiver;//fridge controller addr
  uint8_t t_len;//msg len, usually 4
  uint8_t t_regH;//Reg addr
  uint8_t t_regL;
  uint8_t t_data;//Data to write to Reg MSB first
}TelevisWriteRequest;
typedef struct TelevisWriteReply
{
  uint8_t t_header;//0x82
  uint8_t t_cmd;//0x05
  uint8_t t_sender;//Equal to receiver in request
  uint8_t t_receiver;//Equal to sender in request
  uint8_t t_len;//usually 0
}TelevisWriteReply;


/* Private function prototypes -----------------------------------------------*/
void TelevisTIM2TimeInit(void);

void TTimerDelay(uint32_t msTimeDelay);

void TelevisEventThread100ms(void);

void TelevisEventThread1sec(void);

void TelevisPortRxInit(void);

void TelevisPortReception(void);

void TelevisPortTxInit(void);

void TelevisPortSendMsg(void);

void TelevisHndlReceive(void);

void TelevisSend(void);

void TelevisScan(void);

#endif /* __TELEVIS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
