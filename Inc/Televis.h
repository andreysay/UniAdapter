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
#include "tim.h"
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

//***********TelevisPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize index variable, buffer pointer, USART1 interrupts
// signal TelevisPortReception() for reception
void TelevisPortRxInit(void);
//***********TelevisPortReception***************
// returns none
// Inputs: none
// Outputs: none
// Waiting for data reception from USART1, will signal by USART1 ISR from USART1_IDLE_Callback()n
void TelevisPortReception(void);
//***********TelevisPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer,
// USART1 with ODD parity
void TelevisPortTxInit(void);
//***********TelevisPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// Send Televis message to connected controller through USART1,
// first byte will send with ODD parity, after delay 100 microseconds will send other part of message with EVEN parity
void TelevisPortSendMsg(void);
//***********TelevisHndlReceive***************
// returns none
// Inputs: none
// Outputs: none
// Handle message from controlled by Televis protocol
void TelevisHndlReceive(void);
//***********TelevisSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Televis, signal TelevisPortTxInit() to transmit
void TelevisSend(void);
//***********TelevisScan***************
// returns none
// Inputs: none
// Outputs: none
// Initialize connected controller address scan by setup Televis scan message and 
// signal TelevisPortTxInit() to transmit it to connected controller, if responce will received, controller address was found,
// thread will blocked, MU port reception thread will signaled otherwise will prepare new scan message every 200ms.
void TelevisScan(void);

#endif /* __TELEVIS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
