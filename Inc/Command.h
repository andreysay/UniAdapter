/**
  ******************************************************************************
  * File Name          : Command.h
  * Description        : This file contains the defines of the Command for Modbus/Televis protocol
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMAND_H
#define __COMMAND_H

/* Private define ------------------------------------------------------------*/

// Modbus/Televis protocol commands
#define CMD03    0x03
#define CMD06    0x06
#define CMD10    0x10
#define CMD43    0x2B
#define CMD73    0x73
#define CMD76    0x76
#define SCAN     0xD6
#define DEBUG    0xD6
#define ADDR     0xDD

// Carel protocol commands
#define CSCAN		0x3F

#define FARLOOP1 1
#define FARLOOP2 2
#define NEARLOOP 3
#define BROADCAST 254
#define BUFLEN 32
#define REPLYCNTR 10







#endif /* __COMMAND_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
