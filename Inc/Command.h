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

/**
	Modbus functions
*/
#define RCOIL		 0x01
#define W1COIL	 0x05
#define CMD03    0x03
#define CMD06    0x06
#define CMD10    0x10
#define CMD43    0x2B
#define CMD73    0x73
#define CMD76    0x76

// Televis
#define SCAN     0xD6
#define DEBUG    0xD6
#define ADDR     0xDD

// Carel protocol definition
#define STX		0x02 // start of text
#define ETX		0x03 // end of text
#define ENQ		0x05 // enquiry
#define DIG_VAR		'D'
#define ANA_VAR		'A'
#define INT_VAR		'I'
#define RDIG_VAR	'B'
#define RANA_VAR	'S'
#define RINT_VAR	'U'
#define FREQ			'F'
#define DREQ		0x3F

#define FARLOOP1 1
#define FARLOOP2 2
#define NEARLOOP 3
#define BROADCAST 254
#define BUFLEN 32
#define REPLYCNTR 10







#endif /* __COMMAND_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
