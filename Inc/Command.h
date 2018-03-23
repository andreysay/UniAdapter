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
#define ACK		0x06 // acknowlege
#define DIG_VAR		'D' //0x44
#define ANA_VAR		'A' //0x41
#define INT_VAR		'I' //0x49
#define RDIG_VAR	'B' //0x42
#define RANA_VAR	'S' //0x53
#define RINT_VAR	'U' //0x55
#define FREQ			'F' //0x46
#define RX_VAR		'x' //0x78
#define RV_VAR		'V' //0x56
#define RR_VAR		'R' //0x52
#define RT_VAR		'T' //0x54
#define HW_VAR		'=' //0x3D
#define DREQ			'?' //0x3F
#define TREQ			'T' //0x54
#define RREQ			'R' //0x52
#define	WREQ			'W' //0x57


#define HWidREG 0xFFF2

#define FARLOOP1 1
#define FARLOOP2 2
#define NEARLOOP 3
#define BROADCAST 254
#define BUFLEN 32
#define REPLYCNTR 10







#endif /* __COMMAND_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
