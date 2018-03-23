/**
  ******************************************************************************
  * File Name          : Carel.h
  * Description        : This file contains the defines of the threads for Carel controllers
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAREL_H
#define __CAREL_H

/* Private define ------------------------------------------------------------*/
// Structure which holds carel variable index and corresponding modbus index
typedef struct two_indexes {
	uint32_t carel_idx;
	uint32_t modbus_idx;
} two_idx_struct;

typedef struct three_indexes {
	uint16_t enq_idx;
	uint8_t bit_offset_idx;
	uint16_t byte_idx;
} three_idx_struct;

//***********CarelScan***************
// returns none
// Inputs: none
// Outputs: none
// Initialize connected controller address scan by setup Carel protocol scan message and 
// signal CarelEasyPortTxInit() to transmit it to connected controller, if responce will received, controller address was found,
// thread will blocked, MU port reception thread will signaled otherwise will prepare new scan message every 200ms.
void CarelScan(void);
//***********CarelEasyPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer.
void CarelEasyPortTxInit(void);
//***********CarelPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// Send Modbus message to connected controller through USART1,
void CarelPortSendMsg(void);
//***********CarelPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize index variable, buffer pointer, USART1 interrupts
// signal CtrlPortHandleContinuousReception() for reception
void CarelPortRxInit(void);
//***********CarelPortReception***************
// returns none
// Inputs: none
// Outputs: none
// Waiting for data reception from USART1, will signal by USART1 ISR from USART1_IDLE_Callback()
void CarelPortReception(void);
//***********CarelMPXPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer.
void CarelMPXPortTxInit(void);
//***********CarelHndlReceived***************
// returns none
// Inputs: none
// Outputs: none
// Handle message by Carel protocol
void CarelHndlReceived(void);
//***********CarelSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Carel
void CarelSend(void);
//***********CarelMPXSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Carel, send write command directly to Carel device, return preread values for read modbus requests.
void CarelMPXSend(void);
//***********CarelScanHndl***************
// returns none
// Inputs: none
// Outputs: none
// Handle device recponce on address request
void CarelScanHndl(void);
//***********trigCarelMPX***************
// returns none
// Inputs: none
// Outputs: none
// Prepare flags for read Carel adapters
void trigCarelMPX(void);
//***********trigCarelEasy***************
// returns none
// Inputs: none
// Outputs: none
// Prepare flags for read Carel adapters
void trigCarelEasy(void);
//***********readCarelEasy***************
// returns none
// Inputs: none
// Outputs: none
// Prepare command for specific flag and send it to device 
void readCarelEasy(void);
//***********readCarelMPX***************
// returns none
// Inputs: none
// Outputs: none
// Prepare command for specific flag and send it to device 
void readCarelMPX(void);
//***********handleResponceCarelMPX***************
// returns none
// Inputs: none
// Outputs: none
// Handle data received from CarelMPX device
void handleResponceCarelMPX(void);
//***********readCarelCtrlACK***************
// returns none
// Inputs: none
// Outputs: none
// Handle data received from CarelEasy device
void readCarelCtrlACK(void);





#endif /* __CAREL_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
