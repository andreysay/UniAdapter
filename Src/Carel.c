/**
  ******************************************************************************
  * File Name          : Carel.c
  * Description        : This file provides code for Send/Receive/Handle functions
  *                      for Carel controllers.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_ll_usart.h"
#include "Command.h"
#include "tim.h"
#include "os.h"
#include "ControllerType.h"
#include "LED.h"
#include "main.h"
#include "StartUp.h"

#ifdef APDEBUG
// Counters for debugging
extern uint32_t Count0;
extern uint32_t Count1;
extern uint32_t Count2;
extern uint32_t Count3;
extern uint32_t Count4;
extern uint32_t Count5;
extern uint32_t Count6;
extern uint32_t Count7;
extern uint32_t Count8;
#endif

// link to main.c file
extern uint8_t ProtocolBuf[];
extern uint32_t ProtocolMsgSize;

// link to Televis.c file
extern int32_t CtrlScanSema;
#ifdef APDEBUG
// Counters for debugging
extern uint32_t CntDeviceFound;
extern uint32_t CntCMDSend;
#endif

// link to ControllerType.c file
extern TEvent Event;
extern TEvent *const ev;
extern bool DeviceFound;

// link to main_threads.c file
extern int32_t PortCtrlRxInitSema; 	// defined in main_threads.c file
extern int32_t PortCtrlTxInitSema;
extern int32_t U1_RxSemaphore; 			// defined in main_threads.c file
extern __IO uint32_t U1_idxTx; 			// defined in main_threads.c file
extern uint8_t *U1_pBufferTransmit; // defined in main_threads.c file
extern uint8_t *U1_pBufferReception; // defined in main_threads.c file
extern __IO uint32_t U1_idxRx; 			// defined in main_threads.c file
extern __IO uint32_t U1_BufferReadyIndication; // defined in main_threads.c file
extern __IO uint8_t U1_RxMessageSize; // defined in main_threads.c file
extern uint8_t U1_TxMessageSize; 		// defined in main_threads.c file
extern int32_t U1_TxSemaphore;
extern int32_t U1_RxSemaphore;
extern int32_t U3_RxInitSema;

extern uint8_t U1_RXBufferA[]; // defined in main_threads.c file
extern uint8_t U1_TXBuffer[]; // defined in main_threads.c file

// link Modbus.c file
extern int32_t ModbusSendSema;

int32_t CarelHndlReceiveSema;
int32_t CarelHndlSendSema;

// Carel indexs symbols
#define carel_header	 	0 // SOF(header) index
#define carel_address		1 // cmd index
#define carel_cmd				2 // sender address index


// Carel indexs symbols
#define STX		0x02 // start of text
#define ETX		0x03 // end of text

//*************message and message fragments**********
const uint8_t CarelScanFrame[] = {
	STX, // Carel SOF header
	0x00, // Receiver address
	0x3F, // Scan command 
	ETX, // 
	ETX, // 
	ETX, // 
};



static void CarelCRC(uint8_t* msg, uint8_t size){
   uint8_t crc = 0;
   for (uint8_t i = 0; i < size - 2; i++){
     crc = crc + msg[i];
   }
   msg[size-2] = 0x30 + ((crc >> 4) & 0x0F);
   msg[size-1] = 0x30 + (crc & 0x0F);

}

//***********CarelPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer.
void CarelPortTxInit(void)
{
#ifdef APDEBUG	
	Count5 = 0;
#endif	
	GPIO_PinState pin_RxA_State;
	GPIO_PinState pin_TxB_State;
	GPIO_PinState pin_TxA_State;
	while(1){
		OS_Wait(&PortCtrlTxInitSema);
		pin_RxA_State = HAL_GPIO_ReadPin(GPIO_RxA, PIN_RxA);
		if(pin_RxA_State != GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIO_RxA, PIN_RxA, GPIO_PIN_SET);
		}
		pin_TxB_State = HAL_GPIO_ReadPin(GPIO_TxB, PIN_TxB);
		if(pin_TxB_State != GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIO_TxB, PIN_TxB, GPIO_PIN_SET);
		}
		pin_TxA_State = HAL_GPIO_ReadPin(GPIO_TxA, PIN_TxA);
		if(pin_TxA_State != GPIO_PIN_RESET){
			HAL_GPIO_WritePin(GPIO_TxA, PIN_TxA, GPIO_PIN_RESET);
		}		
		DisableInterrupts();
		for(uint32_t i = 0; i < U1_TxMessageSize; i++){
			U1_TXBuffer[i] = ProtocolBuf[i];
		}
		// buffer pointer initialization
		U1_pBufferTransmit = &U1_TXBuffer[0];
		EnableInterrupts();
		if(!LL_USART_IsEnabledIT_ERROR(USART1)){
				/* Enable Error interrupt */
			LL_USART_EnableIT_ERROR(USART1);
		}
		OS_Signal(&U1_TxSemaphore);
#ifdef APDEBUG		
		Count5++;
#endif		
	}
}


//***********CarelPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// Send Modbus message to connected controller through USART1,
void CarelPortSendMsg(void){
#ifdef APDEBUG	
	Count6 = 0;
#endif	
	while(1){
		OS_Wait(&U1_TxSemaphore);
		/* Turn ORANGE On at start of transfer : Tx sequence started successfully */
		if(DeviceFound){
			LED_OrangeOn();
		}
		LL_USART_TransmitData8(USART1, U1_TXBuffer[U1_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART1);
		OS_Signal(&PortCtrlRxInitSema); // Signal semaphore to initialize data reception		
#ifdef APDEBUG		
		Count6++;
#endif		
	}
}

//***********CarelPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize index variable, buffer pointer, USART1 interrupts
// signal CtrlPortHandleContinuousReception() for reception
void CarelPortRxInit(void)
{
#ifdef APDEBUG	
	Count7 = 0;
#endif
	while(1){
		OS_Wait(&PortCtrlRxInitSema);			
		/* Initialize index to move on buffer */
		U1_idxRx = 0;
		// buffer pointer initialization
		U1_pBufferReception = &U1_RXBufferA[0];
		/* Enable IDLE */
		LL_USART_EnableIT_IDLE(USART1);
		/* Enable RXNE */
		LL_USART_EnableIT_RXNE(USART1);		
		/* Enable Error interrupt */
		LL_USART_EnableIT_ERROR(USART1);
		OS_Signal(&U1_RxSemaphore);
#ifdef APDEBUG		
		Count7++;
#endif		
	}
}

//***********CarelPortReception***************
// returns none
// Inputs: none
// Outputs: none
// Waiting for data reception from USART1, will signal by USART1 ISR from USART1_IDLE_Callback()
void CarelPortReception(void)
{
#ifdef APDEBUG	
	Count8 = 0;
#endif	
	while(1){
			OS_Wait(&U1_RxSemaphore);
			/* Checks if Buffer full indication has been set */			
			if (U1_BufferReadyIndication != 0)
			{
				DisableInterrupts();
				/* Reset indication */
				U1_BufferReadyIndication = 0;
				ProtocolMsgSize = U1_RxMessageSize;
				for(uint8_t i = 0; i < U1_RxMessageSize; i++){
					ProtocolBuf[i] = U1_RXBufferA[i];
//					U1_RXBufferA[i] = 0;
				}
				LEDs_off();
				EnableInterrupts();
				OS_Signal(&CarelHndlReceiveSema);
			} else {
				if(!DeviceFound){
					OS_Signal(&CtrlScanSema); // signal CarelScan() in case SCAN process
				}
			}
#ifdef APDEBUG			
		Count8++;
#endif			
	}
}

//***********CarelHndlReceived***************
// returns none
// Inputs: none
// Outputs: none
// Handle message by Carel protocol
void CarelHndlReceived(void)
{
  uint32_t crc;
  uint8_t msg_cmd, data_len, *crcptr, msg_len;

	while(1){
		OS_Wait(&CarelHndlReceiveSema);
		msg_len = ProtocolMsgSize;
		data_len = ProtocolMsgSize - 2;
		msg_cmd = ev->ev_cmd;
		ProtocolMsgSize = 0;
		
		if( msg_cmd == CSCAN ){
			if(!DeviceFound){
				DeviceFound = true;
			}
		}
		if(DeviceFound && msg_cmd == CSCAN){
			OS_Signal(&CtrlScanSema); // Signal TelevisScan() in case it in SCAN process
		} else {
			OS_Signal(&ModbusSendSema); // Signal ModbusSend() to transmit message for MU device
		}		

	}		
}

//***********CarelScan***************
// returns none
// Inputs: none
// Outputs: none
// Initialize connected controller address scan by setup Carel protocol scan message and 
// signal CarelPortTxInit() to transmit it to connected controller, if responce will received, controller address was found,
// thread will blocked, MU port reception thread will signaled otherwise will prepare new scan message every 200ms.
void CarelScan(void){
	uint8_t i, msg_len;
	uint32_t TimeDelay = 5000000;
	for(i = 0; i < 6; i++){
		ProtocolBuf[i] = CarelScanFrame[i];
	}
	ProtocolBuf[carel_address] = 0x30;
	LED_GreenOn();
	// Delay for 5 sec in case when controller power on 
	T2TimerDelay(TimeDelay);
	LEDs_off();
	while(1){
		OS_Wait(&CtrlScanSema);
		msg_len = 6;
		if(DeviceFound){
#ifdef APDEBUG			
			CntDeviceFound++;
#endif			
			OS_Signal(&U3_RxInitSema);
		} else {
				// Toggle Green Led indicate that scan operation running
				ToggleLedGreen();
				// Setup address to get responce from controller
				if(ProtocolBuf[carel_address]++ > 207){
					ProtocolBuf[carel_address] = 0x31;
				}
				Event.ev_addr = ProtocolBuf[carel_address];
				Event.ev_cmd = ProtocolBuf[carel_cmd];
				Event.ev_debug = 0;
				Event.dataptr = NULL;
				CarelCRC(ProtocolBuf, msg_len);
				U1_TxMessageSize = msg_len;
				OS_Signal(&PortCtrlTxInitSema);
#ifdef APDEBUG				
				CntCMDSend++;
#endif
				OS_Sleep(200);
		}
	}
}
//-----------------------------------------------------------


/************************ (C) COPYRIGHT *****END OF FILE****/
