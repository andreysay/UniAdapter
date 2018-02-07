/**
  ******************************************************************************
  * File Name          : Televis.c
  * Description        : This file provides code for Send/Receive functions
  *                      of the Televis protocol.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"
#include "main.h"
#include "ControllerType.h"
#include "main_threads.h"
#include "os.h"
#include "StartUp.h"
#include "ErrorHandler.h"
#include "LED.h"
#include "Televis.h"

// Televis indexs symbols
#define header 		0 // SOF(header) index
#define cmd				1 // cmd index
#define sender		2 // sender address index
#define receiver	3 // receiver address index
#define len				4 // lenght of data for transmition index
#define regH			5 // register address value index MSB
#define regL			6 // register address value index LSB
#define data			7 // lenght of data for responce index
#define id				8 // id transaction index

#define SOF 0x82		// Televis start of frame(SOF (header))

/* Private variables ---------------------------------------------------------*/
#ifdef APDEBUG
uint32_t DebugTelevisBuf[MAX_BUFFER_SIZE]; // Buffer for debugging purpose
// Counters for debugging
uint32_t CntDeviceFound, CntCMDSend, CountHandleReceive, ErrorCMDTelevislog, ErrorCMD03_073Televislog, ErrorCMD06_076Televislog;
extern uint32_t Count3;
extern uint32_t Count4;
extern uint32_t Count5;
extern uint32_t Count6;
extern uint32_t Count7;
extern uint32_t Count8;
#endif

/* Initial autoreload value */
extern __IO uint8_t TIM2_InterruptFlag;



//*************message and message fragments**********
const uint8_t TelevisScanCMD[] = {
	SOF, // Televis SOF header
	0xD6, // Scan command 
	0x00, // Sender address
	0x00, // Receiver address
	0x01, // Message lenght
	0x02, // Register MSB
};

// link to main.c file
extern uint8_t ProtocolBuf[];

// link to main_threads.c file
extern int32_t PortCtrlRxInitSema; 	// defined in main_threads.c file
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

// link to main.c file
extern int32_t Time100msSemaphore;
extern int32_t Time1secSemaphore;

// link to Modbus.c file
extern int32_t ModbusSendSema;

// link to ControllerType.c
extern TEvent Event;
extern TEvent *const ev;
extern bool DeviceFound;

// Semaphore will use for TelevisHndlReceive() thread
int32_t TelevisHndlReceiveSema;
// Semaphore will use in TelevisSend() thread
int32_t TelevisHndlSendSema;
// Semaphore will use in TelevisPortTxInit() thread
int32_t TelevisPortTxInitSema;
// Semaphore will use in TelevisPortRxInit() thread
int32_t TelevisPortRxInitSema;
// Semaphore will use in TelevisScan() thread
int32_t CtrlScanSema;

// Variable to store Televis message size
uint8_t TelevisMessageSize;

//***********TelevisPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize index variable, buffer pointer, USART1 interrupts
// signal TelevisPortReception() for reception
void TelevisPortRxInit(void)
{
#ifdef APDEBUG	
	Count7 = 0;
#endif	
	while(1){
		OS_Wait(&TelevisPortRxInitSema);
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

//***********TelevisPortReception***************
// returns none
// Inputs: none
// Outputs: none
// Waiting for data reception from USART1, will signal by USART1 ISR from USART1_IDLE_Callback()
void TelevisPortReception(void)
{
#ifdef APDEBUG	
	Count8 = 0;
#endif	
	uint8_t i;
	while(1){
			OS_Wait(&U1_RxSemaphore);
			/* Checks if Buffer full indication has been set */			
			if (U1_BufferReadyIndication != 0)
			{
				DisableInterrupts();
				/* Reset indication */
				U1_BufferReadyIndication = 0;
				TelevisMessageSize = U1_RxMessageSize;
				for(i = 0; i < U1_RxMessageSize; i++){
					ProtocolBuf[i] = U1_RXBufferA[i];
					U1_RXBufferA[i] = 0;
				}
				LEDs_off();
				EnableInterrupts();
				OS_Signal(&TelevisHndlReceiveSema); // signal TelevisHndlReceive() to process received message
			} else {
				if(!DeviceFound){
					OS_Signal(&CtrlScanSema); // signal TelevisScan() in case SCAN process
				}
			}
#ifdef APDEBUG			
		Count8++;
#endif			
	}
}

//***********TelevisPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer,
// USART1 with ODD parity
void TelevisPortTxInit(void)
{
#ifdef APDEBUG	
	Count5 = 0;
#endif	
	while(1){
		OS_Wait(&TelevisPortTxInitSema);
		DisableInterrupts();
		for(uint32_t i = 0; i < U1_TxMessageSize; i++){
			U1_TXBuffer[i] = ProtocolBuf[i];
		}
		// buffer pointer initialization
		U1_pBufferTransmit = &U1_TXBuffer[0];
		EnableInterrupts();
		LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_9B, LL_USART_PARITY_ODD, LL_USART_STOPBITS_1);
		if(!LL_USART_IsEnabledIT_ERROR(USART1)){
				/* Enable Error interrupt */
			LL_USART_EnableIT_ERROR(USART1);
		}
		U1_idxTx = 0;
		OS_Signal(&U1_TxSemaphore); // signal TelevisPortSendMsg() to receive message
#ifdef APDEBUG		
		Count5++;
#endif		
	}
}

//***********TelevisPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// Send Televis message to connected controller through USART1,
// first byte will send with ODD parity, after delay 100 microseconds will send other part of message with EVEN parity
void TelevisPortSendMsg(void){
#ifdef APDEBUG	
	Count6 = 0;
#endif	
	while(1){
		OS_Wait(&U1_TxSemaphore);
		/* Turn ORANGE On at start of transfer : Tx sequence started successfully */
		if(DeviceFound){
			LED_OrangeOn();
		}
		if(!U1_idxTx){
			LL_USART_TransmitData8(USART1, U1_pBufferTransmit[U1_idxTx++]);
			while(!LL_USART_IsActiveFlag_TC(USART1)){}
			LL_USART_DisableIT_TXE(USART1);
			LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_9B, LL_USART_PARITY_EVEN, LL_USART_STOPBITS_1);
			T2TimerDelay(100);
			OS_Signal(&U1_TxSemaphore); // signal ownself to continue with EVEN parity
		} else {
			if(LL_USART_IsActiveFlag_TC(USART1)){
				LL_USART_ClearFlag_TC(USART1);
			}
			LL_USART_TransmitData8(USART1, U1_pBufferTransmit[U1_idxTx++]);
			/* Enable TXE interrupt */
			LL_USART_EnableIT_TXE(USART1);
			OS_Signal(&TelevisPortRxInitSema); // Signal semaphore to initialize data reception	
		}
#ifdef APDEBUG		
		Count6++;
#endif		
	}
}

//***********TelevisCrc16***************
// returns the CRC of an Televis message
// Inputs: pointer to Televis message, size of the message
// Outputs: the CRC
// Calculate CRC for received Televis message
static uint32_t TelevisCrc16(const uint8_t *msg_data, uint8_t msg_lenght) 
{
  uint32_t crc_value = 0xFFFF;
  for (uint8_t i=0; i < msg_lenght; i++){
		crc_value -= msg_data[i];
	}
  return crc_value;
}

//***********TelevisHndlReceive***************
// returns none
// Inputs: none
// Outputs: none
// Handle message from controlled by Televis protocol
void TelevisHndlReceive(void)
{
  uint32_t crc;
  uint8_t msg_cmd, data_len, *crcptr, msg_len;
	TelevisReadReply *trr = (TelevisReadReply*)ProtocolBuf;
	
	while(1){
		OS_Wait(&TelevisHndlReceiveSema);
		msg_len = TelevisMessageSize;
		data_len = TelevisMessageSize - 2;
		msg_cmd = ev->ev_cmd;
		TelevisMessageSize = 0;
		crcptr = &ProtocolBuf[data_len];//pointer to crc in the message
		crc = TelevisCrc16(ProtocolBuf, data_len);//calculate crc
#ifdef APDEBUG
		DebugTelevisBuf[9] = *crcptr;
		DebugTelevisBuf[10] = *(crcptr+1);
		DebugTelevisBuf[11] = crc;
		DebugTelevisBuf[12]  = TelevisMessageSize;
		DebugTelevisBuf[13] = TelevisMessageSize - 2;
		DebugTelevisBuf[14] = ev->ev_cmd;
		DebugTelevisBuf[15] = trr->t_header;
		DebugTelevisBuf[16] = trr->t_sender;
		DebugTelevisBuf[17] = trr->t_len - 2;		
#endif
		if(	trr->t_header == 0x82 && ( (crc >> 8) == (*crcptr) || (crc&0xFF) == (*(crcptr+1)) )){ // check what message has header and right CRC
			DisableInterrupts();
			//--------------------------------
			//82  13  44  31  03  00 00  C2   FE 30 - read response
			//Hdr Cmd Src Dst Len Id     Data Crc
			if( (msg_cmd==CMD03) || (msg_cmd==CMD73) )//Read response
			{
#ifdef APDEBUG				
				if (trr->t_cmd != 0x13 || msg_len < 10){
					ErrorCMD03_073Televislog++;//short message
				}
#endif				
				ev->ev_len = trr->t_len - 2;
				ev->dataptr = &trr->t_data;
			}
			//--------------------------------
			//82  05  44  31  00  FF 03 - write response
			//Hdr Cmd Src Dst Len Crc
			else if( msg_cmd==CMD06 || msg_cmd==CMD10 || (msg_cmd==CMD76) )//Write response
			{
#ifdef APDEBUG				
				if (trr->t_cmd != 0x05 || msg_len < 7){
					ErrorCMD06_076Televislog++;//short message
				}
#endif				
			}
			//--------------------------------
			else if( msg_cmd == CMD43 )//Device ID response
			{
				ev->dataptr = &ProtocolBuf[5];
				ev->ev_len = trr->t_len;
			}
		//--------------------------------
		//82  56  33  EE  03  02 04 02  FD FB - scan response
		//Hdr Cmd Src Dst Len Data      Crc
			else if( msg_cmd == SCAN )//Scan response
			{
				if(!DeviceFound){
					DeviceFound = true;
				}
				ev->dataptr = &ProtocolBuf[regH];
				ev->ev_len = 3;
			}
		//--------------------------------
			else {
			//unknown command
#ifdef APDEBUG				
				ErrorCMDTelevislog++;
#endif				
			}
#ifdef APDEBUG			
			CountHandleReceive++;
#endif			
			EnableInterrupts();
			if(DeviceFound && msg_cmd == SCAN){
				OS_Signal(&CtrlScanSema); // Signal TelevisScan() in case it in SCAN process
			} else {
				OS_Signal(&ModbusSendSema); // Signal ModbusSend() to transmit message for MU device
			}
		} else {
			OS_Signal(&U3_RxInitSema); // In case received message is not valid switch to listen message from MU device
		}
	}
//  if (ev->debug == FARLOOP2) {Modbus.Send(TelevisBuf, len); return false;}
}

//***********TelevisSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Televis, signal TelevisPortTxInit() to transmit
void TelevisSend(void)
{
  uint8_t msg_cmd, msg_len;
  uint32_t data_len, crc;

	while(1){
		OS_Wait(&TelevisHndlSendSema);
		DisableInterrupts();
		TelevisReadRequest  *trr = (TelevisReadRequest*)ProtocolBuf;
		TelevisWriteRequest *twr = (TelevisWriteRequest*)ProtocolBuf;
		trr->t_header = SOF;
		trr->t_sender = 0;
		trr->t_receiver = ev->ev_addr;
		msg_cmd = ev->ev_cmd;
		//--------------------------------
		//82  92  31  44  05  10 A8  01   00 00  FD E8
		//Hdr Cmd Src Dst Len Reg    Num  Id     Crc
		if(( msg_cmd == CMD03 ) || ( msg_cmd == CMD73 ))
		{
			if(msg_cmd == CMD03) trr->t_cmd = 0x92; else trr->t_cmd = 0x12;

			trr->t_len = 5;
			trr->t_regH = ev->ev_reg >> 8;
			trr->t_regL= ev->ev_reg & 0xFF;
			trr->t_num = ev->ev_len;
			trr->t_id  = 0;
			msg_len = 10;
			
#ifdef APDEBUG			
			DebugTelevisBuf[0] = msg_cmd;
			DebugTelevisBuf[1] = trr->t_cmd;		
			DebugTelevisBuf[3] = trr->t_len;
			DebugTelevisBuf[4] = trr->t_regH;
			DebugTelevisBuf[5] = trr->t_regL;
			DebugTelevisBuf[6] = trr->t_num;
			DebugTelevisBuf[7] = trr->t_id;
			DebugTelevisBuf[8] = msg_len;
#endif
		}
		//--------------------------------
		//82  93  31  44  04  10 02  32 00  FE 2D
		//Hdr Cmd Src Dst Len Reg    Data   Crc
		else if( msg_cmd == CMD06 || msg_cmd == CMD10 )
		{
			data_len = ev->ev_len;
#ifdef APDEBUG			
			if( data_len > 20 ){
				ErrorCMDTelevislog++; // error
			}
#endif			
			twr->t_cmd = 0x93;
			twr->t_len  = data_len + 2;
			twr->t_regH = ev->ev_reg >> 8;
			twr->t_regL = ev->ev_reg;
			for(uint32_t i = 0; i < data_len; i++){
				*(&twr->t_data + (i^1)) = *(ev->dataptr + i); //^1 because need to swap bytes
			}
			msg_len = data_len + 7;
		}
		else if( msg_cmd == CMD76 )
		{
			twr->t_cmd = 0x93;
			twr->t_len = 3;
			twr->t_regH = ev->ev_reg>>8;
			twr->t_regL = ev->ev_reg;
			twr->t_data = *(ev->dataptr + 1);//+1 because we send LSB only
			msg_len = 8;
		}
		//--------------------------------
		//82  D6  EE  33  01  02   FD 83
		//Hdr Cmd Src Dst Len Data Crc
		else if( msg_cmd == CMD43 )
		{
			twr->t_cmd = SCAN;
			twr->t_sender = 0;
			twr->t_receiver = ev->ev_addr;
			twr->t_len = 0x01; //Len
			twr->t_regH = *ev->dataptr;//0x02;
			msg_len = 6;
		}
		else if( msg_cmd == SCAN )
		{
			twr->t_cmd = SCAN;
			twr->t_sender = 0;
			twr->t_receiver = ev->ev_addr;
			twr->t_len = 0x01;
			twr->t_regH = 0x02;
			msg_len = 6;
		}
		//--------------------------------
		else msg_len = 1;
		crc = TelevisCrc16(ProtocolBuf, msg_len);
		ProtocolBuf[msg_len++] = crc >> 8;//Add CRC to buf
		ProtocolBuf[msg_len++] = crc & 0xFF;
		U1_TxMessageSize = msg_len;
		//  if(ev->debug == FARLOOP1) Modbus.Send(TelevisBuf, len);
		EnableInterrupts();
		OS_Signal(&TelevisPortTxInitSema);
	}
}

//***********TelevisScan***************
// returns none
// Inputs: none
// Outputs: none
// Initialize connected controller address scan by setup Televis scan message and 
// signal TelevisPortTxInit() to transmit it to connected controller, if responce will received, controller address was found,
// thread will blocked, MU port reception thread will signaled otherwise will prepare new scan message every 200ms.
void TelevisScan(void){
	uint8_t i, msg_len;
	uint32_t crc, TimeDelay = 5000000;
	for(i = 0; i < 6; i++){
		ProtocolBuf[i] = TelevisScanCMD[i];
	}
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
//			OS_Wait(&TelevisCtrlScanSema); 09/01/2018
		} else {
				// Toggle Green Led indicate that scan operation running
				ToggleLedGreen();
				// Setup address to get responce from controller
				if(ProtocolBuf[receiver]++ > 247){
					ProtocolBuf[receiver] = 1;
				}
				Event.ev_addr = ProtocolBuf[receiver];
				Event.ev_cmd = ProtocolBuf[cmd];
				Event.ev_debug = 0;
				Event.dataptr = &ProtocolBuf[regH];
				crc = TelevisCrc16(ProtocolBuf, msg_len);
				ProtocolBuf[msg_len++] = crc >> 8; //Add CRC to buf
				ProtocolBuf[msg_len++] = crc & 0xFF;
				U1_TxMessageSize = msg_len;
				OS_Signal(&TelevisPortTxInitSema);
#ifdef APDEBUG				
				CntCMDSend++;
#endif
				OS_Sleep(200);
		}
	}
}

//-----------------------------------------------------------
/************************ (C) COPYRIGHT *****END OF FILE****/
