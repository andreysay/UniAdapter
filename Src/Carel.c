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

// Semaphores
int32_t CarelHndlReceiveSema;
int32_t CarelHndlSendSema;
int32_t ReadCarelENQSema;
int32_t ReadCarelACKSema;

// Local flags
static bool valuesReadFlag = false;
static bool sendENQ_Flag = false;
static bool sendACK_Flag = false;

// Carel indexs symbols
#define carel_header	 					0 // SOF(header) index
#define carel_address						1 // sender address index
#define carel_DREQ							2 // cmd index 
#define carel_var_type_index 		2 // index for variable type		
#define carel_var_index					3	// index for carel variable index


// Carel protocol symbols
#define DIG_ARRAY_SIZE 199
#define INT_ARRAY_SIZE 127
#define ANA_ARRAY_SIZE 127

// Array to store DIG(digital variable) values from Carel controller. Array indexies correspond digit variable index by formula variable_index = array_index + 1
int16_t DIG_ARRAY[DIG_ARRAY_SIZE];
// Array to store INT(integer variable) values from Carel controller Array indexies correspond indeger variable index by formula variable_index = array_index + 1
int32_t INT_ARRAY[INT_ARRAY_SIZE];
// Array to store ANA(analog variable) values from Carel controller Array indexies correspond analog variable index by formula variable_index = array_index + 1
int32_t ANA_ARRAY[ANA_ARRAY_SIZE];

//*************message and message fragments**********
// Device request command 
const uint8_t CarelScanFrame[] = {
	STX, // Carel SOF header
	0x00, // Receiver address
	0x3F, // Device request command 
	ETX, // 
	0x00, // CRC_Hi
	0x00 // CRC_Lo
};
// ACK responce
const uint8_t cmdACK[] = {
	0x06
};
// Enquire command
const uint8_t cmdENQ[] = {
	0x05,
	0x00
};



//***********CarelCRC***************
// returns none
// Inputs: none
// Outputs: none
// Add CRC to Carel controller message, so a message must have 2 byte place for it.
static void CarelCRC(uint8_t* msg, uint8_t size){
   uint8_t crc = 0;
   for (uint8_t i = 0; i < size - 2; i++){
     crc = crc + msg[i];
   }
   msg[size-2] = 0x30 + ((crc >> 4) & 0x0F);
   msg[size-1] = 0x30 + (crc & 0x0F);

}


static bool CarelCRC_check(uint8_t* msg, uint8_t size){
  uint8_t rc = 0;
  for (int i = 0; i < size - 2; i++){
		rc = rc + msg[i];
	}
  if ( ( ( (rc >> 4) & 0x0F ) == ( msg[size-2] - 0x30 ) ) || ( (rc & 0x0F) == ( msg[size-1] - 0x30 ) ) )
	{ 
		return true;
	}
	return false;
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
					U1_RXBufferA[i] = 0;
				}
				LEDs_off();
				EnableInterrupts();
				OS_Signal(&CarelHndlReceiveSema);
			} else {
				if(!DeviceFound){
					OS_Signal(&CtrlScanSema); // signal CarelScan() in case SCAN process
				} else if(sendACK_Flag){
					OS_Signal(&ReadCarelENQSema);
				} else {
					
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
//		msg_len = ProtocolMsgSize;
//		data_len = ProtocolMsgSize - 2;
		msg_cmd = ev->ev_cmd;
		
		if( msg_cmd == DREQ ){
			if(!DeviceFound){
				DeviceFound = true;
			}
		}
		if(DeviceFound && msg_cmd == DREQ){
			OS_Signal(&CtrlScanSema); // Signal CarelScan() in case it in SCAN process
		} else if(DeviceFound && msg_cmd == ENQ){
			OS_Signal(&ReadCarelACKSema);
		}	else {
			OS_Signal(&ModbusSendSema); // Signal ModbusSend() to transmit message for MU device
		}		

	}		
}

//***********CarelSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Carel
void CarelSend(void)
{
  uint8_t msg_cmd, msg_len;
  uint32_t data_len, crc;

	while(1){
		OS_Wait(&CarelHndlSendSema);
//		DisableInterrupts();
//		ProtocolBuf[carel_header] = STX;
//		ProtocolBuf[carel_address] = ev->ev_addr;
//		msg_cmd = ev->ev_cmd;
		//--------------------------------
//		if( msg_cmd == RCOIL )
//		{

//		}
//		else if(( msg_cmd == CMD03 ) || ( msg_cmd == CMD73 ))
//		{
//			
//#ifdef APDEBUG			
//			DebugTelevisBuf[0] = msg_cmd;
//			DebugTelevisBuf[1] = trr->t_cmd;		
//			DebugTelevisBuf[3] = trr->t_len;
//			DebugTelevisBuf[4] = trr->t_regH;
//			DebugTelevisBuf[5] = trr->t_regL;
//			DebugTelevisBuf[6] = trr->t_num;
//			DebugTelevisBuf[7] = trr->t_id;
//			DebugTelevisBuf[8] = msg_len;
//#endif
//		}
//		//--------------------------------
//		//82  93  31  44  04  10 02  32 00  FE 2D
//		//Hdr Cmd Src Dst Len Reg    Data   Crc
//		else if( msg_cmd == CMD06 || msg_cmd == CMD10 )
//		{

//		}
//		else if( msg_cmd == CMD76 )
//		{

//		}
//		//--------------------------------
//		//82  D6  EE  33  01  02   FD 83
//		//Hdr Cmd Src Dst Len Data Crc
//		else if( msg_cmd == CMD43 )
//		{

//		}
//		else if( msg_cmd == ENQ )
//		{
//			ProtocolBuf[carel_header] = ENQ;
//			msg_len = 2;
//		}
//		//--------------------------------
//		else {
//			
//		}
//		msg_len = ev->ev_len + 2;
//		if( msg_cmd != ENQ ){
//			CarelCRC(ProtocolBuf, msg_len);
//		}
//		U1_TxMessageSize = msg_len;
//		EnableInterrupts();
//		OS_Signal(&PortCtrlTxInitSema);
		if(!valuesReadFlag){
			OS_Signal(&ReadCarelENQSema);
			valuesReadFlag = true;
		}
	}
}

static void sendReadDREQ(void){
	ProtocolBuf[0] = STX;
	ProtocolBuf[1] = ev->ev_addr;
	ProtocolBuf[2] = DREQ;
	ProtocolBuf[3] = 0x30;
	ProtocolBuf[4] = ETX;
	U1_TxMessageSize = ProtocolMsgSize = 7;
	CarelCRC(&ProtocolBuf[0], ProtocolMsgSize);
}

static void sendRead_F(void){
	ProtocolBuf[0] = STX;
	ProtocolBuf[1] = ev->ev_addr;
	ProtocolBuf[2] = FREQ;
	ProtocolBuf[3] = 0x31;
	ProtocolBuf[4] = ETX;
	U1_TxMessageSize = ProtocolMsgSize = 7;
	CarelCRC(&ProtocolBuf[0], ProtocolMsgSize);	
}

static void sendENQ(void){
	ev->ev_cmd = ENQ;
	ProtocolBuf[0] = ENQ;
	ProtocolBuf[1] = ev->ev_addr;
	U1_TxMessageSize = ProtocolMsgSize = 2;
	sendENQ_Flag = true;
	sendACK_Flag = false;
}

static void sendACK(){
	ProtocolBuf[0] = 0x06;
	U1_TxMessageSize = ProtocolMsgSize = 1;
	sendACK_Flag = true;
	sendENQ_Flag = false;
}

void readCarelCtrlENQ(void){
	while(1){
		OS_Wait(&ReadCarelENQSema);
		OS_Sleep(50);
		sendENQ();
		OS_Signal(&PortCtrlTxInitSema);
	}
}

void readCarelCtrlACK(void){
	uint16_t varType;
	uint32_t varIndex;
	bool correctIndexFlag = false;
	bool correctCRC = false;
	
	while(1){
		OS_Wait(&ReadCarelACKSema);
		if(ProtocolBuf[0]){
			DisableInterrupts();
			correctCRC = CarelCRC_check(&ProtocolBuf[0], ProtocolMsgSize);
			if( correctCRC ){
				varType = ProtocolBuf[2];
				varIndex = ProtocolBuf[3] - 0x30;
				switch(varType){
					case RANA_VAR:
						if(varIndex > 0 && varIndex < 128){
							ANA_ARRAY[varIndex] = (((ProtocolBuf[5] - 0x30) << 12) | ((ProtocolBuf[6] - 0x30) << 8) | ((ProtocolBuf[7] - 0x30) << 4) | ProtocolBuf[8]);
							correctIndexFlag = true;
						} else {
							correctIndexFlag = false;
						}
						break;
					case RINT_VAR:
						if(varIndex > 0 && varIndex < 128){
							INT_ARRAY[varIndex] = (((ProtocolBuf[5] - 0x30) << 12) | ((ProtocolBuf[6] - 0x30) << 8) | ((ProtocolBuf[7] - 0x30) << 4) | ProtocolBuf[8]);
							correctIndexFlag = true;
						} else {
							correctIndexFlag = false;
						}
						break;
					case RDIG_VAR:
						if(varIndex > 0 && varIndex < 200){
							DIG_ARRAY[varIndex] = ((ProtocolBuf[5] - 0x30) << 4 ) | (ProtocolBuf[6] - 0x30);
							correctIndexFlag = true;
						} else {
							correctIndexFlag = false;
						}
						break;
					default:
						break;
				}
			}
			EnableInterrupts();
			if(correctIndexFlag){
				OS_Sleep(50);
				sendACK();
				OS_Signal(&PortCtrlTxInitSema);
			} else {
				valuesReadFlag = false;
				OS_Signal(&CarelHndlSendSema);
			}
		} else {
			valuesReadFlag = false;
			OS_Signal(&ModbusSendSema);
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
				if(ProtocolBuf[carel_address]++ > 254){
					ProtocolBuf[carel_address] = 0x31;
				}
				Event.ev_addr = ProtocolBuf[carel_address];
				Event.ev_cmd = ProtocolBuf[carel_DREQ];
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
