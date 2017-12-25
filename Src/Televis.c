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

// Televis symbols
#define header 		0 // SOF place, number in message
#define cmd				1 // cmd place
#define sender		2 // sender address place
#define receiver	3 // receiver address place
#define len				4 // lenght of data for transmition, place
#define regH			5 // register address value place MSB
#define regL			6 // register address value place LSB
#define data			7 // lenght of data for responce, place
#define id				8 // id transaction place

#define SOF 0x82		// Televis start of frame(header)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t DebugTelevisBuf[RX_BUFFER_SIZE]; // Buffer for debugging purpose

/* Initial autoreload value */
extern __IO uint8_t TIM2_InterruptFlag;

// Autoreload value for TIM2 timer
static uint32_t InitialAutoreload = 0;



//*************message and message fragments**********
const uint8_t TelevisScanCMD[] = {
	SOF, // Televis SOF header
	0xD6, // Scan command 
	0x00, // Sender address
	0x00, // Receiver address
	0x01, // Message lenght
	0x02, // Register MSB
};

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
//extern uint8_t ModbusBuf[];

// link to ControllerType.c
extern TEvent Event;
extern TEvent *const ev;
extern bool DeviceFound;

// Semaphore for TelevisHandleReceive thread
int32_t TelevisHndlReceiveSema;

int32_t TelevisHndlSendSema;

int32_t TelevisPortTxInitSema;

int32_t TelevisPortRxInitSema;

int32_t TelevisCtrlScanSema;

uint8_t TelevisMessageSize;

// Buffer for Televis message
uint8_t TelevisBuf[RX_BUFFER_SIZE];


extern uint32_t Count3;
void TelevisEventThread100ms(void){
	Count3 = 0;
	while(1){
		OS_Wait(&Time100msSemaphore);           // 1000 Hz real time task
		Count3++;
	};
}

extern uint32_t Count4;
void TelevisEventThread1sec(void){ 
  Count4 = 0;
	while(1){
		OS_Wait(&Time1secSemaphore);
		if(DeviceFound){
			ToggleLedGreen();
		}
		Count4++;
	};
}


extern uint32_t Count7;
void TelevisPortRxInit(void)
{
	Count7 = 0;
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
		Count7++;
	}
}

extern uint32_t Count8;
void TelevisPortReception(void)
{
	Count8 = 0;
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
					TelevisBuf[i] = U1_RXBufferA[i];
//					U1_RXBufferA[i] = 0;
				}
				LEDs_off();
				EnableInterrupts();
				OS_Signal(&TelevisHndlReceiveSema);
			} else {
				if(!DeviceFound){
					OS_Signal(&TelevisCtrlScanSema);
				}
			}
		Count8++;
	}
}

extern uint32_t Count5;
void TelevisPortTxInit(void)
{
	Count5 = 0;
	while(1){
		OS_Wait(&TelevisPortTxInitSema);
		DisableInterrupts();
		for(uint32_t i = 0; i < U1_TxMessageSize; i++){
			U1_TXBuffer[i] = TelevisBuf[i];
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
		OS_Signal(&U1_TxSemaphore);
		Count5++;
	}
}


extern uint32_t Count6;
void TelevisPortSendMsg(void){
	Count6 = 0;
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
			TTimerDelay(100);
			OS_Signal(&U1_TxSemaphore);
		} else {
			if(LL_USART_IsActiveFlag_TC(USART1)){
				LL_USART_ClearFlag_TC(USART1);
			}
			LL_USART_TransmitData8(USART1, U1_pBufferTransmit[U1_idxTx++]);
			/* Enable TXE interrupt */
			LL_USART_EnableIT_TXE(USART1);
			OS_Signal(&TelevisPortRxInitSema); // Signal semaphore to initialize data reception	
		}
		Count6++;
	}
}

//-----------------------------------------------------------
static uint32_t TelevisCrc16(const uint8_t *msg_data, uint8_t msg_lenght) 
{
  uint32_t crc_value = 0xFFFF;
  for (uint8_t i=0; i < msg_lenght; i++){
		crc_value -= msg_data[i];
	}
  return crc_value;
}
uint32_t CountHandleReceive;
//-----------------------------------------------------------
void TelevisHndlReceive(void)
{
  uint32_t crc;
  uint8_t msg_cmd, data_len, *crcptr, msg_len;
	
  TelevisReadReply *trr = (TelevisReadReply*)TelevisBuf;
	while(1){
		OS_Wait(&TelevisHndlReceiveSema);
		DisableInterrupts();
#ifdef DEBUGVIEW		
		DebugTelevisBuf[9]  = TelevisMessageSize;
		DebugTelevisBuf[10] = TelevisMessageSize - 2;
		DebugTelevisBuf[11] = ev->ev_cmd;
		DebugTelevisBuf[12] = trr->t_header;
		DebugTelevisBuf[13] = trr->t_sender;
		DebugTelevisBuf[14] = trr->t_len - 2;
#endif
		msg_len = TelevisMessageSize;
		data_len = TelevisMessageSize - 2;
		msg_cmd = ev->ev_cmd;
		TelevisMessageSize = 0;
		if(trr->t_header != 0x82){
			if(DeviceFound){
				_Error_Handler(__FILE__, __LINE__);//wrong header
			} else {
				OS_Signal(&TelevisCtrlScanSema);
				OS_Wait(&TelevisHndlReceiveSema);
			}
		}
		if(trr->t_sender != ev->ev_addr){
			if(DeviceFound){
				_Error_Handler(__FILE__, __LINE__);
			} else {
				OS_Signal(&TelevisCtrlScanSema);
				OS_Wait(&TelevisHndlReceiveSema);//wrong addr
			}
		}
		//--------------------------------
		//82  13  44  31  03  00 00  C2   FE 30 - read response
		//Hdr Cmd Src Dst Len Id     Data Crc
		if( (msg_cmd==CMD03) || (msg_cmd==CMD73) )//Read response
		{
			if (trr->t_cmd != 0x13 || msg_len < 10){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			ev->ev_len = trr->t_len - 2;
			ev->dataptr = &trr->t_data;
			crcptr = &TelevisBuf[data_len];//pointer to crc in the message
			crc = TelevisCrc16(TelevisBuf, data_len);//calculate crc
#ifdef DEBUGVIEW
			DebugTelevisBuf[15] = crc;
#endif
		}
		//--------------------------------
		//82  05  44  31  00  FF 03 - write response
		//Hdr Cmd Src Dst Len Crc
		else if( msg_cmd==CMD06 || msg_cmd==CMD10 || (msg_cmd==CMD76) )//Write response
		{
			if (trr->t_cmd != 0x05 || msg_len < 7){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			crcptr = &TelevisBuf[data_len];//pointer to crc in the message
			crc = TelevisCrc16(TelevisBuf, data_len);//calculate crc
		}
		//--------------------------------
		else if( msg_cmd == CMD43 )//Device ID response
		{
			//if (trr->cmd != 0x56 || len < 7) return FALSE;//short message
			ev->dataptr = &TelevisBuf[5];
			ev->ev_len = trr->t_len;
			crcptr = &TelevisBuf[data_len];//pointer to crc in the message
			crc = TelevisCrc16(TelevisBuf, data_len);//calculate crc
		}
		//--------------------------------
		//82  56  33  EE  03  02 04 02  FD FB - scan response
		//Hdr Cmd Src Dst Len Data      Crc
		else if( msg_cmd == SCAN )//Scan response
		{
			if(!DeviceFound){
				DeviceFound = true;
			}
			
			//if (trr->cmd != 0x56 || len < 7) return FALSE;//short message
			ev->dataptr = &TelevisBuf[regH];
			ev->ev_len = 3;//trr->len;
			crcptr = &TelevisBuf[data_len];//pointer to crc in the message
			crc = TelevisCrc16(TelevisBuf, data_len);//calculate crc
		}
		//--------------------------------
		else {
			//unknown command
			_Error_Handler(__FILE__, __LINE__);
		}
		DebugTelevisBuf[15] = *crcptr;
		DebugTelevisBuf[16] = *(crcptr+1);
		if ( (crc >> 8) != (*crcptr) || (crc&0xFF) != (*(crcptr+1)) ){
			//wrong crc
			_Error_Handler(__FILE__, __LINE__);
		}
		CountHandleReceive++;
		EnableInterrupts();
		if(DeviceFound && msg_cmd == SCAN){
			OS_Signal(&TelevisCtrlScanSema);
		} else {
			OS_Signal(&ModbusSendSema);
		}
	}

//  if (ev->debug == FARLOOP2) {Modbus.Send(TelevisBuf, len); return false;}
}
//-----------------------------------------------------------
void TelevisSend(void)
{
  uint8_t data_len, msg_cmd, msg_len;
  uint32_t crc;

	while(1){
		OS_Wait(&TelevisHndlSendSema);
		DisableInterrupts();
		TelevisReadRequest  *trr = (TelevisReadRequest*)TelevisBuf;
		TelevisWriteRequest *twr = (TelevisWriteRequest*)TelevisBuf;
		trr->t_header = SOF;
		trr->t_sender = 0;
		trr->t_receiver = ev->ev_addr;
		msg_cmd = ev->ev_cmd;
		//--------------------------------
		//82  92  31  44  05  10 A8  01   00 00  FD E8
		//Hdr Cmd Src Dst Len Reg    Num  Id     Crc
		if((msg_cmd == CMD03) ||(msg_cmd == CMD73))
		{
			if(msg_cmd == CMD03) trr->t_cmd = 0x92; else trr->t_cmd = 0x12;

			trr->t_len = 5;
			trr->t_regH = ev->ev_reg >> 8;
			trr->t_regL= ev->ev_reg & 0xFF;
			trr->t_num = ev->ev_len;
			trr->t_id  = 0;
			msg_len = 10;
			
#ifdef DEBUGVIEW			
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
		else if(msg_cmd == CMD06 || msg_cmd == CMD10)
		{
			if((data_len = ev->ev_len) > 20){
				_Error_Handler(__FILE__, __LINE__); // error
			}
			twr->t_cmd = 0x93;
			twr->t_len  = data_len + 2;
			twr->t_regH = ev->ev_reg >> 8;
			twr->t_regL = ev->ev_reg;
			//x=ev->size;//if byte size no need to swap
			//for (i=0; i<n; i++) *(&twr->data+(i^x)) = *(ev->dataptr+i);//^x means swap bytes
			for(uint32_t i = 0; i < data_len; i++){
				*(&twr->t_data + (i^1)) = *(ev->dataptr + i); //^1 because need to swap bytes
			}
			msg_len = data_len + 7;
		}
		else if(msg_cmd == CMD76)
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
		else if(msg_cmd == CMD43)
		{
			twr->t_cmd = SCAN;
			twr->t_sender = 0;
			twr->t_receiver = ev->ev_addr;
			twr->t_len = 0x01; //Len
			twr->t_regH = *ev->dataptr;//0x02;
			msg_len = 6;
		}
		else if(msg_cmd == SCAN)
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
		crc = TelevisCrc16(TelevisBuf, msg_len);
		TelevisBuf[msg_len++] = crc >> 8;//Add CRC to buf
		TelevisBuf[msg_len++] = crc & 0xFF;
		U1_TxMessageSize = msg_len;
		//  if(ev->debug == FARLOOP1) Modbus.Send(TelevisBuf, len);
		EnableInterrupts();
		OS_Signal(&TelevisPortTxInitSema);
	}
}


uint32_t CntDeviceFound;
uint32_t CntCMDSend;
void TelevisScan(void){
	uint8_t i, msg_len;
	uint32_t crc, TimeDelay = 5000000;
	for(i = 0; i < 6; i++){
		TelevisBuf[i] = TelevisScanCMD[i];
	}
	LED_GreenOn();
	// Delay for 5 sec in case when controller power on 
	TTimerDelay(TimeDelay);
	LEDs_off();
	while(1){
		OS_Wait(&TelevisCtrlScanSema);
		msg_len = 6;
		if(DeviceFound){
			CntDeviceFound++;
			OS_Signal(&U3_RxInitSema);
			OS_Wait(&TelevisCtrlScanSema);
		} else {
				// Toggle Green Led indicate that scan operation running
				ToggleLedGreen();
				// Setup device address to get responce from controller
				if(TelevisBuf[receiver]++ > 247){
					TelevisBuf[receiver] = 1;
				}
				CntCMDSend++;
				Event.ev_addr = TelevisBuf[receiver];
				Event.ev_cmd = TelevisBuf[cmd];
				Event.ev_debug = 0;
				Event.dataptr = &TelevisBuf[regH];
				crc = TelevisCrc16(TelevisBuf, msg_len);
				TelevisBuf[msg_len++] = crc >> 8; //Add CRC to buf
				TelevisBuf[msg_len++] = crc & 0xFF;
				U1_TxMessageSize = msg_len;
				OS_Signal(&TelevisPortTxInitSema);
				OS_Sleep(200);
		}
	}
}

//***********TTimerDelay***************
// returns none
// Inputs:  Delay time in microseconds
// Outputs: none
// Delay amount of time in microseconds
void TTimerDelay(uint32_t mcsTimeDelay){
	/* Clear the update event flag */
	TIM2->SR = 0;
	while(mcsTimeDelay){
		/* Start the timer counter */
		TIM2->CR1 |= TIM_CR1_CEN;
		/* Loop until the update event flag is set */
		while (!(TIM2->SR & TIM_SR_UIF)){}
		/* The required time delay has been elapsed */
		/* Clear the update event flag */
		TIM2->SR = 0;
		mcsTimeDelay--;
	}
}


//***********TelevisTIM2TimeInit***************
// returns none
// Inputs:  none
// Outputs: none
// Initialize TIM2 to get ticks in microseconds
void TelevisTIM2TimeInit(void)
{
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); 
  
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz      */
  /*
    In this example TIM2 input clock (TIM2CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM2CLK = PCLK1
      PCLK1 = HCLK
      => TIM2CLK = HCLK = SystemCoreClock
    To get TIM2 counter clock at 1 MHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /(2 * 1MHz)) - 1
  */
  LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock/2, 1000000));
  
  /* Set the auto-reload value to have an initial update event frequency of 1 MHz */
    /* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)                 */
	InitialAutoreload = 1U;
  LL_TIM_SetAutoReload(TIM2, InitialAutoreload);
  
  /* Configure the NVIC to handle TIM2 update interrupt */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
}

//***********Televis_GetSize***************
// returns the size of an Televis message
// Inputs:  pointer to Televis message
// Outputs: size of the message
// extracted little Endian from byte 1 and byte 2
uint32_t Televis_GetSize(uint8_t *pt){
  uint8_t msb,lsb;
  uint32_t size;
  lsb = (uint8_t)pt[1];
  msb = (uint8_t)pt[2];
  size = (msb<<8)+lsb;
  return size;
}

//-----------------------------------------------------------
/************************ (C) COPYRIGHT *****END OF FILE****/
