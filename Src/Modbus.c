/**
  ******************************************************************************
  * File Name          : Modbus.c
  * Description        : This file provides code for Send/Receive functions
  *                      of the Modbus protocol.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "ControllerType.h"
#include "os.h"
#include "LED.h"
#include "Modbus.h"

uint8_t DebugModbusBuf[64]; // Buffer for debugging purpose

uint8_t ModbusBuf[RX_BUFFER_SIZE];

uint32_t ModbusMsgSize;

int32_t ModbusReceiveSema;

int32_t ModbusHndlReceiveSema;

int32_t ModbusSendSema;

int32_t ModbusPortTxInitSema;

// link to ControllerType.c
extern TEvent *const ev;
extern TEvent Event;
extern bool DeviceFound;

// link to main_threads.c file
extern int32_t U3_RxSemaphore; 			// defined in main_threads.c file
extern __IO uint32_t U3_idxTx; 			// defined in main_threads.c file
extern uint8_t *U3_pBufferTransmit; // defined in main_threads.c file
extern uint8_t *U3_pBufferReception; // defined in main_threads.c file
extern __IO uint32_t U3_idxRx; 			// defined in main_threads.c file
extern __IO uint32_t U3_BufferReadyIndication; // defined in main_threads.c file
extern __IO uint8_t U3_RxMessageSize; // defined in main_threads.c file
extern uint8_t U3_TxMessageSize; 		// defined in main_threads.c file
extern int32_t U3_TxSemaphore;
extern int32_t U3_RxSemaphore;
extern int32_t U3_RxInitSema;
extern uint8_t U3_RXBuffer[]; // defined in main_threads.c file
extern uint8_t U3_TXBuffer[]; // defined in main_threads.c file

// link to Televis.c
extern int32_t TelevisHndlSendSema;
//extern uint8_t TelevisBuf[];

// Counters for debugging
extern uint32_t Count0;
void ModbusPortRxInit(void)
{
	Count0 = 0;
	GPIO_PinState pinDirState;
	while(1){
		OS_Wait(&U3_RxInitSema);
		pinDirState = HAL_GPIO_ReadPin(GPIO_Dir, PIN_Dir);
		if(pinDirState != GPIO_PIN_RESET){
			HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_RESET);
		}
		// Initialize number of received bytes
		U3_idxRx = 0;
		// Initialize buffer ready indication
		U3_BufferReadyIndication = 0;
		// buffers pointers initialization
		U3_pBufferReception = &U3_RXBuffer[0];
//		U3_pBufferTransmit = &U3_TXBuffer[0];
		/* Enable IDLE */
		LL_USART_EnableIT_IDLE(USART3);
		/* Clear Overrun flag, in case characters have already been sent to USART */
		LL_USART_ClearFlag_ORE(USART3);

		/* Enable RXNE and Error interrupts */
		LL_USART_EnableIT_RXNE(USART3);
		LL_USART_EnableIT_ERROR(USART3);		
		OS_Signal(&U3_RxSemaphore);
		Count0++;
	}
}

extern uint32_t Count1;
void ModbusPortReception(void)
{
	Count1 = 0;
	uint8_t i = 0;
	while(1){
		OS_Wait(&U3_RxSemaphore);
  /* Checks if Buffer full indication has been set */
		if (U3_BufferReadyIndication != 0)
		{
			// Disable interrupts
			DisableInterrupts();
			/* Reset indication */
			U3_BufferReadyIndication = 0;
			ModbusMsgSize = U3_RxMessageSize;
			for(i = 0; i < U3_RxMessageSize; i++){
				ModbusBuf[i] = U3_RXBuffer[i];
//				U3_RXBuffer[i] = 0;
			}
			/* Turn off green led, indication that data from monitor unit received */	
			LEDs_off();
			// Enable interrupts
			EnableInterrupts();
			OS_Signal(&ModbusHndlReceiveSema);
		}
		Count1++;
	}
}

uint32_t Count9;
void ModbusPortTxInit(void){
	Count9 = 0;
	GPIO_PinState pinDirState;
	
	while(1){
		OS_Wait(&ModbusPortTxInitSema);
		U3_pBufferTransmit = &U3_TXBuffer[0];
		DisableInterrupts();
		DebugModbusBuf[13] = U3_TxMessageSize;
		for(uint32_t i = 0; i < U3_TxMessageSize; i++){
			U3_TXBuffer[i] = ModbusBuf[i];
//		ModbusBuf[i] = 0;
		}
		EnableInterrupts();
		pinDirState = HAL_GPIO_ReadPin(GPIO_Dir, PIN_Dir);
		if(pinDirState != GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_SET);
		}
		Count9++;
		OS_Signal(&U3_TxSemaphore);
	}
}


extern uint32_t Count2;
void ModbusPortSendMsg(void){
	Count2 = 0;

	while(1){
		OS_Wait(&U3_TxSemaphore);
		/* Turn ORANGE On at start of transfer : Tx started */
		LED_OrangeOn();
		/* Fill DR with a new char */
		LL_USART_TransmitData8(USART3, U3_TXBuffer[U3_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART3); 
		Count2++;
	}
}
//-----------------------------------------------------------
static uint32_t ModbusCrc16(const uint8_t *msg_data, uint8_t msg_len) 
{
  uint32_t crc_value = 0;
  uint8_t shift_cnt;
  if(msg_data) 
  {
    crc_value = 0xFFFFU;
    for(; msg_len > 0; msg_len--) 
    {
      crc_value = (uint32_t) ((crc_value / 256U) * 256U + ((crc_value % 256U) ^ (*msg_data++)));
      for(shift_cnt = 0; shift_cnt < 8; shift_cnt++) 
      {
        if((crc_value & 0x1) == 1){
					crc_value = (uint32_t) ((crc_value >> 1) ^ 0xA001U);
				} else {
					crc_value >>= 1;
				}
      }
    }
  }
  return crc_value;
}

//-----------------------------------------------------------
//void ModbusReceive(void)
//{

//	ModbusCmd *mb = (ModbusCmd*)ModbusBuf;
//	uint32_t crc;
//	uint8_t *crcptr, data_len, ptr, len;
//	
//	while(1){
//		OS_Wait(&ModbusReceiveSema);
//		msg_len = U3_RxMessageSize;
//		len = ptr; ptr = 0; data_len = msg_len - 2;
//		if(mb->addr != BROADCAST) state= false;	//if wrong addr
//		if(mb->cmd == ADDR)	//Set addr command
//		{
//			if( len < 5 ) state= false;//short message
//			crc = ModbusCrc16(ModbusBuf, data_len);//calculate crc
//			crcptr = &ModbusBuf[data_len];	//pointer to crc in the message
//			if(((crc&0xFF) == *crcptr) && ((crc>>8) == *(crcptr+1))) //verify crc
//			{
//				*data = ModbusBuf[2];//new addr of controller
//				state= true;
//			}
//		}
//		state= false;
//	}
//}
void ModbusHndlReceive(void)//Request Called by App
{
  ModbusCmd *mb = (ModbusCmd*)ModbusBuf;
  ModbusCmd10 *cmd10 = (ModbusCmd10*)ModbusBuf;
  uint32_t crc;
  uint8_t msg_cmd, *crcptr, data_len, msg_len;
	while(1){
		OS_Wait(&ModbusHndlReceiveSema);
		DisableInterrupts();
		ev->ev_reg = 0;
		msg_len = ModbusMsgSize;
		data_len = msg_len - 2;
		ModbusMsgSize = 0;
		msg_cmd = mb->cmd;		
#ifdef DEBUGVIEW		
		DebugModbusBuf[0] = mb->addr;
		DebugModbusBuf[1] = mb->cmd;
		DebugModbusBuf[2] = mb->regHi;
		DebugModbusBuf[3] = mb->regLo;
		DebugModbusBuf[4] = mb->regNumH;
		DebugModbusBuf[5] = mb->regNumL;
		DebugModbusBuf[6] = '\0';		
		DebugModbusBuf[7] = msg_len;
		DebugModbusBuf[8] = data_len;
		DebugModbusBuf[9] = msg_cmd;
#endif		
		if((mb->addr != ev->ev_addr) && (mb->addr != BROADCAST)){
			_Error_Handler(__FILE__, __LINE__); //if wrong addr
		}
		//--------------------------------
		//44   03   C0 A8  00 01  B2 BC
		//Addr Cmd  Reg    Num    Crc
		if( (msg_cmd==CMD03) || (msg_cmd==CMD73) )//multy reg read command
		{ 
			if( msg_len < 8 ){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			ev->ev_len = mb->regNumL << 1; //len always doubled
			crc = ModbusCrc16(ModbusBuf, data_len);//calculate crc
			crcptr = &ModbusBuf[data_len];//pointer to crc in the message
#ifdef DEBUGVIEW
			DebugModbusBuf[10] = crc;
			DebugModbusBuf[11] = ev->ev_len;
#endif			
		}
		//--------------------------------
		//44   06   C0 A8  12 34  B2 BC
		//Addr Cmd  Reg    Data   Crc
		else if( (msg_cmd==CMD06) || (msg_cmd==CMD76) )//one reg write command
		{
			if( msg_len < 7 ){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			crc = ModbusCrc16(ModbusBuf, data_len);//calculate crc
			crcptr = &ModbusBuf[data_len];//pointer to crc in the message
			ev->ev_len = 2;
			//if (len==7) {ev->len=1; ev->size=0;} else {ev->len=2; ev->size=1;}//len depends on request len
			ev->dataptr = &ModbusBuf[4];
		}
		//--------------------------------
		//44   10  C0 A8  00 01  02  12 34  B2 BC
		//Addr Cmd Reg    Num    Len Data   Crc
		else if( msg_cmd==CMD10 )//multy reg write command
		{
			if( msg_len<10 ){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			crc = ModbusCrc16(ModbusBuf, data_len);//calculate crc
			crcptr = &ModbusBuf[data_len];//pointer to crc in the message
			//if (cmd10->len == cmd10->num) {ev->len = cmd10->len; ev->size=0;}//len depends on data len
			//else {ev->len = cmd10->num<<1; ev->size=1;}
			ev->ev_len = cmd10->num<<1;
			ev->dataptr = &ModbusBuf[7];
		}
		//-------------------------------
		//44   2B  00  01 02  B2 BC
		//Addr Cmd MEI Data   Crc
		else if (msg_cmd == CMD43)
		{
			if( data_len < 5 ){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			ev->dataptr = &ModbusBuf[2];
			ev->ev_len = 1;
			crc = ModbusCrc16(ModbusBuf, data_len);//calculate crc
			crcptr = &ModbusBuf[data_len];//pointer to crc in the message
		}
		//-------------------------------
		else if (msg_cmd == DEBUG)
		{
			if (data_len < 5){
				_Error_Handler(__FILE__, __LINE__);//short message
			}
			crc = ModbusCrc16(ModbusBuf, data_len);//calculate crc
			crcptr = &ModbusBuf[data_len];//pointer to crc in the message
		}
  //-------------------------------
		else
		{		
			_Error_Handler(__FILE__, __LINE__);//unknown command
		}
	
		if( (crc&0xFF) == *crcptr && (crc>>8) == *(crcptr+1) )//verify crc
		{
			ev->ev_cmd = msg_cmd;
			ev->ev_reg |= mb->regHi;
			ev->ev_reg = ev->ev_reg << 8;
			ev->ev_reg |= mb->regLo;
			if(msg_cmd == DEBUG) ev->ev_debug = ModbusBuf[2];
			if(mb->addr == BROADCAST){
				_Error_Handler(__FILE__, __LINE__);
			}

//    if(ev->debug == NEARLOOP) {Send(); return FALSE;}
//    if (cmd == DEBUG) {Send(); return FALSE;}
		} else {
			_Error_Handler(__FILE__, __LINE__);
		}
		EnableInterrupts();
		OS_Signal(&TelevisHndlSendSema);
	}
}

void ModbusSend(void)//Reply Called by App
{
  uint32_t crc;
  uint8_t msg_cmd, data_len, msg_len;
	ModbusReply03 *r03=(ModbusReply03*)ModbusBuf;
	
	while(1){
		OS_Wait(&ModbusSendSema);
		r03->addr = ev->ev_addr;
		msg_cmd = ev->ev_cmd;
		r03->cmd = msg_cmd;
#ifdef DEBUGVIEW
		DebugModbusBuf[12] = ev->ev_addr;
		DebugModbusBuf[13] = ev->ev_cmd;
#endif
		//--------------------------------
		//44   03   02  00 C2   F4 1A
		//Addr Cmd Len  Data    Crc
		if( (msg_cmd==CMD03) || (msg_cmd==CMD73) )
		{
			if( (data_len = ev->ev_len ) > 20) data_len = 2;
			
			r03->len = data_len;
			//if (ev->reg & 0xC000) x=1; else x=0;
			for(uint32_t i = 0; i < data_len; i++){
				*(&r03->data + (i^1)) = *(ev->dataptr + i);//need to swap bytes
			}
			msg_len = data_len + 3;
#ifdef DEBUGVIEW
		DebugModbusBuf[14] = ev->ev_len;
		DebugModbusBuf[15] = r03->len;
		for(uint32_t i = 0; i < data_len; i++){	
			DebugModbusBuf[16+i] = *(&r03->data + i);	
		}
		DebugModbusBuf[16+data_len] = ':';
		for(uint32_t i = 0; i < data_len; i++){	
			DebugModbusBuf[16+data_len+i] = *(ev->dataptr + i);	
		}
		DebugModbusBuf[16+data_len+data_len] = ':';
#endif		
		}
		//--------------------------------
		//44   06   C0 A8  12 34  B2 BC
		//Addr Cmd  Reg    Data   Crc
		else if( (msg_cmd==CMD06) || (msg_cmd==CMD76) )//Should return the same reply as request
		{
			//ModbusReply06 *r06=(ModbusReply06*)ModbusBuf;
			//r06->reg = ev->reg;
			//for (i=0; i<2; i++) *(&r06->data+i) = *(ev->dataptr+i);
			//if(ev->len == 1) len = 5; else 
			msg_len = 6;
		}
		//--------------------------------
		//44   10   C0 A8  00 01  B2 BC
		//Addr Cmd  Reg    Num    Crc
		else if( msg_cmd==CMD10 ) 
		{
			ModbusReply10 *r10=(ModbusReply10*)ModbusBuf;
			r10->reg = ev->ev_reg;
			r10->num = ev->ev_len;
			msg_len = 6;
		}
		//--------------------------------
		else if( msg_cmd==CMD43 )
		{
			for(uint32_t i=0; i<ev->ev_len; i++){
				ModbusBuf[i+3] = *(ev->dataptr+i);
			}
			msg_len = ev->ev_len+3;
		}
		//--------------------------------
		else {
			msg_len = 1;
		}
		crc = ModbusCrc16(ModbusBuf, msg_len);//Calculate CRC
		ModbusBuf[msg_len++] = crc;//Add CRC to buf
		ModbusBuf[msg_len++] = crc>>8;
		U3_TxMessageSize = msg_len;
		OS_Signal(&ModbusPortTxInitSema);
	}
}

/************************ (C) COPYRIGHT *****END OF FILE****/
