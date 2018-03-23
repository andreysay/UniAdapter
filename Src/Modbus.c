/**
  ******************************************************************************
  * File Name          : Modbus.c
  * Description        : This file provides code for Send/Receive functions
  *                      of the Modbus protocol.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"
#include "main.h"
#include "usart.h"
#include "ControllerType.h"
#include "os.h"
#include "LED.h"
#include "Carel.h"
#include "Modbus.h"

#define stop_cpu1   __breakpoint(1)

// Modbus indexs symbols
#define modbus_address		0
#define modbus_function		1
#define modbus_regHi			2
#define modbus_regLo			3
#define modbus_valHi			4
#define modbus_valLo			5
#define modbus_bytecount	2
#define modbus_startdata	3

#ifdef APDEBUG
uint8_t DebugModbusBuf[64]; // Buffer for debugging purpose
// counters for errors debuging
uint32_t ErrorCMD06_76Modbuslog, ErrorCMDDEBModbuslog, ErrorCMD03_73Modbuslog, ErrorCMD10Modbuslog, ErrorCMD43Modbuslog, ErrorCMDModbuslog;

// Counters for debugging
extern uint32_t Count0;
extern uint32_t Count1;
extern uint32_t Count2;
uint32_t Count9, Count10, Count11;
#endif

// Semaphore will use in ModbusHndlReceive() for thread management
int32_t ModbusHndlReceiveSema;
// Semaphore will use in ModBusSend() for thread management
int32_t ModbusSendSema;
// Semaphore will use in ModbusPortTxInit for thread management
int32_t ModbusPortTxInitSema;

uint32_t ModbusMsgSize;
uint8_t ModbusBuf[ModbusADUSize];

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

// link to Carel.c
extern int32_t CarelHndlSendSema;
extern bool writeACK;
extern bool dataReadyFlag;

// link to main.c 
extern uint32_t ProtocolMsgSize;
extern uint8_t ProtocolBuf[];

/* Private function prototypes -----------------------------------------------*/
static uint32_t ModbusCrc16(const uint8_t *msg_data, uint8_t msg_len);

//***********ModbusPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize MU port for data reception, will signal from TelevisScan() after connected controller address found
void ModbusPortRxInit(void)
{
#ifdef APDEBUG	
	Count0 = 0;
#endif	
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
		/* Enable IDLE */
		LL_USART_EnableIT_IDLE(USART3);
		/* Clear Overrun flag, in case characters have already been sent to USART */
		LL_USART_ClearFlag_ORE(USART3);

		/* Enable RXNE and Error interrupts */
		LL_USART_EnableIT_RXNE(USART3);
		LL_USART_EnableIT_ERROR(USART3);		
		OS_Signal(&U3_RxSemaphore);
#ifdef APDEBUG		
		Count0++;
#endif		
	}
}

//***********ModbusPortReception***************
// returns none
// Inputs: none
// Outputs: none
// Wait for data reception from MU trough USART3, will signal from USART3 ISR trough USART3_IDLE_Callback()
void ModbusPortReception(void)
{
#ifdef APDEBUG	
	Count1 = 0;
#endif	

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
			for(uint8_t i = 0; i < U3_RxMessageSize; i++){
				ModbusBuf[i] = U3_RXBuffer[i];
				U3_RXBuffer[i] = 0;
			}
			/* Turn off green led, indication that data from monitor unit received */	
			LEDs_off();
			// Enable interrupts
			EnableInterrupts();
			OS_Signal(&ModbusHndlReceiveSema);
		}
#ifdef APDEBUG		
		Count1++;
#endif		
	}
}

//***********ModbusPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize variables and fill out buffers for MU port transmittion
void ModbusPortTxInit(void){
#ifdef APDEBUG	
	Count9 = 0;
#endif	
	GPIO_PinState pinDirState;
	
	while(1){
		OS_Wait(&ModbusPortTxInitSema);
		U3_pBufferTransmit = &U3_TXBuffer[0];
		pinDirState = HAL_GPIO_ReadPin(GPIO_Dir, PIN_Dir);
		if(pinDirState != GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_SET);
		}
		DisableInterrupts();
#ifdef APDEBUG		
		DebugModbusBuf[13] = U3_TxMessageSize;
#endif		
		for(uint32_t i = 0; i < U3_TxMessageSize; i++){
			U3_TXBuffer[i] = ModbusBuf[i];
//			ModbusBuf[i] = 0;
		}
		EnableInterrupts();
		OS_Signal(&U3_TxSemaphore);
#ifdef APDEBUG
		Count9++;
#endif		
	}
}

//***********ModbusPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// transmit data to MU device through USART3
void ModbusPortSendMsg(void){
#ifdef APDEBUG	
	Count2 = 0;
#endif
	while(1){
		OS_Wait(&U3_TxSemaphore);
		/* Turn ORANGE On at start of transfer : Tx started */
		LED_OrangeOn();
		/* Fill DR with a new char */
		LL_USART_TransmitData8(USART3, U3_TXBuffer[U3_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART3);
#ifdef APDEBUG		
		Count2++;
#endif		
	}
}

//***********ModbusCrc16***************
// returns the CRC of an Modbus message
// Inputs:  pointer to Modbus message, size of the message
// Outputs: the CRC
// calculated CRC for received Modbus message
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

//***********ModbusHndlReceive***************
// returns none
// Inputs: none
// Outputs: none
// Handle received from MU port Modbus message, will signal from ModbusPortReception()
void ModbusHndlReceive(void)//Request Called by App
{
  ModbusCmd *mb = (ModbusCmd*)ModbusBuf;
  ModbusCmd10 *cmd10 = (ModbusCmd10*)ModbusBuf;
  uint32_t crc;
  uint8_t msg_cmd, *crcptr, data_len, msg_len;
#ifdef APDEBUG	
	Count10 = 0;
#endif	
	
	while(1){
		OS_Wait(&ModbusHndlReceiveSema);
		ev->ev_reg = 0;
		msg_len = ModbusMsgSize;
		data_len = msg_len - 2;
		ModbusMsgSize = 0;
		crc = ModbusCrc16(ModbusBuf, data_len); //calculate crc
		crcptr = &ModbusBuf[data_len]; //pointer to crc in the message
		// Listen for messages addressed only for connected controller or broadcast and valid CRC
		if( ((mb->addr == ev->ev_addr) || (mb->addr == BROADCAST)) && ((crc&0xFF) == *crcptr && (crc>>8) == *(crcptr+1)) ){ 
			DisableInterrupts();		
			msg_cmd = mb->cmd;		
#ifdef APDEBUG		
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
			DebugModbusBuf[10] = crc;
			DebugModbusBuf[11] = mb->regNumL << 1;		
#endif		
			//--------------------------------
			//44   03   C0 A8  00 01  B2 BC
			//Addr Cmd  Reg    Num    Crc
			if( (msg_cmd == CMD03) || (msg_cmd == CMD73) ) //multy reg read command
			{
#ifdef APDEBUG				
				if( msg_len < 8 ){
					ErrorCMD03_73Modbuslog++;//short message
				}
#endif
				ev->ev_len = mb->regNumL << 1; //len always doubled		
			}
			//--------------------------------
			//44   06   C0 A8  12 34  B2 BC
			//Addr Cmd  Reg    Data   Crc
			else if( (msg_cmd == CMD06) || (msg_cmd == CMD76) )//one reg write command
			{
#ifdef APDEBUG				
				if( msg_len < 7 ){
					ErrorCMD06_76Modbuslog++;//short message
				}
#endif				
				ev->ev_len = 2;
				ev->dataptr = &ModbusBuf[4];
			}
			//--------------------------------
			//44   10  C0 A8  00 01  02  12 34  B2 BC
			//Addr Cmd Reg    Num    Len Data   Crc
			else if( msg_cmd == CMD10 ) //multy reg write command
			{
#ifdef APDEBUG				
				if( msg_len < 10 ){
					ErrorCMD10Modbuslog++; //short message
				}
#endif				
				ev->ev_len = cmd10->numLo << 1;
				ev->dataptr = &ModbusBuf[7];
			}
			//-------------------------------
			//44   2B  00  01 02  B2 BC
			//Addr Cmd MEI Data   Crc
			else if( msg_cmd == CMD43 )
			{
#ifdef APDEBUG				
				if( data_len < 5 ){
					ErrorCMD43Modbuslog++; //short message
				}
#endif				
				ev->dataptr = &ModbusBuf[2];
				ev->ev_len = 1;
			}
			//-------------------------------
			else if( msg_cmd == DEBUG )
			{
#ifdef APDEBUG				
				if( data_len < 5 ){
					ErrorCMDDEBModbuslog++;//short message
				}
#endif				
				ev->ev_debug = ModbusBuf[2];
			}
		//-------------------------------
			else
			{
#ifdef APDEBUG				
				ErrorCMDModbuslog++;//unknown command
#endif				
			}
	
			ev->ev_cmd = msg_cmd;
			ev->ev_reg |= mb->regHi;
			ev->ev_reg = ev->ev_reg << 8;
			ev->ev_reg |= mb->regLo;
			EnableInterrupts();
//			if( ev->ev_debug == NEARLOOP ){
//				OS_Signal(&ModbusPortTxInitSema);
//			} else {
				OS_Signal(&TelevisHndlSendSema); // signal TelevisSend() to continue handle message
//			}
		} else {
			OS_Signal(&U3_RxInitSema); // signal ModbusPortRxInit() to listen new message
		}
#ifdef APDEBUG		
		Count10++;
#endif		
	}
}

//***********ModbusCarelHndlRcvd***************
// returns none
// Inputs: none
// Outputs: none
// Handle received from MU port Modbus message, will signal from ModbusPortReception()
void ModbusCarelHndlRcvd(void)
{
  uint32_t crc;
  uint8_t msg_cmd, *crcptr, data_len, msg_len;
#ifdef APDEBUG	
	Count10 = 0;
#endif	
	
	while(1){
		OS_Wait(&ModbusHndlReceiveSema);
		ev->ev_reg = 0;
		msg_len = ModbusMsgSize;
		data_len = msg_len - 2;
		ModbusMsgSize = 0;
		msg_cmd = ModbusBuf[modbus_function];		
		crc = ModbusCrc16(&ModbusBuf[0], data_len); //calculate crc
		crcptr = &ModbusBuf[data_len]; //pointer to crc in the message
		// Listen for messages addressed only for connected controller or broadcast and valid CRC
		if( ((ModbusBuf[modbus_address] == ev->ev_addr) || (ModbusBuf[modbus_address] == BROADCAST)) && ((crc&0xFF) == *crcptr && (crc>>8) == *(crcptr+1)) ){ 
			DisableInterrupts();
			if( dataReadyFlag ){
				ev->ev_cmd = msg_cmd;
				ev->ev_reg = ( ModbusBuf[modbus_regHi] << 8 ) | ModbusBuf[modbus_regLo]; // save start register index
				if( (msg_cmd != DEBUG) && (msg_cmd != RCOIL) && (msg_cmd != W1COIL) ){
					if( ev->ev_reg > 0 && ev->ev_reg < 128 ){
						ev->ev_vartype = ANA_VAR;
					} else if( ev->ev_reg > 127 && ev->ev_reg < 255 ){
						ev->ev_vartype = INT_VAR;
					} else if(ev->ev_reg > 10000 && ev->ev_reg < 10039){
						ev->ev_vartype = DIG_VAR;
					} else if(ev->ev_reg == HWidREG){
						ev->ev_vartype = HW_VAR;
					} else { // Prepare exception
						ModbusBuf[0] = ev->ev_addr;
						ModbusBuf[1] = 0x80 + msg_cmd;
						ModbusBuf[2] = 0x02; // ILLEGAL DATA ADDRESS
						msg_len = 3;
						crc = ModbusCrc16(ModbusBuf, msg_len); //Calculate CRC
						ModbusBuf[msg_len++] = crc; //Add CRC to buf
						ModbusBuf[msg_len++] = crc >> 8;
						ev->ev_reg = NULL;
					}
				} else if(msg_cmd == RCOIL || msg_cmd == W1COIL ) { // this comparision block for compatibility with old software, server sends reques directly to coil variable, no offset
					ev->ev_vartype = DIG_VAR;
				}
				if( msg_cmd == RCOIL ){ //Read coils
					// save quantity registers to read	
					ev->ev_len = ( ModbusBuf[modbus_valHi] << 8 ) | ModbusBuf[modbus_valLo];
					if( ev->ev_len < 0x0001 || ev->ev_len > 0x07D0 ){
						ModbusBuf[0] = ev->ev_addr;
						ModbusBuf[1] = 0x80 + msg_cmd;
						ModbusBuf[3] = 0x03; // ILLEGAL DATA VALUE
						msg_len = 3;
						crc = ModbusCrc16(ModbusBuf, msg_len); //Calculate CRC
						ModbusBuf[msg_len++] = crc; //Add CRC to buf
						ModbusBuf[msg_len++] = crc >> 8;						
						ev->ev_cmd = NULL;
					}
				
				} else if( msg_cmd == W1COIL ){ //one coil write command
					// save register value
					ev->ev_regvalue = ( ModbusBuf[modbus_valHi] << 8 ) | ModbusBuf[modbus_valLo];
				}
				else if( msg_cmd == CMD03 ){ //multy reg read command
					// save quantity registers to read
					ev->ev_len = ( ModbusBuf[modbus_valHi] << 8 ) | ModbusBuf[modbus_valLo]; 
				
				} else if( msg_cmd == CMD06 ){ // one register write command		
					// save register value
					ev->ev_regvalue = ( ModbusBuf[modbus_valHi] << 8 ) | ModbusBuf[modbus_valLo];

				} else if( msg_cmd == DEBUG ){
				

				} else { // Prepare exception
					ModbusBuf[0] = ev->ev_addr;
					ModbusBuf[1] = 0x80 + msg_cmd;
					ModbusBuf[2] = 0x01; // ILLEGAL FUNCTION
					msg_len = 3;
					crc = ModbusCrc16(ModbusBuf, msg_len); //Calculate CRC
					ModbusBuf[msg_len++] = crc; //Add CRC to buf
					ModbusBuf[msg_len++] = crc >> 8;						
					ev->ev_cmd = NULL;
				}
			} else { // Exception when device not equired
				ModbusBuf[0] = ev->ev_addr;
				ModbusBuf[1] = 0x80 + msg_cmd;
				ModbusBuf[2] = 0x06; // SERVER DEVICE BUSY
				msg_len = 3;
				crc = ModbusCrc16(ModbusBuf, msg_len); //Calculate CRC
				ModbusBuf[msg_len++] = crc; //Add CRC to buf
				ModbusBuf[msg_len++] = crc >> 8;				
				ev->ev_cmd = NULL;
			}
			EnableInterrupts();
			if( (!ev->ev_reg) || (!ev->ev_cmd) ){
				U3_TxMessageSize = msg_len;
				OS_Signal(&ModbusPortTxInitSema);
			} else {
				OS_Signal(&CarelHndlSendSema);
			}
		} else {
			OS_Signal(&U3_RxInitSema); // signal ModbusPortRxInit() to listen new message
		}
#ifdef APDEBUG		
		Count10++;
#endif		
	}
}

//***********ModbusSend***************
// returns none
// Inputs: none
// Outputs: none
// Prepare Modbus message to send, thread will signal from TelevisHndlReceive()
void ModbusSend(void)//Reply Called by App
{
  uint32_t crc;
  uint8_t msg_cmd, data_len, msg_len;
	ModbusReply03 *r03 = (ModbusReply03*)ModbusBuf;
#ifdef APDEBUG	
	Count11 = 0;
#endif	
	
	while(1){
		OS_Wait(&ModbusSendSema);
		r03->addr = ev->ev_addr;
		msg_cmd = ev->ev_cmd;
		r03->cmd = msg_cmd;
#ifdef APDEBUG
		DebugModbusBuf[12] = ev->ev_addr;
		DebugModbusBuf[13] = ev->ev_cmd;
#endif
		//--------------------------------
		//44   03   02  00 C2   F4 1A
		//Addr Cmd Len  Data    Crc
		if( (msg_cmd == CMD03) || (msg_cmd == CMD73) )
		{
			if( (data_len = ev->ev_len ) > 20) data_len = 2;
			
			r03->len = data_len;
			//if (ev->reg & 0xC000) x=1; else x=0;
			for(uint32_t i = 0; i < data_len; i++){
				*(&r03->data + (i^1)) = *(ev->dataptr + i);//need to swap bytes
			}
			msg_len = data_len + 3;
#ifdef APDEBUG
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
		else if( (msg_cmd == CMD06) || (msg_cmd == CMD76) )//Should return the same reply as request
		{
			msg_len = 6;
		}
		//--------------------------------
		//44   10   C0 A8  00 01  B2 BC
		//Addr Cmd  Reg    Num    Crc
		else if( msg_cmd == CMD10 ) 
		{
			ModbusReply10 *r10=(ModbusReply10*)ModbusBuf;
			r10->regHi = (ev->ev_reg >> 8);
			r10->regLo = (ev->ev_reg & 0xFF);
			r10->numHi = (ev->ev_len >> 8);
			r10->numLo = (ev->ev_len & 0xFF);
			msg_len = 6;
		}
		//--------------------------------
		else if( msg_cmd == CMD43 )
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
		crc = ModbusCrc16(ModbusBuf, msg_len); //Calculate CRC
		ModbusBuf[msg_len++] = crc; //Add CRC to buf
		ModbusBuf[msg_len++] = crc >> 8;
		U3_TxMessageSize = msg_len;
		OS_Signal(&ModbusPortTxInitSema);
#ifdef APDEBUG		
		Count11++;
#endif		
	}
}

//***********ModbusSendCarel***************
// returns none
// Inputs: none
// Outputs: none
// Prepare Modbus message to send, thread will signal from CarelHndlReceive()
void ModbusSendCarel(void)
{
  uint32_t crc;
	uint16_t i, j;
  uint8_t msg_cmd, data_len, msg_len;
#ifdef APDEBUG	
	Count11 = 0;
#endif	
	
	while(1){
		OS_Wait(&ModbusSendSema);
		ModbusBuf[modbus_address] = ev->ev_addr;
		ModbusBuf[modbus_function] = ev->ev_cmd;
		msg_cmd = ev->ev_cmd;
		
		if( msg_cmd == RCOIL ){
			ModbusBuf[modbus_bytecount] = ev->ev_len;
			data_len = modbus_startdata + ModbusBuf[modbus_bytecount];
			for(i = modbus_startdata, j = 0; i < data_len; i += 2, j++){
				ModbusBuf[i] = ev->dataptr[j];
			}
			free(ev->dataptr); // free memory allocated in corresponding section CarelSend function, Carel.c file
			msg_len = data_len;			
		} else if( msg_cmd == CMD03 ){
			ModbusBuf[modbus_bytecount] = ev->ev_len * 2;
			data_len = modbus_startdata + ModbusBuf[modbus_bytecount];
			for(i = modbus_startdata, j = 0; i < data_len; i++, j++){
				ModbusBuf[i] = ev->dataptr[j];
			}
			free(ev->dataptr); // free memory allocated in corresponding section CarelSend function, Carel.c file
			msg_len = data_len;
		}
		else if( msg_cmd == CMD06 || msg_cmd == W1COIL )//Should return the same reply as request
		{
			if(writeACK){
				msg_len = 6;
				writeACK = false; // Set false for next device write operation
			} else {
				if( msg_cmd == CMD06 ){
					ModbusBuf[0] = 0x86; // Exception
					ModbusBuf[1] = 0x04; // SLAVE DEVICE FAILURE
					msg_len = 2;
				} else if( msg_cmd == W1COIL ){
					ModbusBuf[0] = 0x85; // Exception
					ModbusBuf[1] = 0x04; // SLAVE DEVICE FAILURE
					msg_len = 2;
				} else {
					ModbusBuf[0] = '?';
					ModbusBuf[1] = '?';
					ModbusBuf[2] = '?';
					msg_len = 3;
				}
			}
		}
		else { // Exception
				ModbusBuf[0] = 0x80 + msg_cmd;
				ModbusBuf[1] = 0x01; // ILLEGAL FUNCTION
				msg_len = 2;
		}
		crc = ModbusCrc16(ModbusBuf, msg_len); //Calculate CRC
		ModbusBuf[msg_len++] = crc; //Add CRC to buf 
		ModbusBuf[msg_len++] = crc >> 8;
#ifdef APDEBUG
		if( msg_cmd == W1COIL ){
			for(uint16_t i = 0; i < msg_len; i++){
				DebugModbusBuf[i] = ModbusBuf[i];
			}
		}
#endif
		U3_TxMessageSize = msg_len;
		OS_Signal(&ModbusPortTxInitSema);
#ifdef APDEBUG		
		Count11++;
#endif		
	}
}

/************************ (C) COPYRIGHT *****END OF FILE****/
