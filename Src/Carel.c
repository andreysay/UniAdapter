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
#include <stdlib.h>
#include "stm32f1xx_ll_usart.h"
#include "Command.h"
#include "tim.h"
#include "os.h"
#include "ControllerType.h"
#include "ControllerPort.h"
#include "LED.h"
#include "main.h"
#include "StartUp.h"
#include "Carel.h"

#ifdef APDEBUG
// Counters for debugging
extern uint32_t Count0;
extern uint32_t Count1;
extern uint32_t Count2;
//extern uint32_t Count3;
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
extern int32_t U3_RxInitSema;

extern uint8_t U1_RXBufferA[]; // defined in main_threads.c file
extern uint8_t U1_TXBuffer[]; // defined in main_threads.c file

// link Modbus.c file
extern int32_t ModbusSendSema;

// Semaphores
int32_t CarelHndlReceiveSema;
int32_t CarelHndlSendSema;
int32_t CarelReadSema;
int32_t ReadCarelENQSema;
int32_t ReadCarelACKSema;
int32_t CarelScanHndlSema;

// Local flags
static bool deviceScanFlag = false;
static bool sendENQ_Flag = false;
static bool sendACK_Flag = false;
static bool sendDREQ_Flag = false;
static bool sendF_Flag = false;
static bool sendWrite_Flag = false;

// local variables
static 	uint32_t idxFront, idxBack;

// Global flags
bool writeACK = true;
bool dataReadyFlag = false;

// Carel indexs symbols
#define carel_header	 					0 // SOF(header) index
#define carel_address						1 // sender address index
#define carel_REQ								2 // cmd index
#define carel_vartype						2	// variable type index
#define carel_varidx						3 // variable device index
#define carel_regs							3	// index for carel variable index
#define carel_etx								4 // index end of text symbol
#define carel_start_data				4
#define carel_val4							4
#define carel_val3							5
#define carel_val2							6
#define carel_val1							7

// Indexes for CAREL_DEVICE_PARAM array
#define carel_dev_hwid		0 // Index where store hardware id device
#define carel_dev_dat1 		1 // Unknown data field
#define carel_dev_dat2 		2 // Index where store hardware id device
#define carel_dev_fwid		3 // Index where store firmware id of device


// Carel protocol symbols
#define PJEASY_VAR_ARRAY_SIZE		510
#define DIG_ARRAY_SIZE 		199
#define INT_ARRAY_SIZE 		208
#define ANA_ARRAY_SIZE 		208
#define IDX_ANA_MAP_SIZE	35
#define IDX_INT_MAP_SIZE	108
#define IR33IDX_INT_MAP_SIZE	134
#define IDX_DIG_MAP_SIZE	39
#define DEVICE_PARAM_SIZE 16
#define DATA_MPX_MAP_SIZE 25

// Carel hardware ID
#define CAREL_PJEZ	173
#define CAREL_IR33	147

// Array to store DIG(digital variable) values from Carel controller. Array indexies correspond digit variable index by formula variable_index = array_index + 1
int16_t DIG_ARRAY[DIG_ARRAY_SIZE];
// Array to store INT(integer variable) values from Carel controller Array indexies correspond indeger variable index by formula variable_index = array_index + 1
int32_t INT_ARRAY[INT_ARRAY_SIZE];
// Array to store ANA(analog variable) values from Carel controller Array indexies correspond analog variable index by formula variable_index = array_index + 1
int32_t ANA_ARRAY[ANA_ARRAY_SIZE];

//int32_t PJEASY_VAR_ARRAY[PJEASY_VAR_ARRAY_SIZE];

int32_t CAREL_DEVICE_PARAM[DEVICE_PARAM_SIZE];

// Carel - Modbus analog variables index relation
// const two_idx_struct carel_modbus_ANA_mapping[IDX_ANA_MAP_SIZE] = { 	{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5},
//																																				{6, 6}, {7, 7}, {8, 8}, {9, 9}, {10, 10}, {11, 11},
//																																				{12, 12}, {13, 13}, {14, 14}, {15, 15}, {16, 16}, {17, 17},
//																																				{18, 18}, {19, 19}, {20, 20}, {21, 21}, {22, 22}, {23, 23},
//																																				{24, 24}, {25, 25}, {26, 26}, {27, 27}, {28, 28}, {29, 29},
//																																				{30, 30}, {31, 31}, {32, 32}, {33, 33}, {34, 34}};
// Carel - Modbus integer variables index relation 
const two_idx_struct carel_modbus_INT_mapping[IDX_INT_MAP_SIZE] = { {0, 0}, {1, 129}, {2, 130}, {3, 131}, {4, 132}, {5, 133},
																																			{6, 134}, {7, 135}, {8, 136}, {9, 137}, {10, 138}, {11, 139},
																																			{12, 140}, {13, 141}, {14, 142}, {15, 143}, {16, 144}, {17, 145},
																																			{18, 146}, {19, 147}, {20, 148}, {21, 149}, {22, 150}, {23, 151},
																																			{24, 152}, {25, 153}, {26, 154}, {27, 155}, {28, 156}, {29, 157},
																																			{30, 158}, {31, 159}, {32, 160}, {33, 161}, {34, 162}, {35, 163},
																																			{36, 164}, {37, 165}, {38, 166}, {39, 167}, {40, 168}, {41, 169},
																																			{42, 170}, {43, 171}, {44, 172}, {45, 173}, {46, 174}, {47, 175},
																																			{48, 176}, {49, 177}, {50, 178}, {51, 179}, {52, 180}, {53, 181},
																																			{54, 182}, {55, 183}, {56, 184}, {57, 185}, {58, 186}, {59, 187},
																																			{60, 188}, {61, 189}, {62, 190}, {63, 191}, {64, 192}, {65, 193},
																																			{66, 194}, {67, 195}, {68, 196}, {69, 197}, {70, 198}, {71, 199},
																																			{72, 200}, {73, 201}, {74, 202}, {75, 203}, {76, 204}, {77, 205},
																																			{78, 206}, {79, 207}, {80, 208}, {81, 209}, {82, 210}, {83, 211},
																																			{84, 212}, {85, 213}, {86, 214}, {87, 215}, {88, 216}, {89, 217},
																																			{90, 218}, {91, 219}, {92, 220}, {93, 221}, {94, 222}, {95, 223},
																																			{96, 224}, {97, 225}, {98, 226}, {99, 227}, {100, 228}, {101, 229},
																																			{102, 230}, {103, 231}, {104, 232}, {105, 233}, {106, 234}
};
// Carel IR33 - Modbus integer variables index relation 
const two_idx_struct carelIR33_modbus_INT_mapping[IR33IDX_INT_MAP_SIZE] = { {0, 0}, {1, 101}, {2, 102}, {3, 103}, {4, 104}, {5, 105},
																																					{6, 106}, {7, 107}, {8, 108}, {9, 109}, {10, 110}, {11, 111},
																																					{12, 112}, {13, 113}, {14, 114}, {15, 115}, {16, 116}, {17, 117},
																																					{18, 118}, {19, 119}, {20, 120}, {21, 121}, {22, 122}, {23, 123},
																																					{24, 124}, {25, 125}, {26, 126}, {27, 127}, {28, 128}, {29, 129},
																																					{30, 130}, {31, 131}, {32, 132}, {33, 133}, {34, 134}, {35, 135},
																																					{36, 136}, {37, 137}, {38, 138}, {39, 139}, {40, 140}, {41, 141},
																																					{42, 142}, {43, 143}, {44, 144}, {45, 145}, {46, 146}, {47, 147},
																																					{48, 148}, {49, 149}, {50, 150}, {51, 151}, {52, 152}, {53, 153},
																																					{54, 154}, {55, 155}, {56, 156}, {57, 157}, {58, 158}, {59, 159},
																																					{60, 160}, {61, 161}, {62, 162}, {63, 163}, {64, 164}, {65, 165},
																																					{66, 166}, {67, 167}, {68, 168}, {69, 169}, {70, 170}, {71, 171},
																																					{72, 172}, {73, 173}, {74, 174}, {75, 175}, {76, 176}, {77, 177},
																																					{78, 178}, {79, 179}, {80, 180}, {81, 181}, {82, 182}, {83, 183},
																																					{84, 184}, {85, 185}, {86, 186}, {87, 187}, {88, 188}, {89, 189},
																																					{90, 190}, {91, 191}, {92, 192}, {93, 193}, {94, 194}, {95, 195},
																																					{96, 196}, {97, 197}, {98, 198}, {99, 199}, {100, 200}, {101, 201},
																																					{102, 202}, {103, 203}, {104, 204}, {105, 205}, {106, 206}, {107, 207},
																																					{108, 208}, {109, 209}, {110, 210}, {111, 211}, {112, 212}, {113, 213},
																																					{114, 214}, {115, 215}, {116, 216}, {117, 217}, {118, 218}, {119, 219},
																																					{120, 220}, {121, 221}, {122, 222}, {123, 223}, {124, 224}, {125, 225},
																																					{126, 226}, {125, 225}, {126, 226}, {127, 227}, {128, 228}, {129, 229},
																																					{130, 230}, {131, 231}																																				
																																};
// Carel - Modbus digital variables index relation
//const two_idx_struct carel_modbus_DIG_mapping[IDX_DIG_MAP_SIZE] = { {0, 0}, {1, 10001}, {2, 10002}, {3, 10003}, {4, 10004}, {5, 10005}, {6, 10006},
//																																			{7, 10007}, {8, 10008}, {9, 10009}, {10, 10010}, {11, 10011}, {12, 10012},
//																																			{13, 10013}, {14, 10014}, {15, 10015}, {16, 10016}, {17, 10017}, {18, 10018},
//																																			{19, 10019}, {20, 10020}, {21, 10021}, {22, 10022}, {23, 10023}, {24, 10024},
//																																			{25, 10025}, {26, 10026}, {27, 10027}, {28, 10028}, {29, 10029}, {30, 10030},
//																																			{31, 10031}, {32, 10032}, {33, 10033}, {34, 10034}, {35, 10035}, {36, 10036},
//																																			{37, 10037}, {38, 10038} };
// Structure which holds integer value and corresponding char value 
typedef struct int_to_char {
	uint8_t integer_value;
	uint8_t char_value;
} int_to_char;

// Array integer numbers and corresponding HEX values
const int_to_char int_to_char_array[] = { {0, '0'}, {1, '1'},
																		{2, '2'}, {3, '3'},
																		{4, '4'}, {5, '5'},
																		{6, '6'}, {7, '7'},
																		{8, '8'}, {9, '9'},
																		{10, 'A'}, {11, 'B'},
																		{12, 'C'}, {13, 'D'},
																		{14, 'E'}, {15, 'F'}
};

typedef struct char_int_idx {
	char letter;
	uint16_t idx;
} var_type_struct;

typedef struct data_MPX_map {
	uint16_t enq_idx; // variables block index
	var_type_struct var_type_idx[4]; // structures array which holds: variable type, index, relative position in data R/W requests 0, 4, 8, 12
} data_MPX_map;

const data_MPX_map data_MPX_mapping_array[DATA_MPX_MAP_SIZE] = { // array contain enquire indexes and variables type and corresponding variable index
//																									{ 0x45, { {'D', 1},  {'D', 2},  {'D', 3},  {'D', 4} } },
																									{ 0x45, { {'D', 1},  {'?', 0},  {'?', 0},  {'?', 0} } },
//																									{ 0x46, { {'D', 5},  {'D', 6},  {'D', 7},  {'D', 8} } },
																									{ 0x46, { {'D', 21},  {'?', 0},  {'?', 0},  {'?', 0} } },
																									{ 0x47, { {'A', 1},  {'A', 2},  {'A', 3},  {'A', 4} } },
																									{ 0x4F, { {'D', 60}, {'?', 0},  {'?', 0},  {'?', 0} } },
																									{ 0x50, { {'A', 7},  {'A', 8},  {'I', 9},  {'I', 10} } },
																									{ 0x51, { {'I', 11}, {'I', 12}, {'I', 13}, {'I', 14} } },
																									{ 0x52, { {'A', 15}, {'I', 16}, {'A', 17}, {'I', 18} } },
																									{ 0x53, { {'A', 19}, {'I', 20}, {'I', 21}, {'I', 22} } },
																									{ 0x54, { {'A', 23}, {'I', 24}, {'I', 25}, {'I', 26} } },
																									{ 0x55, { {'A', 27}, {'A', 28}, {'I', 29}, {'I', 30} } },
																									{ 0x56, { {'I', 31}, {'I', 32}, {'I', 33}, {'I', 34} } },
																									{ 0x57, { {'I', 35}, {'I', 36}, {'I', 37}, {'I', 38} } },
																									{ 0x58, { {'I', 40}, {'I', 41}, {'I', 42}, {'I', 43} } },
																									{ 0x59, { {'I', 44}, {'I', 45}, {'I', 46}, {'A', 46} } },
																									{ 0x5A, { {'A', 47}, {'I', 49}, {'A', 49}, {'I', 52} } },
																									{ 0x5B, { {'?',0x0}, {'I', 53}, {'I', 54}, {'I', 55} } },
																									{ 0x5C, { {'I', 56}, {'I', 57}, {'I', 58}, {'I', 59} } },
																									{ 0x5D, { {'I', 60}, {'I', 61}, {'A', 61}, {'I', 63} } },
																									{ 0x5E, { {'I', 64}, {'I', 65}, {'I', 66}, {'I', 67} } },
																									{ 0x5F, { {'I', 68}, {'I', 69}, {'I', 70}, {'I', 71} } },
																									{ 0x60, { {'I', 72}, {'I', 73}, {'I', 74}, {'I', 75} } },
																									{ 0x61, { {'I', 76}, {'I', 77}, {'I', 78}, {'I', 79} } },
																									{ 0x62, { {'I', 80}, {'I', 81}, {'I', 82}, {'I', 83} } },
																									{ 0x63, { {'I', 84}, {'I', 85}, {'I', 86}, {'I', 87} } },
																									{ 0x64, { {'I', 88}, {'I', 89}, {'I', 90}, {'I', 91} } }
};

//const int8_t MPX_ANA_VAR_OFFSET[64] = { [0 ... 6] = -1, [7] = 0, [8] = 1, [9 ... 14] = -1, [15] = 0, [16] = -1,
//																				 [17] = 2, [18] = -1, [19] = 0, [20 ... 22] = -1, [23] = 0, [24 ... 48] = -1,
//																				 [49] = 2, [50 ... 60] = -1,  [61] = 2 };

const three_idx_struct digital_variable_idx_bit_offset[65] = {	{0x00, 0, 0}, 
																															{0x45, 8, 0}, {0x45, 9, 0}, {0x45, 10, 0}, {0x45, 11, 0},
																															{0x45, 8, 2}, {0x45, 9, 2}, {0x45, 10, 2}, {0x45, 11, 2},
																															{0x45, 12, 2}, {0x45, 13, 2}, {0x45, 14, 2}, {0x45, 15, 2},
																															{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
																															{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
																															{0x46, 8, 0}, {0x46, 9, 0}, {0x46, 10, 0}, {0x46, 11, 0},
																															{0x46, 12, 0}, {0x46, 13, 0}, {0x46, 14, 0}, {0x46, 15, 0},
																															{0x46, 8, 1}, {0x46, 9, 1}, {0x46, 10, 1}, {0x46, 11, 1},
																															{0x46, 12, 1}, {0x46, 13, 1}, {0x46, 14, 1}, {0x46, 15, 1},
																															{0x46, 8, 2}, {0x46, 9, 2}, {0x46, 10, 2}, {0x46, 11, 2},
																															{0x46, 12, 2}, {0x46, 13, 2}, {0x46, 14, 2}, {0,0,0},
																															{0x46, 8, 3}, {0x46, 9, 3}, {0x46, 10, 3}, {0x46, 11, 3},
																															{0x46, 12, 3}, {0,0,0}, {0,0,0}, {0,0,0},
																															{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0},
																															{0,0,0}, {0,0,0}, {0,0,0}, {0x4F, 8, 0},
																															{0x4F, 0, 0}, {0x4F, 8, 1}, {0x4F, 0, 1}, {0x4F, 8, 2}
};

//*************message and message fragments**********
// Device request command 
const uint8_t CarelScanFrame[] = {
	STX, // Carel SOF header
	0x00, // Receiver address
	DREQ, // Device request command 
	ETX, // 
	0x00, // CRC_Hi
	0x00 // CRC_Lo
};

// local functios definition
static void sendMPXRead(void);


//***********CarelCRC***************
// returns none
// Inputs: pointer to buffer, buffer size
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

//***********CarelCRC_check***************
// returns none
// Inputs:  pointer to buffer, buffer size
// Outputs: none
// Check CRC from Carel controller message, return true if check successfull.
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

//***********CarelEasyPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer.
void CarelEasyPortTxInit(void)
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

//***********CarelMPXPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer.
GPIO_PinState pin_EnaRx_State;
GPIO_PinState pin_EnaTx_State;
void CarelMPXPortTxInit(void)
{
#ifdef APDEBUG	
	Count5 = 0;
#endif	

	
	while(1){
		OS_Wait(&PortCtrlTxInitSema);
		pin_EnaRx_State = HAL_GPIO_ReadPin(GPIO_EnaRx, PIN_EnaRx);
		pin_EnaTx_State = HAL_GPIO_ReadPin(GPIO_EnaTx, PIN_EnaTx);
		if( pin_EnaRx_State != GPIO_PIN_SET ){
			HAL_GPIO_WritePin(GPIO_EnaRx, PIN_EnaRx, GPIO_PIN_SET);
		}
		if( pin_EnaTx_State != GPIO_PIN_SET ){
			HAL_GPIO_WritePin(GPIO_EnaTx, PIN_EnaTx, GPIO_PIN_SET);
		}
//		LL_USART_DisableDirectionRx(USART1);
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
			if( U1_BufferReadyIndication != 0 ){
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
				if( !DeviceFound ){
					OS_Signal(&CtrlScanSema); // signal CarelScan() in case SCAN process
				} else if(sendACK_Flag){
					OS_Signal(&ReadCarelACKSema);
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
  uint8_t msg_cmd;

	while(1){
		OS_Wait(&CarelHndlReceiveSema);
		msg_cmd = ev->ev_cmd;
		DisableInterrupts();
		if( ( msg_cmd == CMD06 || msg_cmd == W1COIL ) && ProtocolBuf[0] == ACK){
			writeACK = true; // signal that after successfull device write operation ACK received, so we can send responce to MU
			OS_Signal(&ModbusSendSema); // Signal ModbusSend() to transmit message for MU device
		} else if(!DeviceFound && msg_cmd == DREQ && deviceScanFlag){
			OS_Signal(&CarelScanHndlSema);
		} else if(DeviceFound && msg_cmd == DREQ && sendDREQ_Flag){ 
			OS_Signal(&ReadCarelACKSema);
		} else if(DeviceFound && msg_cmd == FREQ && sendF_Flag){	
			OS_Signal(&ReadCarelACKSema);
		} else if(DeviceFound && msg_cmd == ENQ && sendENQ_Flag){				
			OS_Signal(&ReadCarelACKSema);
		} else if(DeviceFound && msg_cmd == RREQ && sendDREQ_Flag){				
			OS_Signal(&ReadCarelACKSema);
		} else if(DeviceFound && msg_cmd == TREQ && sendF_Flag){				
			OS_Signal(&ReadCarelACKSema);
		} else if(DeviceFound && msg_cmd == RREQ && sendENQ_Flag){				
			OS_Signal(&ReadCarelACKSema);
		} else if(DeviceFound && msg_cmd == RREQ && sendWrite_Flag ){
			OS_Signal(&CarelHndlSendSema);
		}
		else {
			OS_Signal(&ModbusSendSema); // Signal ModbusSend() to transmit message for MU device
		}		
		EnableInterrupts();
	}		
}

//***********CarelSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Carel, send write command directly to Carel device, return preread values for read modbus requests.
void CarelSend(void)
{
  uint8_t msg_cmd, msg_len, var_idx;
	uint16_t i, j, carel_wvalue_mask = 0x000F;

	while(1){
		OS_Wait(&CarelHndlSendSema);
		DisableInterrupts();
		ProtocolBuf[carel_header] = STX;
		ProtocolBuf[carel_address] = ev->ev_carel_addr;
		msg_cmd = ev->ev_cmd;
		var_idx = ev->ev_reg; // server sends reques directly to a variable
		
		if( msg_cmd == RCOIL ){
			ev->dataptr = (uint8_t *)malloc( sizeof(uint8_t) * ev->ev_len );
			for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j++){
				ev->dataptr[j] = DIG_ARRAY[i];
			}			
		} else if( msg_cmd == W1COIL ){ //one coil write command
			ProtocolBuf[carel_vartype] = ev->ev_vartype;
			ProtocolBuf[carel_varidx] = var_idx + 0x30;
			ProtocolBuf[carel_varidx + 1] = ev->ev_regvalue + 0x30; 
			ProtocolBuf[carel_varidx + 2] = ETX;
			msg_len = 8;
		}	else if( msg_cmd == CMD03 ){
			// allocate memory for registers data, will free in CMD03 section ModbusSendCarel function, Modbus.c file
			ev->dataptr = (uint8_t *)malloc(sizeof(uint8_t) * ev->ev_len * 2);
			
			if(ev->ev_vartype == ANA_VAR){
				for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j += 2){
					ev->dataptr[j] = ( ANA_ARRAY[i] >> 8 );
					ev->dataptr[j+1] = ( ANA_ARRAY[i] & 0xFF );
				}
			} else if(ev->ev_vartype == INT_VAR){
				if(CAREL_DEVICE_PARAM[0] == CAREL_PJEZ){
					for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j += 2){
						ev->dataptr[j] = ( INT_ARRAY[i] >> 8 );
						ev->dataptr[j+1] = ( INT_ARRAY[i] & 0xFF );
					}
				} else if(CAREL_DEVICE_PARAM[0] == CAREL_IR33){
					for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j += 2){
						ev->dataptr[j] = ( INT_ARRAY[i] >> 8 );
						ev->dataptr[j+1] = ( INT_ARRAY[i] & 0xFF );
					}
				} else {
					/* Nothing to do! */
				}					
			} else if( ev->ev_vartype == HW_VAR ){
					ev->dataptr[0] = ( CAREL_DEVICE_PARAM[carel_dev_hwid] >> 8);
					ev->dataptr[1] = ( CAREL_DEVICE_PARAM[carel_dev_hwid] & 0xFF );
			} else {
				
			}
		} else if( msg_cmd == CMD06 ){
			ProtocolBuf[carel_vartype] = ev->ev_vartype;			
			ProtocolBuf[carel_varidx] = var_idx + 0x30;
			ProtocolBuf[carel_val4] = int_to_char_array[( (ev->ev_regvalue >> 12) & carel_wvalue_mask )].char_value;
			ProtocolBuf[carel_val3] = int_to_char_array[( (ev->ev_regvalue >> 8)  & carel_wvalue_mask )].char_value;
			ProtocolBuf[carel_val2] = int_to_char_array[( (ev->ev_regvalue >> 4)  & carel_wvalue_mask )].char_value;
			ProtocolBuf[carel_val1] = int_to_char_array[(  ev->ev_regvalue        & carel_wvalue_mask)].char_value;
			ProtocolBuf[carel_val1 + 1] = ETX;
			msg_len = 11;
		} else {
			
		}

		EnableInterrupts();
		if( msg_cmd == RCOIL || msg_cmd == CMD03 ){
			OS_Signal(&ModbusSendSema);
		} else {
			CarelCRC(ProtocolBuf, msg_len);
			U1_TxMessageSize = msg_len;
			OS_Signal(&PortCtrlTxInitSema);
		}
	}
}

//***********CarelMPXSend***************
// returns none
// Inputs: none
// Outputs: none
// Convert message from Modbus to Carel, send write command directly to Carel device, return preread values for read modbus requests.
void CarelMPXSend(void)
{
  uint8_t msg_cmd, msg_len, var_idx, reg_idx;
	static uint8_t msg_cmd_stored;
	uint16_t i, j, carel_wvalue_mask = 0x000F;

	while(1){
		OS_Wait(&CarelHndlSendSema);
		DisableInterrupts();
		ProtocolBuf[carel_header] = STX;
		ProtocolBuf[carel_address] = ev->ev_carel_addr;
		msg_cmd = ev->ev_cmd;
		var_idx = ev->ev_reg; // server sends reques directly to a variable

		if( msg_cmd == RCOIL ){
			ev->dataptr = (uint8_t *)malloc( sizeof(uint8_t) * ev->ev_len );
 			for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j++){
				ev->dataptr[j] = DIG_ARRAY[i];
			}			
		} else if(  msg_cmd == W1COIL  ){
			msg_cmd_stored = ev->ev_cmd;
			sendWrite_Flag = true;
			sendENQ_Flag = false;
			sendF_Flag = false;
			sendACK_Flag = false;
			sendDREQ_Flag = false;
			idxFront = 0x4F;
			idxBack = 0x50;
			sendMPXRead();
		}
		else if( msg_cmd == CMD06 ){ //one reg write command
			msg_cmd_stored = ev->ev_cmd;
			sendWrite_Flag = true;
			sendENQ_Flag = false;
			sendF_Flag = false;
			sendACK_Flag = false;
			sendDREQ_Flag = false;
			idxFront = 0;
			reg_idx = i = j = 0; // index variables	
			for(; i < DATA_MPX_MAP_SIZE; i++){
				j = 0;
				for(; j < 4; j++){
					if(data_MPX_mapping_array[i].var_type_idx[j].idx == var_idx && data_MPX_mapping_array[i].var_type_idx[j].letter == ev->ev_vartype ){
						idxFront = data_MPX_mapping_array[i].enq_idx;
						idxBack = data_MPX_mapping_array[i].enq_idx + 1; 
						break;
					}
				}
				if(idxFront){
					break;
				}
			}
			if(idxFront == 0x47){
				idxBack = 0x4A;
			} else if(idxFront == 0x4A){
				idxBack = 0x4F;
			} else if(idxFront == 0x64){
				idxBack = 0x64;
			}
			else {
				/* Nothing to do! */
			}			
			sendMPXRead();
		} else if( msg_cmd == RREQ ){
			sendWrite_Flag = false;
			sendENQ_Flag = false;
			sendF_Flag = false;
			sendACK_Flag = false;
			sendDREQ_Flag = false;			
			if( msg_cmd_stored == CMD06 ){			
				reg_idx = i = j = 0; // index variables	
				for(; i < DATA_MPX_MAP_SIZE; i++){
					j = 0;
					for(; j < 4; j++){
						if(data_MPX_mapping_array[i].var_type_idx[j].idx == var_idx && data_MPX_mapping_array[i].var_type_idx[j].letter == ev->ev_vartype ){
							reg_idx = data_MPX_mapping_array[i].enq_idx;
							break;
						}
					}
					if(reg_idx){
						break;
					}
				}
				ev->ev_cmd = CMD06;
				ProtocolBuf[carel_REQ] = WREQ;
				ProtocolBuf[carel_regs] = reg_idx;
				// j is a offset founded above for specific variable
				ProtocolBuf[4 + 4 * j] = int_to_char_array[( ( ev->ev_regvalue >> 12 ) & 0x0F )].char_value ;  
				ProtocolBuf[5 + 4 * j] = int_to_char_array[( ( ev->ev_regvalue >> 8 ) & 0x0F )].char_value ; 
				ProtocolBuf[6 + 4 * j] = int_to_char_array[( ( ev->ev_regvalue >> 4 ) & 0x0F )].char_value ;
				ProtocolBuf[7 + 4 * j] = int_to_char_array[( ev->ev_regvalue & 0x0F )].char_value ;
			}
			if( msg_cmd_stored == W1COIL ){
				ev->ev_cmd = W1COIL;
				ProtocolBuf[carel_REQ] = WREQ;
				ProtocolBuf[carel_regs] = 0x4F;
				switch(var_idx){
					case 60:
						j = 0;
						ProtocolBuf[4 + 4 * j] = 0x30;  
						ProtocolBuf[5 + 4 * j] = ( ev->ev_regvalue == 0 ) ? 0x30 : 0x31 ; 						
						break;
					case 61:
						j = 0;
						ProtocolBuf[6 + 4 * j] = 0x30 ;
						ProtocolBuf[7 + 4 * j] = ( ev->ev_regvalue == 0 ) ? 0x30 : 0x31  ;					
						break;
					case 62:
						j = 1;
						ProtocolBuf[4 + 4 * j] = 0x30;  
						ProtocolBuf[5 + 4 * j] = ( ev->ev_regvalue == 0 ) ? 0x30 : 0x31 ; 
						break;
					case 63:
						j = 1;
						ProtocolBuf[6 + 4 * j] = 0x30 ;
						ProtocolBuf[7 + 4 * j] = ( ev->ev_regvalue == 0 ) ? 0x30 : 0x31  ;
						break;
					case 64:
						j = 2;
						ProtocolBuf[4 + 4 * j] = 0x30;  
						ProtocolBuf[5 + 4 * j] = ( ev->ev_regvalue == 0 ) ? 0x30 : 0x31 ; 
						break;
					default:
						break;
				}
			}
			ProtocolBuf[20] = ETX;
			msg_len = 23;
		} else if( msg_cmd == CMD03 ){
			// allocate memory for registers data, will free in CMD03 section ModbusSendCarel function, Modbus.c file
			ev->dataptr = (uint8_t *)malloc(sizeof(uint8_t) * ev->ev_len * 2);		
			if(ev->ev_vartype == ANA_VAR){
				for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j += 2){
					ev->dataptr[j] = ( ANA_ARRAY[i] >> 8 );
					ev->dataptr[j+1] = ( ANA_ARRAY[i] & 0xFF );
				}
			} else if(ev->ev_vartype == INT_VAR){
				for(i = var_idx, j = 0; i < (var_idx + ev->ev_len); i++, j += 2){
					ev->dataptr[j] = ( INT_ARRAY[i] >> 8 );
					ev->dataptr[j+1] = ( INT_ARRAY[i] & 0xFF );
				}					
			} else if(ev->ev_vartype == HW_VAR){
					ev->dataptr[0] = ( CAREL_DEVICE_PARAM[carel_dev_hwid] >> 8);
					ev->dataptr[1] = ( CAREL_DEVICE_PARAM[carel_dev_hwid] & 0xFF );
			} else {
				
			} 
		} else {
			
		}

		EnableInterrupts();
		if( msg_cmd == RCOIL || msg_cmd == CMD03 ){
			OS_Signal(&ModbusSendSema);
		} else if( ( msg_cmd ==  CMD06 || msg_cmd == W1COIL ) && sendWrite_Flag){
			OS_Signal(&PortCtrlTxInitSema);
		}
		else {
			CarelCRC(ProtocolBuf, msg_len);
			U1_TxMessageSize = msg_len;
			OS_Signal(&PortCtrlTxInitSema);
		}
	}
}

//***********trigCarelEasy***************
// returns none
// Inputs: none
// Outputs: none
// Prepare flags for read Carel adapters
void trigCarelEasy(void){
	while(1){
		OS_Wait(&CarelReadSema);
			OS_Signal(&ReadCarelENQSema);
			sendDREQ_Flag = true;
			sendF_Flag = false;
			sendENQ_Flag = false;
			sendACK_Flag = false;
	}
}

//***********trigCarelMPX***************
// returns none
// Inputs: none
// Outputs: none
// Prepare flags for read Carel adapters
void trigCarelMPX(void){
	while(1){
		OS_Wait(&CarelReadSema);
		sendENQ_Flag = true;
		sendF_Flag = false;
		sendACK_Flag = false;
		sendDREQ_Flag = false;	
		OS_Signal(&ReadCarelENQSema);
	}
}

//***********sendReadDREQ***************
// returns none
// Inputs: none
// Outputs: none
// Prepare device request
static void sendReadDREQ(void){
	ProtocolBuf[carel_header] = STX;
	ProtocolBuf[carel_address] = ev->ev_carel_addr;
	ProtocolBuf[carel_REQ] = ev->ev_cmd = DREQ;
	ProtocolBuf[carel_regs] = 0x31;
	ProtocolBuf[carel_etx] = ETX;
	U1_TxMessageSize = ProtocolMsgSize = 7;
	CarelCRC(&ProtocolBuf[0], ProtocolMsgSize);
}

//***********sendRead_F***************
// returns none
// Inputs: none
// Outputs: none
// Prepare device request to receive HWid and FWid
static void sendRead_F(void){
	ProtocolBuf[carel_header] = STX;
	ProtocolBuf[carel_address] = ev->ev_carel_addr;
	ProtocolBuf[carel_REQ] = ev->ev_cmd = FREQ;
	ProtocolBuf[carel_regs] = 0x31;
	ProtocolBuf[carel_etx] = ETX;
	U1_TxMessageSize = ProtocolMsgSize = 7;
	CarelCRC(&ProtocolBuf[0], ProtocolMsgSize);	
}
//***********sendENQ***************
// returns none
// Inputs: none
// Outputs: none
// Prepare equire command { ADDR, 0x05 } for sequentially receive data from all variables
static void sendENQ(void){
	ProtocolBuf[0] = ev->ev_cmd = ENQ;
	ProtocolBuf[1] = ev->ev_carel_addr;
	U1_TxMessageSize = ProtocolMsgSize = 2;
}
//***********sendACK***************
// returns none
// Inputs: none
// Outputs: none
// Prepare ACK command {0x06}, for acknowlege that data received
static void sendACK(){
	ProtocolBuf[0] = ACK;
	U1_TxMessageSize = ProtocolMsgSize = 1;
}

//***********readCarelEasy***************
// returns none
// Inputs: none
// Outputs: none
// Prepare command for specific flag and send it to device 
void readCarelEasy(void){
	while(1){
		OS_Wait(&ReadCarelENQSema);
		if(sendDREQ_Flag){
			sendReadDREQ();
		}
		if(sendF_Flag){
			sendRead_F();
		}
		if(sendENQ_Flag){
			OS_Sleep(50);
			sendENQ();
		}
		if(sendACK_Flag){
			OS_Sleep(50);
			sendACK();
		}
		OS_Signal(&PortCtrlTxInitSema);
	}
}

//***********sendMPXRead***************
// returns none
// Inputs: none
// Outputs: none
// Prepare device request
static void sendMPXRead(void){	
	ProtocolBuf[carel_header] = STX;
	ProtocolBuf[carel_address] = ev->ev_carel_addr;
	ProtocolBuf[carel_REQ] = ev->ev_cmd = RREQ;	
	ProtocolBuf[carel_regs] = idxFront;
	ProtocolBuf[4] = idxBack;
	ProtocolBuf[5] = ETX;
	U1_TxMessageSize = ProtocolMsgSize = 8;
	CarelCRC(&ProtocolBuf[0], ProtocolMsgSize);
}

//***********readCarelMPX***************
// returns none
// Inputs: none
// Outputs: none
// Prepare command for specific flag and send it to device 
void readCarelMPX(void){
	idxFront = 0x00;
	idxBack = 0x44;
	while(1){
		OS_Wait(&ReadCarelENQSema);
		if(idxFront == 0x64 && idxBack == 0x64){
			idxFront = 0x00;
			idxBack = 0x44;
			dataReadyFlag = true; // set this flag to start responce to MU device
		} else {
			if(sendENQ_Flag){
				OS_Sleep(50);
				idxFront = idxBack;
				idxBack++;
				if(idxFront == 0x47){
					idxBack = 0x4A;
				} else if(idxFront == 0x4A){
					idxBack = 0x4F;
				} else if(idxFront == 0x64){
					idxBack = 0x64;
				}
				else {
					/* Nothing to do! */
				}
				sendMPXRead();
			} else if(sendACK_Flag){
				OS_Sleep(50);
				sendACK();
			} else {
				/* Nothing to do! */
			}	
			OS_Signal(&PortCtrlTxInitSema);
		}
	}
}

//***********char_to_int10_15***************
// returns none
// Inputs: char corresponding ( 9 < HEX number < 16 )
// Outputs: none
// Return integer value for HEX number
static uint8_t char_to_int10_15(uint8_t char_int){
	switch(char_int){
    case 'A': return 10;
    case 'B': return 11;
    case 'C': return 12;
    case 'D': return 13;
    case 'E': return 14;
    case 'F': return 15;
    default: return INT8_MAX;
	}
}

static uint8_t char_to_int(uint8_t char_int){
	switch(char_int){
    case '0': return 0 ;
    case '1': return 1 ;
    case '2': return 2 ;
    case '3': return 3 ;
    case '4': return 4 ;
    case '5': return 5 ;
    case '6': return 6 ;
    case '7': return 7 ;
    case '8': return 8 ;
    case '9': return 9 ;
    case 'A': return 10;
    case 'B': return 11;
    case 'C': return 12;
    case 'D': return 13;
    case 'E': return 14;
    case 'F': return 15;
    default : return INT8_MAX;
  }
}

//***********readCarelCtrlACK***************
// returns none
// Inputs: none
// Outputs: none
// Handle data received from CarelEasy device
void readCarelCtrlACK(void){
	uint16_t varType;
	uint32_t varIndex;
//	bool correctIndexFlag = false;
	bool correctCRC = false;
	
	while(1){
		OS_Wait(&ReadCarelACKSema);
 		if(ProtocolBuf[0]){ // If buffer not contain zero, Carel replied zero when send all data
			DisableInterrupts();
			if(ProtocolBuf[0] == ACK){ // If receive ACK don't check CRC
				correctCRC = false;
			} else {
				correctCRC = CarelCRC_check(&ProtocolBuf[0], ProtocolMsgSize);
			}
			if( correctCRC ){
				varType = ProtocolBuf[carel_vartype]; // Get variable type
				varIndex = ProtocolBuf[carel_varidx] - 0x30; // Get variable index
				switch(varType){
					case RANA_VAR: // In case of analog variable calculate value and place to analog variables array
						if(varIndex > 0 && varIndex < ANA_ARRAY_SIZE){
							ANA_ARRAY[varIndex] = (
																			(( ((ProtocolBuf[5] >> 4) == 3) ? (ProtocolBuf[5] - 0x30) : char_to_int10_15(ProtocolBuf[5]) ) << 12) |  
																		  (( ((ProtocolBuf[6] >> 4) == 3) ? (ProtocolBuf[6] - 0x30) : char_to_int10_15(ProtocolBuf[6]) ) << 8)  | 
																		  (( ((ProtocolBuf[7] >> 4) == 3) ? (ProtocolBuf[7] - 0x30) : char_to_int10_15(ProtocolBuf[7]) ) << 4)  | 
																		  ((  (ProtocolBuf[8] >> 4) == 3) ? (ProtocolBuf[8] - 0x30) : char_to_int10_15(ProtocolBuf[8]))
																		);
//							correctIndexFlag = true;
						} else {
//							correctIndexFlag = false;
						}
						break;
					case RINT_VAR: // In case of integer variable calculate value and place to integer variables array
						if(varIndex > 0 && varIndex < INT_ARRAY_SIZE){
							INT_ARRAY[varIndex] = (
																			(( ((ProtocolBuf[5] >> 4) == 3) ? (ProtocolBuf[5] - 0x30) : char_to_int10_15(ProtocolBuf[5]) ) << 12) | 
																		  (( ((ProtocolBuf[6] >> 4) == 3) ? (ProtocolBuf[6] - 0x30) : char_to_int10_15(ProtocolBuf[6]) ) << 8)  | 
																		  (( ((ProtocolBuf[7] >> 4) == 3) ? (ProtocolBuf[7] - 0x30) : char_to_int10_15(ProtocolBuf[7]) ) << 4)  | 
																		  ((  (ProtocolBuf[8] >> 4) == 3) ? (ProtocolBuf[8] - 0x30) : char_to_int10_15(ProtocolBuf[8]))
																		);
//							correctIndexFlag = true;
						} else {
//							correctIndexFlag = false;
						}
						break;
					case RDIG_VAR: // In case of digital variable calculate value and place to digital variables array
						if(varIndex > 0 && varIndex < DIG_ARRAY_SIZE){
							DIG_ARRAY[varIndex] = ((ProtocolBuf[5] - 0x30) << 4 ) | (ProtocolBuf[6] - 0x30);
//							correctIndexFlag = true;
						} else {
//							correctIndexFlag = false;
						}
						break;
					case RX_VAR: // In case of sendRead_F() place device information, HWid and FWid to corresponding array
						CAREL_DEVICE_PARAM[carel_dev_hwid] = (( ((ProtocolBuf[5] >> 4) == 3) ? (ProtocolBuf[5] - 0x30) : char_to_int10_15(ProtocolBuf[5]) ) << 4) | ( ((ProtocolBuf[6] >> 4) == 3) ? (ProtocolBuf[6] - 0x30) : char_to_int10_15(ProtocolBuf[6]) ); // Calculate Carel device HWid
						CAREL_DEVICE_PARAM[carel_dev_dat1] = (( ((ProtocolBuf[9] >> 4) == 3) ? (ProtocolBuf[9] - 0x30) : char_to_int10_15(ProtocolBuf[9]) ) << 4) | ( ((ProtocolBuf[10] >> 4) == 3) ? (ProtocolBuf[10] - 0x30) : char_to_int10_15(ProtocolBuf[10]) ); // ???
						CAREL_DEVICE_PARAM[carel_dev_dat2] = (( ((ProtocolBuf[17] >> 4) == 3) ? (ProtocolBuf[17] - 0x30) : char_to_int10_15(ProtocolBuf[17]) ) << 4) | ( ((ProtocolBuf[18] >> 4) == 3) ? (ProtocolBuf[18] - 0x30) : char_to_int10_15(ProtocolBuf[18]) ); // Calculate Carel device HWid
						CAREL_DEVICE_PARAM[carel_dev_fwid] = (( ((ProtocolBuf[20] >> 4) == 3) ? (ProtocolBuf[20] - 0x30) : char_to_int10_15(ProtocolBuf[20]) ) << 4) | ( ((ProtocolBuf[22] >> 4) == 3) ? (ProtocolBuf[22] - 0x30) : char_to_int10_15(ProtocolBuf[22]) ); // Calculate Carel device FWid
						break;
					case RR_VAR:
						
						break;
					default:
						break;
				}
			}
			if( sendDREQ_Flag ){
				sendF_Flag = true;
				sendDREQ_Flag = false;
				sendENQ_Flag = false;
				sendACK_Flag = false;
				OS_Signal(&ReadCarelENQSema);
			} else if( sendF_Flag ){
				sendENQ_Flag = true;
				sendF_Flag = false;
				sendDREQ_Flag = false;
				sendACK_Flag = false;
				OS_Signal(&ReadCarelENQSema);
			} else if( sendENQ_Flag ){
				sendACK_Flag = true;
				sendENQ_Flag = false;
				sendF_Flag = false;
				sendDREQ_Flag = false;
				OS_Signal(&ReadCarelENQSema);
			}	else if( sendACK_Flag ){
				sendENQ_Flag = true;
				sendACK_Flag = false;
				sendF_Flag = false;
				sendDREQ_Flag = false;
				OS_Signal(&ReadCarelENQSema);
			} else {
					/* Nothing to do! */
			}
		} else {
			if( (ev->ev_cmd == CMD06 || ev->ev_cmd == W1COIL) && !writeACK ){
				OS_Signal(&ModbusSendSema);
			}
			dataReadyFlag = true; // Receive from controller zero after scan cycle, set this flag to start process request from MU device
//			array_idx = 0;
		}
		EnableInterrupts();
	}
}


uint16_t digital_variable_mask_1 = 1;
//***********handleResponceCarelMPX***************
// returns none
// Inputs: none
// Outputs: none
// Handle data received from CarelEasy device
void handleResponceCarelMPX(void){
	uint16_t varType, dataValue[4], data_idx;
	uint32_t varIndex;
//	bool correctIndexFlag = false;
	bool correctCRC = false;
	
	while(1){
		OS_Wait(&ReadCarelACKSema);
		DisableInterrupts();
		if(ProtocolBuf[0] == ACK){ // If receive ACK don't check CRC
			correctCRC = false;
		} else {
			correctCRC = CarelCRC_check(&ProtocolBuf[0], ProtocolMsgSize);
		}
		if( correctCRC ){
			varIndex = ProtocolBuf[carel_varidx]; // Get variable index
			data_idx = carel_start_data;
			dataValue[0] = ( char_to_int( ProtocolBuf[data_idx] ) << 12 ) | ( char_to_int(ProtocolBuf[data_idx + 1]) << 8 ) | ( char_to_int(ProtocolBuf[data_idx + 2]) << 4 ) | char_to_int(ProtocolBuf[data_idx + 3]);
			data_idx += 4;
			dataValue[1] = ( char_to_int( ProtocolBuf[data_idx] ) << 12 ) | ( char_to_int(ProtocolBuf[data_idx + 1]) << 8 ) | ( char_to_int(ProtocolBuf[data_idx + 2]) << 4 ) | char_to_int(ProtocolBuf[data_idx + 3]);
			data_idx += 4;
			dataValue[2] = ( char_to_int( ProtocolBuf[data_idx] ) << 12 ) | ( char_to_int(ProtocolBuf[data_idx + 1]) << 8 ) | ( char_to_int(ProtocolBuf[data_idx + 2]) << 4 ) | char_to_int(ProtocolBuf[data_idx + 3]);
			data_idx += 4;
			dataValue[3] = ( char_to_int( ProtocolBuf[data_idx] ) << 12 ) | ( char_to_int(ProtocolBuf[data_idx + 1]) << 8 ) | ( char_to_int(ProtocolBuf[data_idx + 2]) << 4 ) | char_to_int(ProtocolBuf[data_idx + 3]);
			for(uint16_t i = 0; i < DATA_MPX_MAP_SIZE; i++){
				if(data_MPX_mapping_array[i].enq_idx == varIndex){
					for(uint8_t j = 0; j < 4; j++){
						varType = data_MPX_mapping_array[i].var_type_idx[j].letter; 
						switch(varType){
							case 'A':
								ANA_ARRAY[data_MPX_mapping_array[i].var_type_idx[j].idx] = dataValue[j];
								break;
							case 'I':
								INT_ARRAY[data_MPX_mapping_array[i].var_type_idx[j].idx] = dataValue[j];
								break;
							case 'D':
								switch(varIndex){
									case 0x45:
										DIG_ARRAY[1] = 	( ( dataValue[0] & (digital_variable_mask_1 << 8)  ) ? 1 : 0 );
										DIG_ARRAY[2] = 	( ( dataValue[0] & (digital_variable_mask_1 << 9)  ) ? 1 : 0 );
										DIG_ARRAY[3] = 	( ( dataValue[0] & (digital_variable_mask_1 << 10) ) ? 1 : 0 );
										DIG_ARRAY[4] = 	( ( dataValue[0] & (digital_variable_mask_1 << 11) ) ? 1 : 0 );
										DIG_ARRAY[5] = 	( ( dataValue[2] & (digital_variable_mask_1 << 8)  ) ? 1 : 0 );
										DIG_ARRAY[6] = 	( ( dataValue[2] & (digital_variable_mask_1 << 9)  ) ? 1 : 0 );
										DIG_ARRAY[7] = 	( ( dataValue[2] & (digital_variable_mask_1 << 10) ) ? 1 : 0 );
										DIG_ARRAY[8] = 	( ( dataValue[2] & (digital_variable_mask_1 << 11) ) ? 1 : 0 );
										DIG_ARRAY[9] = 	( ( dataValue[2] & (digital_variable_mask_1 << 12) ) ? 1 : 0 );
										DIG_ARRAY[10] = ( ( dataValue[2] & (digital_variable_mask_1 << 13) ) ? 1 : 0 );
										DIG_ARRAY[11] = ( ( dataValue[2] & (digital_variable_mask_1 << 14) ) ? 1 : 0 );
										DIG_ARRAY[12] = ( ( dataValue[2] & (digital_variable_mask_1 << 15) ) ? 1 : 0 );
										break;
									case 0x46:
										DIG_ARRAY[21] = ( ( dataValue[0] & (digital_variable_mask_1 << 8)  ) ? 1 : 0 );
										DIG_ARRAY[22] = ( ( dataValue[0] & (digital_variable_mask_1 << 9)  ) ? 1 : 0 );
										DIG_ARRAY[23] = ( ( dataValue[0] & (digital_variable_mask_1 << 10) ) ? 1 : 0 );
										DIG_ARRAY[24] = ( ( dataValue[0] & (digital_variable_mask_1 << 11) ) ? 1 : 0 );
										DIG_ARRAY[25] = ( ( dataValue[0] & (digital_variable_mask_1 << 12) ) ? 1 : 0 );
										DIG_ARRAY[26] = ( ( dataValue[0] & (digital_variable_mask_1 << 13) ) ? 1 : 0 );
										DIG_ARRAY[27] = ( ( dataValue[0] & (digital_variable_mask_1 << 14) ) ? 1 : 0 );
										DIG_ARRAY[28] = ( ( dataValue[0] & (digital_variable_mask_1 << 15) ) ? 1 : 0 );
									
										DIG_ARRAY[29] = ( ( dataValue[1] & (digital_variable_mask_1 << 8)  ) ? 1 : 0 );
										DIG_ARRAY[30] = ( ( dataValue[1] & (digital_variable_mask_1 << 9)  ) ? 1 : 0 );
										DIG_ARRAY[31] = ( ( dataValue[1] & (digital_variable_mask_1 << 10) ) ? 1 : 0 );
										DIG_ARRAY[32] = ( ( dataValue[1] & (digital_variable_mask_1 << 11) ) ? 1 : 0 );
										DIG_ARRAY[33] = ( ( dataValue[1] & (digital_variable_mask_1 << 12) ) ? 1 : 0 );
										DIG_ARRAY[34] = ( ( dataValue[1] & (digital_variable_mask_1 << 13) ) ? 1 : 0 );
										DIG_ARRAY[35] = ( ( dataValue[1] & (digital_variable_mask_1 << 14) ) ? 1 : 0 );
										DIG_ARRAY[36] = ( ( dataValue[1] & (digital_variable_mask_1 << 15) ) ? 1 : 0 );	

										DIG_ARRAY[37] = ( ( dataValue[2] & (digital_variable_mask_1 << 8)  ) ? 1 : 0 );
										DIG_ARRAY[38] = ( ( dataValue[2] & (digital_variable_mask_1 << 9)  ) ? 1 : 0 );
										DIG_ARRAY[39] = ( ( dataValue[2] & (digital_variable_mask_1 << 10) ) ? 1 : 0 );
										DIG_ARRAY[40] = ( ( dataValue[2] & (digital_variable_mask_1 << 11) ) ? 1 : 0 );
										DIG_ARRAY[41] = ( ( dataValue[2] & (digital_variable_mask_1 << 12) ) ? 1 : 0 );
										DIG_ARRAY[42] = ( ( dataValue[2] & (digital_variable_mask_1 << 13) ) ? 1 : 0 );
										DIG_ARRAY[43] = ( ( dataValue[2] & (digital_variable_mask_1 << 14) ) ? 1 : 0 );
										
										DIG_ARRAY[45] = ( ( dataValue[3] & (digital_variable_mask_1 << 8)  ) ? 1 : 0 );
										DIG_ARRAY[46] = ( ( dataValue[3] & (digital_variable_mask_1 << 9)  ) ? 1 : 0 );
										DIG_ARRAY[47] = ( ( dataValue[3] & (digital_variable_mask_1 << 10) ) ? 1 : 0 );
										DIG_ARRAY[48] = ( ( dataValue[3] & (digital_variable_mask_1 << 11) ) ? 1 : 0 );
										DIG_ARRAY[49] = ( ( dataValue[3] & (digital_variable_mask_1 << 12) ) ? 1 : 0 );
										break;
									case 0x4F:
										DIG_ARRAY[60] = ( ( dataValue[0] & (digital_variable_mask_1 << 8) ) ? 1 : 0 );
										DIG_ARRAY[61] = ( ( dataValue[0] & digital_variable_mask_1 ) ? 1 : 0 );
										DIG_ARRAY[62] = ( ( dataValue[1] & (digital_variable_mask_1 << 8) ) ? 1 : 0 );
										DIG_ARRAY[63] = ( ( dataValue[1] & digital_variable_mask_1 ) ? 1 : 0 );
										DIG_ARRAY[64] = ( ( dataValue[2] & (digital_variable_mask_1 << 8) ) ? 1 : 0 );										
										break;
									default:
										break;
								}
								break;
							default:
								break;
						}
					}
				}
			}

		}
		if( sendDREQ_Flag ){
			sendF_Flag = true;
			sendDREQ_Flag = false;
			sendENQ_Flag = false;
			sendACK_Flag = false;
			OS_Signal(&ReadCarelENQSema);
		} else if( sendF_Flag ){
			sendENQ_Flag = true;
			sendF_Flag = false;
			sendDREQ_Flag = false;
			sendACK_Flag = false;
			OS_Signal(&ReadCarelENQSema);
		} else if( sendENQ_Flag ){
			sendACK_Flag = true;
			sendENQ_Flag = false;
			sendF_Flag = false;
			sendDREQ_Flag = false;
			OS_Signal(&ReadCarelENQSema);
		}	else if( sendACK_Flag ){
				sendENQ_Flag = true;
				sendACK_Flag = false;
				sendF_Flag = false;
				sendDREQ_Flag = false;
			OS_Signal(&ReadCarelENQSema);
		} else {
				// Nothing to do!
		}
		EnableInterrupts();
	}
}

//***********CarelScanHndl***************
// returns none
// Inputs: none
// Outputs: none
// Handle device recponce on address request
void CarelScanHndl(void){
	bool correctCRC = false;
	
	while(1){
		OS_Wait(&CarelScanHndlSema);
		correctCRC = CarelCRC_check(&ProtocolBuf[0], ProtocolMsgSize);
		if(correctCRC){
			if(ProtocolBuf[carel_vartype] == RV_VAR){
				if( !DeviceFound ){
					DeviceFound = true;
					CAREL_DEVICE_PARAM[carel_dev_hwid] = ProtocolBuf[carel_regs] - 0x30;
					OS_Signal(&CtrlScanSema);
				}				
			}
		}
	}
}

//***********CarelScan***************
// returns none
// Inputs: none
// Outputs: none
// Initialize connected controller address scan by setup Carel protocol scan message and 
// signal CarelEasyPortTxInit() to transmit it to connected controller, if responce will received, controller address was found,
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
			deviceScanFlag = false;
#ifdef APDEBUG			
			CntDeviceFound++;
#endif			
			OS_Signal(&U3_RxInitSema);
			CtrlScanSema = 0;
		} else {
				// Toggle Green Led indicate that scan operation running
				ToggleLedGreen();
				deviceScanFlag = true;
				// Setup address to get responce from controller
				if(ProtocolBuf[carel_address]++ > 254){
					ProtocolBuf[carel_address] = 0x31;
				}
				Event.ev_carel_addr = ProtocolBuf[carel_address];
				Event.ev_addr = ProtocolBuf[carel_address] - 0x30;
				Event.ev_cmd = ProtocolBuf[carel_REQ];
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
