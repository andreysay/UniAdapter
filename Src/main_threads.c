/**
  ******************************************************************************
  * File Name          : main_threads.c
  * Description        : This file provides code for Send/Receive functions
  *                      for Dixel or RS485 support controllers.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_usart.h"
#include "ControllerType.h"
#include "main.h"
#include "StartUp.h"
#include "os.h"
#include "ControllerPort.h"
#include "LED.h"

#ifdef APDEBUG
// Counters for debugging
uint32_t Count0, Count1, Count2, Count3, Count4, Count5, Count6, Count7, Count8;
#endif

// link to ControllerType.c
extern bool DeviceFound;

// Semaphore for periodic thread which runs every 100ms
int32_t Time100msSemaphore;
// Semaphore for periodic thread which runs every 1 second
int32_t Time1secSemaphore;

// USART3 MU_Port semaphores
// Semaphore for reception
int32_t U3_RxSemaphore;
// Semaphore for transmition
int32_t U3_TxSemaphore;
// Semaphore for reception initialization
int32_t U3_RxInitSema;

/**
  * @brief RX/TX buffers for storing received data through USART3
  */
uint8_t U3_RXBuffer[RX_BUFFER_SIZE];
uint8_t U3_TXBuffer[RX_BUFFER_SIZE];
//Pointer to U3_RXBuffer used in USART3.c USART3_Reception_Callback function to handle data reception
uint8_t *U3_pBufferReception = NULL;
//Pointer to U3_TXBuffer used in USART3.c USART3_TXEmpty_Callback function to handle data transmission
uint8_t *U3_pBufferTransmit = NULL;
// RX buffer ready indication
__IO uint32_t     U3_BufferReadyIndication;
// index to move on buffer U3_TXBuffer[]
__IO uint32_t U3_idxTx = 0;
// index to move on buffer U3_RXBuffer[]
__IO uint32_t U3_idxRx = 0;
// Tx/Rx message size variable
uint8_t U3_TxMessageSize = 0;
uint8_t U3_RxMessageSize = 0;

// USART1 controller port semaphores
// Semaphore for Controller port TX mode initialization
int32_t PortCtrlTxInitSema;
// Semaphore for Controller port RX mode initialization
int32_t PortCtrlRxInitSema;
// Semaphore for transmition
int32_t U1_TxSemaphore;
// Semaphore for reception
int32_t U1_RxSemaphore;

/**
  * @brief RX/TX buffers and semaphores for storing received data through USART1
  */
uint8_t U1_RXBufferA[RX_BUFFER_SIZE];
uint8_t U1_TXBuffer[RX_BUFFER_SIZE];
// RX buffer ready indication
__IO uint32_t     U1_BufferReadyIndication;
// Tx/Rx message size variable
__IO uint8_t U1_RxMessageSize = 0;
uint8_t U1_TxMessageSize = 0;
// index to move on buffer U1_TXBuffer[]
__IO uint32_t U1_idxTx = 0;
// index to move on buffer U1_RXBufferA[]
__IO uint32_t U1_idxRx = 0;

//Pointer to U1_RXBufferA used in USART1 USART1_Reception_Callback function to handle data reception
uint8_t *U1_pBufferReception = NULL;
//Pointer to U1_TXBuffer used in USART1 USART1_TXEmpty_Callback function to handle data transmission
uint8_t *U1_pBufferTransmit = NULL;

//***********EventThread100ms***************
// returns none
// Inputs: none
// Outputs: none
// Event thread which active every 100 milli second 
void EventThread100ms(void){
#ifdef APDEBUG	
	Count3 = 0;
#endif	
	while(1){
		OS_Wait(&Time100msSemaphore);           // 1000 Hz real time task
#ifdef APDEBUG		
		Count3++;
#endif		
	}
}
//***********EventThread1sec***************
// returns none
// Inputs: none
// Outputs: none
// Event thread which active every 1 second 
void EventThread1sec(void){ 
#ifdef APDEBUG	
  Count4 = 0;
#endif	
	while(1){
		OS_Wait(&Time1secSemaphore);
		if(DeviceFound){
			ToggleLedGreen();
		}
#ifdef APDEBUG		
		Count4++;
#endif		
	};
}

//***********MU_PortReceptionInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize MU port for data reception, 
// Must run first to receive command from MU device
// will signal MU_PortHandleContinuousReception() to continue
void MU_PortReceptionInit(void)
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
		U3_pBufferTransmit = &U3_TXBuffer[0];
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

//***********MU_PortHandleContinuousReception***************
// returns none
// Inputs: none
// Outputs: none
// Wait for data reception from MU trough USART3, will signal from USART3 ISR trough USART3_IDLE_Callback()
void MU_PortHandleContinuousReception(void)
{
#ifdef APDEBUG	
	Count1 = 0;
#endif	
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
			U1_TxMessageSize = U3_RxMessageSize;
			for(i = 0; i < U3_RxMessageSize; i++){
				U1_TXBuffer[i] = U3_RXBuffer[i];
				U3_RXBuffer[i] = 0;
			}
			/* Turn on green led, indication that data from monitor unit received */	
			HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_SET);
			// Enable interrupts
			EnableInterrupts();
			OS_Signal(&PortCtrlTxInitSema);
		}
#ifdef APDEBUG		
		Count1++;
#endif		
	}
}

//***********MU_PortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// transmit data to MU device through USART3
void MU_PortSendMsg(void){
#ifdef APDEBUG	
	Count2 = 0;
#endif	
	GPIO_PinState pinDirState;
	while(1){
		OS_Wait(&U3_TxSemaphore);
		pinDirState = HAL_GPIO_ReadPin(GPIO_Dir, PIN_Dir);
		if(pinDirState != GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_SET);
		}
		/* Turn LEDs On at start of transfer : Tx started */
		HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_SET);
		/* Fill DR with a new char */
		LL_USART_TransmitData8(USART3, U3_TXBuffer[U3_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART3); 
#ifdef APDEBUG		
		Count2++;
#endif		
	}
}

//***********CtrlPortRxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize index variable, buffer pointer, USART1 interrupts
// signal CtrlPortHandleContinuousReception() for reception
void CtrlPortRxInit(void)
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

//***********CtrlPortHandleContinuousReception***************
// returns none
// Inputs: none
// Outputs: none
// Waiting for data reception from USART1, will signal by USART1 ISR from USART1_IDLE_Callback()
void CtrlPortHandleContinuousReception(void)
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
				U3_TxMessageSize = U1_RxMessageSize;
				for(uint8_t i = 0; i < U1_RxMessageSize; i++){
					U3_TXBuffer[i] = U1_RXBufferA[i];
					U1_RXBufferA[i] = 0;
				}
				LEDs_off();
				EnableInterrupts();
				OS_Signal(&U3_TxSemaphore);
			}
#ifdef APDEBUG			
		Count8++;
#endif			
	}
}

//***********CtrlPortTxInit***************
// returns none
// Inputs: none
// Outputs: none
// Initialize controller port for transmission, index variable, pointer to buffer and transmission buffer,
void CtrlPortTxInit(void)
{
#ifdef APDEBUG	
	Count5 = 0;
#endif	
	while(1){
		OS_Wait(&PortCtrlTxInitSema);
		// buffer pointer initialization
		U1_pBufferTransmit = &U1_TXBuffer[0];
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


//***********CtrlPortSendMsg***************
// returns none
// Inputs: none
// Outputs: none
// Send Modbus message to connected controller through USART1,
void CtrlPortSendMsg(void){
#ifdef APDEBUG	
	Count6 = 0;
#endif	
	while(1){
		OS_Wait(&U1_TxSemaphore);
		/* Turn ORANGE On at start of transfer : Tx sequence started successfully */
		HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_SET);
		LL_USART_TransmitData8(USART1, U1_TXBuffer[U1_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART1);
		OS_Signal(&PortCtrlRxInitSema); // Signal semaphore to initialize data reception		
#ifdef APDEBUG		
		Count6++;
#endif		
	}
}

// Free time counter
uint32_t Freetime;
void IdleTask(void){ // dummy task
  Freetime = 0;      // this task cannot block, sleep or kill
  while(1){
    Freetime++; 
  }
}
/************************ (C) COPYRIGHT *****END OF FILE****/
