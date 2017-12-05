/**
  ******************************************************************************
  * File Name          : Dixel.c
  * Description        : This file provides code for Send/Receive functions
  *                      for Dixel controllers.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_usart.h"
#include "ControllerType.h"
#include "main.h"
#include "StartUp.h"
#include "os.h"
#include "ControllerPort.h"

// USART3 MU_Port semaphores
// Semaphore for transmit
int32_t U3_RxSemaphore;
// Semaphore for reception
int32_t U3_TxSemaphore;
// Semaphore for reception initialisation
int32_t U3_RxInitSema;

/**
  * @brief RX buffers for storing received data through USART3
  */
uint8_t U3_RXBuffer[RX_BUFFER_SIZE];
uint8_t U3_TXBuffer[RX_BUFFER_SIZE];

__IO uint32_t     U3_BufferReadyIndication;
// index to move on buffer U3_TXBuffer[]
__IO uint32_t U3_idxTx = 0;
// index to move on buffer U3_RXBuffer[]
__IO uint32_t U3_idxRx = 0;
uint8_t U3_TxMessageSize = 0;
uint8_t U3_RxMessageSize = 0;

/**
  * @brief RX/TX buffers and semaphores for storing received data through USART1
  */
// Semaphore for Controller port initialisation
int32_t PortCtrlTxInitSema;
int32_t PortCtrlRxInitSema;
int32_t U1_TxSemaphore;
int32_t U1_RxSemaphore;
int32_t CtrlRxTimeIsNotExpired = 1;
__IO uint32_t U1_idxTx = 0;
__IO uint32_t U1_idxRx = 0;

uint8_t U1_RXBufferA[RX_MESSAGE_SIZE];
//uint8_t U1_RXBufferB[RX_MESSAGE_SIZE];
uint8_t U1_TXBuffer[RX_BUFFER_SIZE];
__IO uint32_t     U1_BufferReadyIndication;
uint8_t U1_RxMessageSize = 0;
uint8_t U1_TxMessageSize = 0;

//uint8_t *pBufferMessage;
uint8_t *pBufferReception;

// Counters for debugging
uint32_t Count0;
void MU_PortReceptionInit(void)
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

uint32_t Count1;
void MU_PortHandleContinuousReception(void)
{
	Count1 = 0;
	uint8_t i;
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
			/* Turn on orange led, indication that data from monitor unit received */	
			HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_SET);
			// Enable interrupts
			EnableInterrupts();
			OS_Signal(&PortCtrlTxInitSema);
			OS_Wait(&U3_RxSemaphore);
		}
//		OS_Signal(&U3_RxSemaphore);
		Count1++;
	}
}

uint32_t Count2;
void MU_PortSendMsg(void){
	Count2 = 0;
	GPIO_PinState pinDirState;
	while(1){
		OS_Wait(&U3_TxSemaphore);
		pinDirState = HAL_GPIO_ReadPin(GPIO_Dir, PIN_Dir);
		if(pinDirState != GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_SET);
		}
		/* Fill DR with a new char */
		LL_USART_TransmitData8(USART3, U3_TXBuffer[U3_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART3); 
//		OS_Signal(&U3_RxSemaphore);
		Count2++;
	}
}

uint32_t Count7;
void CtrlPortRxInit(void)
{
	Count7 = 0;
	while(1){
		OS_Wait(&PortCtrlRxInitSema);
		/* Initializes Buffer indication : */
		U1_BufferReadyIndication = 0;
		/* Initializes time expiration semaphore */
		CtrlRxTimeIsNotExpired = 5;
		/* Initialize index to move on buffer */
		U1_idxRx = 0;
		pBufferReception 		= U1_RXBufferA;
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

uint32_t Count8;
void CtrlPortHandleContinuousReception(void)
{
	Count8 = 0;
	uint8_t i;
	while(1){
		if(CtrlRxTimeIsNotExpired){ // If time not expired
			OS_Wait(&U1_RxSemaphore); // Will signal from USART1_IDLE_Callback function in USART1.c driven by interrupt
			/* Checks if Buffer full indication has been set */			
			if (U1_BufferReadyIndication != 0)
			{
				DisableInterrupts();
				/* Reset indication */
				U1_BufferReadyIndication = 0;
				U3_TxMessageSize = U1_RxMessageSize;
				for(i = 0; i < U1_RxMessageSize; i++){
					U3_TXBuffer[i] = U1_RXBufferA[i];
					U1_RXBufferA[i] = 0;
				}
				/* Turn on green led, indication that data from controller received */
				HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_SET);
				EnableInterrupts();
				OS_Signal(&U3_TxSemaphore);
				OS_Wait(&U1_RxSemaphore);
			}			
		} else {
			OS_Signal(&U3_RxInitSema); // Signal USART3 receive new data
			OS_Wait(&U1_RxSemaphore);
		}
		Count8++;
	}
}

uint32_t Count5;
void CtrlPortTxInit(void)
{
	Count5 = 0;
	while(1){
		OS_Wait(&PortCtrlTxInitSema);
		if(!LL_USART_IsEnabledIT_ERROR(USART1)){
				/* Enable Error interrupt */
			LL_USART_EnableIT_ERROR(USART1);
		}
#ifdef RS485		
		CtrlPortRegistersTxInit();
#endif
		OS_Signal(&U1_TxSemaphore);
		Count5++;
	}
}


uint32_t Count6;
void CtrlPortSendMsg(void){
	Count6 = 0;
	while(1){
		OS_Wait(&U1_TxSemaphore);
#ifdef RS485
		LL_USART_DisableDirectionRx(USART1);
#endif
		LL_USART_TransmitData8(USART1, U1_TXBuffer[U1_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART1); 
		OS_Signal(&PortCtrlRxInitSema);
		Count6++;
	}
}


uint32_t Freetime;
void IdleTask(void){ // dummy task
  Freetime = 0;      // this task cannot block, sleep or kill
  while(1){
    Freetime++; 
  }
}
/************************ (C) COPYRIGHT *****END OF FILE****/
