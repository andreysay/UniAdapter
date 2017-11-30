/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "os.h"
#include "StartUP.h"
#include "BSP.h"
#include "ErrorHandler.h"
#include "USART3.h"
#include "FIFO.h"
#include "USART1.h"
#include "ControllerPort.h"
#include "Televis.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
char string[20];  // global to assist in debugging
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* it can be changed to while(1) if needed */
#define stop_cpu   __breakpoint(0)
#define THREADFREQ 1000   // frequency in Hz

// Counters for debugging
uint32_t Count0, Count1, Count2, Count3, Count4, Count5, Count6, Count7, Count8;
// Semaphore for controller scan
int32_t DeviceScan;

// USART3 MU_Port semaphores
// Semaphore for transmit
int32_t U3_RxSemaphore;
// Semaphore for reception
int32_t U3_TxSemaphore;
// Semaphore for reception initialisation
int32_t U3_RxInitSema;

// RxLed - green light
int32_t Time100msSemaphore;
// TxLed - orange light
int32_t Time1secSemaphore;

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

/**
  * @brief RX/TX buffers and semaphores for storing received data through USART1
  */
// Semaphore for Controller port initialisation
extern CtrlPortReg CtrlPortRegisters;
int32_t PortCtrlTxInitSema;
int32_t PortCtrlRxInitSema;
int32_t U1_TxSemaphore;
int32_t U1_RxSemaphore;
int32_t CtrlRxTimeIsNotExpired = 1;
__IO uint32_t U1_idxTx = 0;
__IO uint32_t U1_idxRx = 0;

uint8_t U1_RXBufferA[RX_MESSAGE_SIZE];
uint8_t U1_RXBufferB[RX_MESSAGE_SIZE];
uint8_t U1_TXBuffer[RX_BUFFER_SIZE];
__IO uint32_t     U1_BufferReadyIndication;
uint8_t U1_RxMessageSize = 0;

uint8_t *pBufferMessage;
uint8_t *pBufferReception;

TEvent Event;
bool DeviceFound = false;
uint8_t ReplyCntr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void EventThread100ms(void){
	Count3 = 0;
	while(1){
		OS_Wait(&Time100msSemaphore);           // 1000 Hz real time task
		HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOB, LED_GRN_PIN);
		while(CtrlRxTimeIsNotExpired){ CtrlRxTimeIsNotExpired--; };
		Count3++;
	};
}
void EventThread1sec(void){ 
  Count4 = 0;
	while(1){
		OS_Wait(&Time1secSemaphore);
		Count4++;
	};
}
void MU_PortReceptionInit(void)
{
	Count0 = 0;
	while(1){
		OS_Wait(&U3_RxInitSema);
		HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_RESET);
		// Initialize number of received bytes
		U3_idxRx = 0;
		// Initialize buffer ready indication
		U3_BufferReadyIndication = 0;

		/* Clear Overrun flag, in case characters have already been sent to USART */
		LL_USART_ClearFlag_ORE(USART3);

		/* Enable RXNE and Error interrupts */
		LL_USART_EnableIT_RXNE(USART3);
		LL_USART_EnableIT_ERROR(USART3);
		OS_Signal(&U3_RxSemaphore);
		Count0++;
	}
}

void MU_PortHandleContinuousReception(void)
{
	Count1 = 0;
	uint8_t i;
	while(1){
		OS_Wait(&U3_RxSemaphore);
  /* Checks if Buffer full indication has been set */
		if (U3_BufferReadyIndication != 0)
		{
			DisableInterrupts();
			/* Reset indication */
			U3_BufferReadyIndication = 0;
			for(i = 0; i < RX_MESSAGE_SIZE; i++){
				U1_TXBuffer[i] = U3_RXBuffer[i];
			}
			EnableInterrupts();
			OS_Signal(&PortCtrlTxInitSema);
			OS_Wait(&U3_RxSemaphore);
		}
		OS_Signal(&U3_RxSemaphore);
		Count1++;
	}
}

void MU_PortSendMsg(void){
	Count2 = 0;
	while(1){
		OS_Wait(&U3_TxSemaphore);
		HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_SET);
		/* Fill DR with a new char */
		LL_USART_TransmitData8(USART3, U3_TXBuffer[U3_idxTx++]);
		/* Enable TXE interrupt */
		LL_USART_EnableIT_TXE(USART3); 
		OS_Signal(&U3_RxSemaphore);
		Count2++;
	}
}


void CtrlPortRxInit(void)
{
	Count7 = 0;
	while(1){
		OS_Wait(&PortCtrlRxInitSema); // Will signal in USART1_TransmitComplete_Callback function USART1.c
		/* Initializes Buffer indication : */
		U1_BufferReadyIndication = 0;
		/* Initializes time expiration semaphore */
		CtrlRxTimeIsNotExpired = 5;
		/* Initialize index to move on buffer */
		U1_idxRx = 0;
		pBufferReception 		= U1_RXBufferA;
		pBufferMessage      = U1_RXBufferB;
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
					U3_TXBuffer[i] = pBufferReception[i];
				}
				EnableInterrupts();
				OS_Signal(&U3_TxSemaphore);
				OS_Wait(&U1_RxSemaphore);
			}			
		} else {
			OS_Signal(&U3_TxSemaphore);
			OS_Wait(&U1_RxSemaphore);
		}
		Count8++;
	}
}

void CtrlPortTxInit(void)
{
	Count5 = 0;
	while(1){
		OS_Wait(&PortCtrlTxInitSema);
		if(!LL_USART_IsEnabledIT_ERROR(USART1)){
				/* Enable Error interrupt */
			LL_USART_EnableIT_ERROR(USART1);
		}
		CtrlPortRegistersTxInit();
		OS_Signal(&U1_TxSemaphore);
		Count5++;
	}
}

void CtrlPortSendMsg(void){
	Count6 = 0;
	while(1){
		OS_Wait(&U1_TxSemaphore);
		LL_USART_DisableDirectionRx(USART1);
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
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
	OS_Init();            // initialize, disable interrupts
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	USART1_Init();
	USART3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
	CtrlPortRegistersInit();
  /* USER CODE BEGIN 2 */

	OS_PeriodTrigger0_Init(&Time100msSemaphore, 100);
	OS_PeriodTrigger1_Init(&Time1secSemaphore, 1000);
	OS_InitSemaphore(&U3_RxInitSema, 1);
	OS_InitSemaphore(&U3_RxSemaphore, 0);
	OS_InitSemaphore(&U3_TxSemaphore, 0);
	OS_InitSemaphore(&Time100msSemaphore, 0);
	OS_InitSemaphore(&Time1secSemaphore, 0);
	
	OS_InitSemaphore(&PortCtrlRxInitSema, 0);
	OS_InitSemaphore(&U1_RxSemaphore, 0);
	OS_InitSemaphore(&PortCtrlTxInitSema, 0);
	OS_InitSemaphore(&U1_TxSemaphore, 0);
	
	
	OS_AddThread(&MU_PortReceptionInit, 0);
	OS_AddThread(&MU_PortHandleContinuousReception, 2);
	
	OS_AddThread(&CtrlPortTxInit, 0);	
	OS_AddThread(&CtrlPortSendMsg, 2);
	
	OS_AddThread(&CtrlPortRxInit, 0);
	OS_AddThread(&CtrlPortHandleContinuousReception, 2);
	
	OS_AddThread(&MU_PortSendMsg, 2);
	
	OS_AddThread(&EventThread100ms, 2);
	OS_AddThread(&EventThread1sec, 2);
	OS_AddThread(&IdleTask,7);     // lowest priority, dummy task
	OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			stop_cpu;// Should not be here
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
