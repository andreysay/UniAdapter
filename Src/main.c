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
#include "USART1.h"
#include "ControllerType.h"
#include "ControllerPort.h"
#include "main_threads.h"
#include "Televis.h"
#include "Modbus.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* it can be changed to while(1) if needed */
#define stop_cpu   __breakpoint(0)

// Counters for debugging
uint32_t Count3, Count4;

// Semaphore for periodic thread which runs every 100ms
int32_t Time100msSemaphore;
// Semaphore for periodic thread which runs every 1 second
int32_t Time1secSemaphore;

/* Semaphores defined in main_threads.c file */
// Semaphore for transmit
extern int32_t U3_RxSemaphore;
// Semaphore for reception
extern int32_t U3_TxSemaphore;
// Semaphore for reception initialisation
extern int32_t U3_RxInitSema;

// Semaphore for Controller port initialisation
extern int32_t PortCtrlTxInitSema;
extern int32_t PortCtrlRxInitSema;
extern int32_t U1_TxSemaphore;
extern int32_t U1_RxSemaphore;

// link to Televis.c file
extern int32_t TelevisHndlReceiveSema;
extern int32_t TelevisPortTxInitSema;
extern int32_t TelevisPortRxInitSema;
extern int32_t TelevisCtrlScanSema;
extern int32_t TelevisHndlSendSema;

// link to Modbus.c file
extern int32_t ModbusSendSema;
extern int32_t ModbusHndlReceiveSema;
extern int32_t ModbusPortTxInitSema;

uint32_t ControllerType = Unknown;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void EventThread100ms(void){
	Count3 = 0;
	while(1){
		OS_Wait(&Time100msSemaphore);           // 1000 Hz real time task
		Count3++;
	};
}
void EventThread1sec(void){ 
  Count4 = 0;
	while(1){
		OS_Wait(&Time1secSemaphore);
		HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOB, LED_GRN_PIN);
		Count4++;
	};
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/
  /* USER CODE BEGIN Init */
	SystemClock_Config();// set processor clock to fastest speed
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	USART1_Init();
	USART3_Init();
  /* Configure ADC */
  /* Note: This function configures the ADC but does not enable it.           */
  /*       To enable it, use function "Activate_ADC()".                       */
  /*       This is intended to optimize power consumption:                    */
  /*       1. ADC configuration can be done once at the beginning             */
  /*          (ADC disabled, minimal power consumption)                       */
  /*       2. ADC enable (higher power consumption) can be done just before   */
  /*          ADC conversions needed.                                         */
  /*          Then, possible to perform successive "Activate_ADC()",          */
  /*          "Deactivate_ADC()", ..., without having to set again            */
  /*          ADC configuration.                                              */
  Configure_ADC();
  
  /* Activate ADC */
  /* Perform ADC activation procedure to make it ready to convert. */
  Activate_ADC();
	// Detect input voltage from CFG pin
	ControllerTypeDetection();
	// Disable ADC conversion
	Disable_ADC();
	
	OS_Init();            // initialize, disable interrupts
	/* USER CODE BEGIN 2 */
	switch(ControllerType){
		case Dixel:
			Dixel_Init();

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
	
			OS_AddThread(&MU_PortReceptionInit, 2);
			OS_AddThread(&MU_PortHandleContinuousReception, 3);
		
			OS_AddThread(&CtrlPortTxInit, 2);	
			OS_AddThread(&CtrlPortSendMsg, 3);
		
			OS_AddThread(&CtrlPortRxInit, 2);
			OS_AddThread(&CtrlPortHandleContinuousReception, 3);
			OS_AddThread(&MU_PortSendMsg, 0);
			OS_AddThread(&EventThread100ms, 4);
			OS_AddThread(&EventThread1sec, 4);
			OS_AddThread(&IdleTask,7);     // lowest priority, dummy task
			OS_Launch(); // doesn't return, interrupts enabled in here
			break;
		case Eliwell:
			Dixel_Init();
			TelevisTIM2TimeInit();
			OS_PeriodTrigger0_Init(&Time100msSemaphore, 100);
			OS_PeriodTrigger1_Init(&Time1secSemaphore, 1000);
			OS_InitSemaphore(&TelevisCtrlScanSema, 1);
			OS_InitSemaphore(&TelevisPortTxInitSema, 0);
			OS_InitSemaphore(&U1_TxSemaphore, 0);		
			OS_InitSemaphore(&TelevisPortRxInitSema, 0);
			OS_InitSemaphore(&U1_RxSemaphore, 0);
			OS_InitSemaphore(&TelevisHndlReceiveSema, 0);		
			OS_InitSemaphore(&ModbusSendSema, 0);
			OS_InitSemaphore(&ModbusPortTxInitSema, 0);
			OS_InitSemaphore(&U3_TxSemaphore, 0);
			OS_InitSemaphore(&U3_RxSemaphore, 0);
			OS_InitSemaphore(&ModbusHndlReceiveSema, 0);
			OS_InitSemaphore(&TelevisHndlSendSema, 0);
			OS_InitSemaphore(&Time100msSemaphore, 0);
			OS_InitSemaphore(&Time1secSemaphore, 0);	


	
			OS_AddThread(&TelevisScan, 2);
		
			OS_AddThread(&TelevisPortTxInit, 2);
			OS_AddThread(&TelevisPortSendMsg, 3);
			OS_AddThread(&TelevisSend, 3);			
		
			OS_AddThread(&TelevisPortRxInit, 2);
			OS_AddThread(&TelevisPortReception, 3);
			OS_AddThread(&TelevisHndlReceive, 2);
			
			OS_AddThread(&ModbusPortRxInit, 2);
			OS_AddThread(&ModbusPortReception, 3);
			OS_AddThread(&ModbusHndlReceive, 3);
			
			OS_AddThread(&ModbusSend, 3);
			OS_AddThread(&ModbusPortTxInit, 2);
			OS_AddThread(ModbusPortSendMsg, 3);

			OS_AddThread(&TelevisEventThread100ms, 4);
			OS_AddThread(&TelevisEventThread1sec, 4);
			OS_AddThread(&IdleTask,7);     // lowest priority, dummy task
			OS_Launch(); // doesn't return, interrupts enabled in here			
			break;
		default:
			EnableInterrupts();
			LED_ErrorBlinking(LED_BLINK_ERROR);
	}
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef DEBUGVIEW		
		stop_cpu;// Should not be here
#endif
		HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
  }

}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{

//}

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
