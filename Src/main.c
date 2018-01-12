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
#include "Carel.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* it can be changed to while(1) if needed */
#define stop_cpu   __breakpoint(0)

/* Semaphores defined in main_threads.c file */
extern int32_t Time100msSemaphore;
extern int32_t Time1secSemaphore;
// Semaphore for transmit
extern int32_t U3_RxSemaphore;
// Semaphore for reception
extern int32_t U3_TxSemaphore;
// Semaphore for reception initialization
extern int32_t U3_RxInitSema;

// Semaphore for Controller port initialization
extern int32_t PortCtrlTxInitSema;
extern int32_t PortCtrlRxInitSema;
extern int32_t U1_TxSemaphore;
extern int32_t U1_RxSemaphore;

// link to Televis.c file
extern int32_t TelevisHndlReceiveSema;
extern int32_t TelevisPortTxInitSema;
extern int32_t TelevisPortRxInitSema;
extern int32_t CtrlScanSema;
extern int32_t TelevisHndlSendSema;

// link to Modbus.c file
extern int32_t ModbusSendSema;
extern int32_t ModbusHndlReceiveSema;
extern int32_t ModbusPortTxInitSema;

// link to ControllerType.c file
extern bool DeviceFound;

// link to Carel.c file
extern int32_t CarelHndlReceiveSema;

// Variable to store connected controller Type
// which will detect by voltage on CFG pin 0 - 400mVolt = Dixel, 3000 - 3300mVolt = Eliwell
uint32_t ControllerType = Unknown;

// Buffer for Televis/Carel and etc messages
uint8_t ProtocolBuf[RX_BUFFER_SIZE];
// Variable to store size of received message
uint32_t ProtocolMsgSize;

/* Private function prototypes -----------------------------------------------*/
// link to os.c file
extern void SystemClock_Config(void);


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
	// Detect input voltage from CFG pin and setup connected controller type
	ControllerTypeDetection();
	// Disable ADC conversion
	Disable_ADC();
	
	OS_Init();            // initialize, disable interrupts
	/* USER CODE BEGIN 2 */
	switch(ControllerType){
		case Dixel:
			DeviceFound = true;
			CtrlPortReg_Init();
		  /* TRIGGER INITIALIZATION CODE BEGIN */
			OS_PeriodTrigger0_Init(&Time100msSemaphore, 100);
			OS_PeriodTrigger1_Init(&Time1secSemaphore, 1000);
			/* TRIGGER INITIALIZATION CODE END */
		
			/* SEMAPHORE INITIALIZATION CODE BEGIN */
			OS_InitSemaphore(&U3_RxInitSema, 1);
			OS_InitSemaphore(&U3_RxSemaphore, 0);
			OS_InitSemaphore(&U3_TxSemaphore, 0);
			OS_InitSemaphore(&Time100msSemaphore, 0);
			OS_InitSemaphore(&Time1secSemaphore, 0);	
			OS_InitSemaphore(&PortCtrlRxInitSema, 0);
			OS_InitSemaphore(&U1_RxSemaphore, 0);
			OS_InitSemaphore(&PortCtrlTxInitSema, 0);
			OS_InitSemaphore(&U1_TxSemaphore, 0);
			/* SEMAPHORE INITIALIZATION CODE END */
		
			/* THREADs INITIALIZATION CODE BEGIN */
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
			/* THREADs INITIALIZATION CODE END */
			break;
		case Eliwell:
			CtrlPortReg_Init();
			TIM2TimeInit();
			/* TRIGGER INITIALIZATION CODE BEGIN */
			OS_PeriodTrigger0_Init(&Time100msSemaphore, 100);
			OS_PeriodTrigger1_Init(&Time1secSemaphore, 1000);
			/* TRIGGER INITIALIZATION CODE END */
		
			/* SEMAPHORE INITIALIZATION CODE BEGIN */
			// Initialize CtrlScanSema to 1 to start run with that thread
			OS_InitSemaphore(&CtrlScanSema, 1);
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
			OS_InitSemaphore(&Time1secSemaphore, 0);	
			/* SEMAPHORE INITIALIZATION CODE END */

			/* THREADs INITIALIZATION CODE BEGIN */
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

			OS_AddThread(&EventThread1sec, 4);
			OS_AddThread(&IdleTask,7);     // lowest priority, dummy task
			OS_Launch(); // doesn't return, interrupts enabled in here	
			/* THREADs INITIALIZATION CODE END */
			break;
		case CarelEasy:
			CtrlPortReg_Init();
			TIM2TimeInit();
			USART1_CarelEasyInit();
			/* TRIGGER INITIALIZATION CODE BEGIN */
			OS_PeriodTrigger0_Init(&Time100msSemaphore, 100);
			OS_PeriodTrigger1_Init(&Time1secSemaphore, 1000);
			/* TRIGGER INITIALIZATION CODE END */
		
			/* SEMAPHORE INITIALIZATION CODE BEGIN */
			// Initialize CtrlScanSema to 1 to start run with that thread
			OS_InitSemaphore(&CtrlScanSema, 1);
			OS_InitSemaphore(&TelevisPortTxInitSema, 0);
			OS_InitSemaphore(&U1_TxSemaphore, 0);
			OS_InitSemaphore(&PortCtrlRxInitSema, 0);
			OS_InitSemaphore(&U1_RxSemaphore, 0);
			OS_InitSemaphore(&CarelHndlReceiveSema, 0);
			OS_InitSemaphore(&U3_RxInitSema, 0);
			/* SEMAPHORE INITIALIZATION CODE END */
		
			/* THREADs INITIALIZATION CODE BEGIN */
			OS_AddThread(&CarelScan, 2);
		
			OS_AddThread(&CarelPortTxInit, 2);
			OS_AddThread(&CarelPortSendMsg, 3);
			
			OS_AddThread(&CarelPortRxInit, 2);			
			OS_AddThread(&CarelPortReception, 3);
			
			OS_AddThread(&CarelHndlReceived, 2);
			
			OS_AddThread(&EventThread1sec, 4);
			OS_AddThread(&IdleTask,7);     // lowest priority, dummy task
			OS_Launch(); // doesn't return, interrupts enabled in here		
			/* THREADs INITIALIZATION CODE END */
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
#ifdef APDEBUG		
		stop_cpu;// Should not be here
#endif
		LED_ErrorBlinking(LED_BLINK_ERROR);
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
