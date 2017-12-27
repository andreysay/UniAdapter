/**
  ******************************************************************************
  * File Name          : USART3.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "ErrorHandler.h"
#include "os.h"
#include "LED.h"
#include "USART3.h"

//#define stop_U3_idle   __breakpoint(1)
extern uint8_t DebugModbusBuf[];

extern __IO uint32_t U3_idxTx; // defined in main_threads.c file
extern __IO uint32_t U3_idxRx; // defined in main_threads.c file
extern uint8_t *U3_pBufferTransmit; // defined in main_threads.c file
extern uint8_t *U3_pBufferReception; // defined in main_threads.c file
extern __IO uint32_t     U3_BufferReadyIndication; // defined in main_threads.c file
extern uint8_t U3_TxMessageSize;	// defined in main_threads.c file
extern uint8_t U3_RxMessageSize;	// defined in main_threads.c file
extern int32_t U3_RxSemaphore;	// defined in main_threads.c file

//------------UART3_Init------------
/**
  * @brief  This function configures USARTx Instance.
  * @note   This function is used to :
  *         -1- Enable GPIO clock, USART clock and configures the USART pins.
  *         -2- NVIC Configuration for USART interrupts.
  *         -3- Configure USART functional parameters.
  *         -4- Enable USART.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
// Initialize the UART3 for 9,600 baud rate (assuming 72 MHz clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void USART3_Init(void){
  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* Enable USART peripheral clock *******************************************/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Input Floating function, High Speed, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_FLOATING);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USART3_IRQn, 0);  
  NVIC_EnableIRQ(USART3_IRQn);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  // LL_USART_Disable(USARTx_INSTANCE);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  LL_USART_SetHWFlowCtrl(USART3, LL_USART_HWCONTROL_NONE);

  /* Set Baudrate to 9600 using APB frequency set to 72000000/APB_Div Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 72000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
  */
  LL_USART_SetBaudRate(USART3, SystemCoreClock/APB_Div3, USART3_BAUDRATE); 

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USART3);
  EnableInterrupts();
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART3_TransmitComplete_Callback(void)
{
	DebugModbusBuf[13] = U3_idxTx;
	if(U3_idxTx >= U3_TxMessageSize)
  { 
		U3_idxTx = 0;
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USART3);
		/* Set DE pin to low level (TC interrupt is occured )*/
		HAL_GPIO_WritePin(GPIO_Dir, PIN_Dir, GPIO_PIN_RESET);
    /* Turn LEDs Off at end of transfer : Tx sequence completed successfully */
		HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_RESET);
		OS_Signal(&U3_RxSemaphore); // Signal semaphore to start accept new request from MU Port
  }
}

/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART_TXEmpty_Callback(void)
{
  if(U3_idxTx >= (U3_TxMessageSize - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART3);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USART3);
  }
  /* Fill DR with a new char */
  LL_USART_TransmitData8(USART3, U3_pBufferTransmit[U3_idxTx++]);
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART3_Reception_Callback(void)
{
	if(!U3_idxRx){
		/* Turn GREEN led on, indication that data from controller received */
		LED_GreenOn();
	}
  /* Read Received character. RXNE flag is cleared by reading of DR register */
  U3_pBufferReception[U3_idxRx++] = LL_USART_ReceiveData8(USART3);
	// Check that we're not owerflow buffer size
	if(U3_idxRx >= (RX_BUFFER_SIZE - 1)){	
		U3_idxRx = 0;
	}
}

/**
  * @brief  Function called from USART IRQ Handler when IDLE flag is set
  *         Function is in charge of signal to main thread CtrlPortHandleContinuousReception
  *					that reception of data has been end.
  * @param  None
  * @retval None
  */
void USART3_IDLE_Callback(void){
	if(U3_idxRx < RX_BUFFER_SIZE){
		/* Idle detected, Buffer full indication has been set */
		U3_BufferReadyIndication = 1;
		/* Save received data size */
		U3_RxMessageSize = U3_idxRx;
		/* Initiliaze Buffer index to zero */
		U3_idxRx = 0;
		/* Clear IDLE status */
		LL_USART_ClearFlag_IDLE(USART3);
		OS_Signal(&U3_RxSemaphore);
	}	else {
		U3_idxRx = 0;
	}		
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART3) && LL_USART_IsEnabledIT_RXNE(USART3))   /* Check RXNE flag value in SR register */
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
		USART3_Reception_Callback();
  }	
	if(LL_USART_IsEnabledIT_TXE(USART3) && LL_USART_IsActiveFlag_TXE(USART3))
	{
		/* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART_TXEmpty_Callback();
	}
	if(LL_USART_IsEnabledIT_TC(USART3) && LL_USART_IsActiveFlag_TC(USART3))
	{
		/* Clear TC flag */
		LL_USART_ClearFlag_TC(USART3);
		/* Call function in charge of handling end of transmission of sent character
				and prepare next charcater transmission */
		USART3_TransmitComplete_Callback();
  }
	if(LL_USART_IsActiveFlag_IDLE(USART3) && LL_USART_IsEnabledIT_IDLE(USART3))   /* Check IDLE flag value in SR register */
  {
    /* IDLE flag will be cleared by a software sequence (an read to the
				USART_SR register followed by a read to the USART_DR register (done in call) */
    /* Call function in charge of handling Idle interrupt */
		USART3_IDLE_Callback();
  }	
  /* USER CODE END USART3_IRQn 0 */

  /* USER CODE BEGIN USART3_IRQn 1 */
  if(LL_USART_IsEnabledIT_ERROR(USART3) && LL_USART_IsActiveFlag_NE(USART3))
  {
    /* Call Error function */
    USARTxError_Callback(USART3);
  }
  /* USER CODE END USART3_IRQn 1 */
}
