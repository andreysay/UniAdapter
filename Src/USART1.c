/**
  ******************************************************************************
  * File Name          : USART1.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "ErrorHandler.h"
#include "LED.h"
#include "USART1.h"
#include "ControllerPort.h"
#include "ControllerType.h"
	
extern int32_t U1_RxSemaphore; // defined in main_threads.c file
extern __IO uint32_t U1_idxTx; // defined in main_threads.c file
extern uint8_t *U1_pBufferTransmit; // defined in main_threads.c file
extern uint8_t *U1_pBufferReception; // defined in main_threads.c file
extern __IO uint32_t U1_idxRx; // defined in main_threads.c file
extern __IO uint32_t     U1_BufferReadyIndication; // defined in main_threads.c file
extern CtrlPortReg CtrlPortRegisters;	// defined in ControllerPort.c
extern __IO uint8_t U1_RxMessageSize; // defined in main_threads.c file
extern uint8_t U1_TxMessageSize; // defined in main_threads.c file
extern uint32_t ControllerType;

__IO uint8_t Televis1stByteFlag = 1;


//------------UART1_Init------------
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
// Initialize the UART1 for 9,600 baud rate (assuming 72 MHz clock),
// 8 bit word length, no parity bits, two stop bit
// Input: none
// Output: none
void USART1_Init(void){

  /* (1) Enable GPIO clock and configures the USART pins *********************/

  /* Enable the peripheral clock of GPIO Port */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /* Enable USART peripheral clock *******************************************/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  /* Configure Rx Pin as : Input Floating function, High Speed, Pull up */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_FLOATING);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0);  
  NVIC_EnableIRQ(USART1_IRQn);

  /* (3) Configure USART functional parameters ********************************/

  /* Disable USART prior modifying configuration registers */
  /* Note: Commented as corresponding to Reset value */
  LL_USART_Disable(USART1);

  /* TX/RX direction */
  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);

  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_2);

  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);

  /* Set Baudrate to 9600 using APB frequency set to 72000000/APB_Div Hz */
  /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
  /* Ex :
      Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance
  
      In this example, Peripheral Clock is expected to be equal to 72000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
  */
  LL_USART_SetBaudRate(USART1, SystemCoreClock/APB_Div1, USARTx_BAUDRATE);

  /* (4) Enable USART *********************************************************/
  LL_USART_Enable(USART1);
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART1_TransmitComplete_Callback(void)
{
	if(U1_idxTx >= U1_TxMessageSize)
  { 
		U1_idxTx = 0;
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USART1);
#ifdef RS485
		/* Set EnaRx and EnaTx pin in reception mode */
		CtrlPortRegisters.EnaRx = GPIO_PIN_RESET;
		CtrlPortRegisters.EnaTx = GPIO_PIN_RESET;		
		HAL_GPIO_WritePin(GPIO_EnaRx, PIN_EnaRx, CtrlPortRegisters.EnaRx);
		HAL_GPIO_WritePin(GPIO_EnaTx, PIN_EnaTx, CtrlPortRegisters.EnaTx);
		LL_USART_EnableDirectionRx(USART1);
#endif
    /* Turn ORANGE Off at end of transfer : Tx sequence completed successfully */
		LEDs_off();
		/* Initializes Buffer indication : */
		U1_BufferReadyIndication = 0;
		Televis1stByteFlag = 1;
  }
}

/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART1_TXEmpty_Callback(void)
{
  if(U1_idxTx >= (U1_TxMessageSize - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USART1);
  }
  /* Fill DR with a new char */
  LL_USART_TransmitData8(USART1, U1_pBufferTransmit[U1_idxTx++]);
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART1_Reception_Callback(void)
{
	if(!U1_idxRx){
		/* Turn GREEN led on, indication that data from controller received */
		LED_GreenOn();
	}
		/* Read Received character. RXNE flag is cleared by reading of DR register */
	U1_pBufferReception[U1_idxRx++] = LL_USART_ReceiveData8(USART1);

	// check that we're not overflow buffer size
	if(U1_idxRx >= (RX_BUFFER_SIZE - 1)){
		U1_BufferReadyIndication = 1;
	/* Save received data size */
		U1_RxMessageSize = U1_idxRx;		
	/* Initiliaze Buffer index to zero */		
		U1_idxRx = 0;		
	}
}

/**
  * @brief  Function called from USART IRQ Handler when IDLE flag is set
  *         Function is in charge of signal to main thread CtrlPortHandleContinuousReception
  *					that reception of data has been end.
  * @param  None
  * @retval None
  */
void USART1_IDLE_Callback(void){
	if(U1_idxRx >= 2){ // If recived 2 bytes or more, otherwise it uncomplete message   
		/* Idle detected, Buffer full indication has been set */
		U1_BufferReadyIndication = 1;
		/* Save received data size */
		U1_RxMessageSize = U1_idxRx;
		/* Initiliaze Buffer index to zero */
		U1_idxRx = 0;
		/* Signal thread that data received */
		OS_Signal(&U1_RxSemaphore);
	}
		/* Clear IDLE status */
	LL_USART_ClearFlag_IDLE(USART1);	
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))   /* Check RXNE flag value in SR register */
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
		USART1_Reception_Callback();
  }	
	if(LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1))
	{
		/* TXE flag will be automatically cleared when writing new data in DR register */
		/* Call function in charge of handling empty DR => will lead to transmission of next character */
		/* Disable RXNE interrupt */
    USART1_TXEmpty_Callback();
	}
	if(LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1))
	{
		/* Clear TC flag */
		LL_USART_ClearFlag_TC(USART1);
		/* Call function in charge of handling end of transmission of sent character
				and prepare next charcater transmission */
		USART1_TransmitComplete_Callback();
  }
	if(LL_USART_IsActiveFlag_IDLE(USART1) && LL_USART_IsEnabledIT_IDLE(USART1))   /* Check IDLE flag value in SR register */
  {
    /* IDLE flag will be cleared by a software sequence (an read to the
				USART_SR register followed by a read to the USART_DR register (done in call) */
    /* Call function in charge of handling Idle interrupt */
		USART1_IDLE_Callback();
  }	
	
  /* USER CODE END USART3_IRQn 0 */

  /* USER CODE BEGIN USART3_IRQn 1 */
  if(LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))
  {
    /* Call Error function */
    USARTxError_Callback(USART1);
  }
  /* USER CODE END USART3_IRQn 1 */
}
