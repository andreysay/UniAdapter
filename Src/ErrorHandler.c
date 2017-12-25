/**
  ******************************************************************************
  * File Name          : ErrorHandler.c
  * Description        : This file provides code for the Error Handler
  *                      
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "ErrorHandler.h"

/* USER CODE BEGIN 0 */
uint8_t ErrorBuff[255];
uint32_t ErrorLine;
__IO uint32_t sr_reg;
extern __IO uint8_t TIM2_InterruptFlag;
extern __IO uint8_t HardFaultFlag;
/* USER CODE END 0 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	memcpy(ErrorBuff, file, 50);
	ErrorLine = (uint32_t)line;
	if(!HardFaultFlag){
		LED_ErrorBlinking(9);
	}
  /* USER CODE END Error_Handler_Debug */ 
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void USARTxError_Callback(USART_TypeDef* USARTx)
{
	
	if( USARTx == USART1 ){
		/* Disable USARTx_IRQn */
		NVIC_DisableIRQ(USART1_IRQn);
  
		/* Error handling example :
			- Read USART SR register to identify flag that leads to IT raising
			- Perform corresponding error handling treatment according to flag
		*/
		sr_reg = LL_USART_ReadReg(USART1, SR);
	} else if ( USARTx == USART3 ){
		  /* Disable USARTx_IRQn */
		NVIC_DisableIRQ(USART3_IRQn);
  
		/* Error handling example :
			- Read USART SR register to identify flag that leads to IT raising
			- Perform corresponding error handling treatment according to flag
		*/
		sr_reg = LL_USART_ReadReg(USART3, SR);
	}
	
  if (sr_reg & LL_USART_SR_NE)
  {
    /* case Noise Error flag is raised : blinking red led 300ms */
		_Error_Handler(__FILE__, __LINE__);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
		_Error_Handler(__FILE__, __LINE__);
  }
}

/**
  * @brief  Initialize TIM2 with default 10Hz frequency for red LED error blinking
  * @param  Period - time interval, 0 mean 100ms, 1 - 200ms, .... 9 - 1000ms, 0 <= Period <= 9
  * @retval None
  */
void LED_ErrorBlinking(uint32_t Period)
{
  /* Configure the timer time base */
  Configure_TIM2TimeBase(10000, 10);
	LED_Init();
	/* Set time for error RED blinking */
	SetTIM2_Period(Period);
	while(1){
		if(TIM2_InterruptFlag){
			LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_2);
			TIM2_InterruptFlag = 0;
		}
	}
}

/**
  * @brief  Initialize RED led.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
