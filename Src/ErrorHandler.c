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
#include "ErrorHandler.h"

/* USER CODE BEGIN 0 */
uint8_t ErrorBuff[255];
uint32_t ErrorLine;
  __IO uint32_t sr_reg;
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
	HAL_GPIO_WritePin(GPIO_LED_GRN, LED_GRN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_LED_RED, LED_RED_PIN, GPIO_PIN_SET);
  while(1) 
  {
		HAL_GPIO_TogglePin(GPIO_LED_RED, LED_RED_PIN);
		HAL_Delay(500);
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
    /* case Noise Error flag is raised : blinking red led */
		while(1){
			HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN);
			HAL_Delay(250);
		}
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
		while(1){
			HAL_GPIO_WritePin(GPIOB, LED_GRN_PIN, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN);
			HAL_Delay(1000);
		}
  }
}

void LED_ErrorBlinking(uint32_t Period)
{
  /* Turn RED on */
  LL_GPIO_SetOutputPin(GPIO_LED_RED, LED_RED_PIN);
  
  /* Toggle IO in an infinite loop */
  while (1)
  {
    /* Error if LED is slowly blinking (1 sec. period) */
    LL_GPIO_TogglePin(GPIO_LED_RED, LED_RED_PIN);  
    LL_mDelay(Period);
  }
}

/**
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(GPIO_LED_RED, LED_RED_PIN, LL_GPIO_MODE_OUTPUT);
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
