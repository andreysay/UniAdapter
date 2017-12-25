/**
  ******************************************************************************
  * File Name          : ErrorHandler.h
  * Description        : This file provides data definition for
  *                      Error handling.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERRORHANDLER_H
#define __ERRORHANDLER_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/**
  * @brief Toggle periods for various blinking modes, 0 <= period <= 9
  */
#define LED_BLINK_FAST  2
#define LED_BLINK_SLOW  5
#define LED_BLINK_ERROR 9	 
	 
void LED_Init(void);	 
	 
void LED_ErrorBlinking(uint32_t Period);	 
	 
void _Error_Handler(char *, int);
	 
/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */	 
void USARTxError_Callback(USART_TypeDef* USARTx);
/* USER CODE END Private defines */

	 
#ifdef __cplusplus
}
#endif
#endif /*__ ERRORHANDLER_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
