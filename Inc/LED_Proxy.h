/**
  ******************************************************************************
  * File Name          : 
  * Description        : This file provides data definition
  *                      of the  interface.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_PROXY_H
#define __LED_PROXY_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "UAdapterProxy.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
	 
/* class LedProxy */
	 
/* USER CODE BEGIN Private defines */
struct LedProxy{
	GPIO_TypeDef * 	Led_GPIO;
	uint16_t				Led_PIN;
};

typedef struct LedProxy LedProxy;

LedProxy* LedProxyCreate(GPIO_TypeDef* xGPIO, uint16_t xPIN);
void LedOn(LedProxy* const me);
void LedOff(LedProxy* const me);
void LedToggle(LedProxy* const me, uint32_t time_milSec);
/* USER CODE END Private defines */

	 
#ifdef __cplusplus
}
#endif
#endif /*__ LED_PROXY_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
