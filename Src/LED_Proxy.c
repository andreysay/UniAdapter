/**
  ******************************************************************************
  * File Name          : LED_Proxy.c
  * Description        : This file provides code for the configuration
  *                      of the LED instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LED_Proxy.h"

/* USER CODE BEGIN 0 */
LedProxy* LedProxyCreate(GPIO_TypeDef* xGPIO, uint16_t xPIN){
	LedProxy* Led = (LedProxy *)malloc(sizeof(LedProxy));
	Led->Led_GPIO = xGPIO;
	Led->Led_PIN = xPIN;
	
	return Led;
}

void LedOn(LedProxy* const me){
	HAL_GPIO_WritePin(me->Led_GPIO, me->Led_PIN, GPIO_PIN_SET);
}

void LedOff(LedProxy* const me){
	HAL_GPIO_WritePin(me->Led_GPIO, me->Led_PIN, GPIO_PIN_RESET);
}

void LedToggle(LedProxy* const me, uint32_t time_milSec){
	HAL_GPIO_TogglePin(me->Led_GPIO, me->Led_PIN);
	HAL_Delay(time_milSec);
}
/* USER CODE END 0 */



/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
