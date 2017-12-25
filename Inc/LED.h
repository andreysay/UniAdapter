/**
  ******************************************************************************
  * File Name          : LED.h
  * Description        : This file provides defenition
  *                      for LEDs I/O operations.
  ******************************************************************************
	**/
	/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H
#ifdef __cplusplus
 extern "C" {
#endif

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */
/* Private function prototypes -----------------------------------------------*/
void LEDs_Init(void);
	 
void LED_RedOn(void);
	 
void LED_GreenOn(void);
	 
void LED_OrangeOn(void);
	 
void ToggleLedRed(void);

void ToggleLedGreen(void);
	 
void ToggleLedOrange(void);
	 
void LEDs_off(void);
	 
#ifdef __cplusplus
}
#endif
#endif /*__LED_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
