/**
  ******************************************************************************
  * File Name          : Led.c
  * Description        : This file provides code for the LEDs
  *                      I/O operations.
  ******************************************************************************
	*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "main.h"
#include "LED.h"




/**
  * @brief  Initialize RED&GREEN leds.
  * @param  None
  * @retval None
  */
void LEDs_Init(void)
{
  /* Enable the LED2 Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	
	/* Configure IO in output push-pull mode to drive external LED GREEn */
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);

  /* Configure IO in output push-pull mode to drive external LED RED */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
}

void LED_RedOn(void){
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);
}

void LED_GreenOn(void){
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);	
}

void LED_OrangeOn(void){
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);
}

void ToggleLedRed(void){
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_2);
}

void ToggleLedGreen(void){
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_1);	
}

void ToggleLedOrange(void){
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_1);
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_2);
}

void LEDs_off(void){
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
