/**
  ******************************************************************************
  * File Name          : 
  * Description        : This file provides data definition
  *                      of the IWDG interface.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H
#define __IWDG_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_iwdg.h"
#include "tim.h"
#include "LED.h"
#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
void     Check_IWDG_Reset(void);
void     Configure_IWDG(void);
/* USER CODE END Private defines */

	 
#ifdef __cplusplus
}
#endif
#endif /*__ IWDG_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
