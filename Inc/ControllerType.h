/**
  ******************************************************************************
  * File Name          : ControllerType.h
  * Description        : This file provides data definition for
  *                      Controller port usage.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLLERTYPE_H
#define __CONTROLLERTYPE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f1xx_hal.h"

#define Dixel 		1
#define Eliwell 	2
#define CarelEasy 3
#define CarelMPX	4
#define Unknown 65535
#define LED_ADC_BLINK_ERROR 3

/* USER CODE BEGIN Includes */
/** @addtogroup Controller_Included
  * @{
  */

/**
  * @}
  */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct TEvent
{
  uint8_t ev_addr;
	uint8_t ev_carel_addr;
  uint8_t ev_cmd;
  uint8_t *dataptr;
	uint8_t ev_size;
	uint8_t ev_vartype;
  uint8_t ev_debug;
	uint32_t ev_len;
	uint32_t ev_reg;
	int32_t  ev_regvalue;
} TEvent;
/* USER CODE END Private defines */

void ControllerTypeDetection(void);



#endif /*__CONTROLLERTYPE_H */

/**
  * @}
  */

/**
  * @}
  */
