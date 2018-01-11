/**
  ******************************************************************************
  * File Name          : ControllerPort.h
  * Description        : This file provides data definition for
  *                      Controller port usage.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLLERPORT_H
#define __CONTROLLERPORT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f1xx_hal.h"

/**
  * @}
  */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
	 
struct ControllerPortData {
	GPIO_PinState EnaRx;
	GPIO_PinState EnaTx;
	GPIO_PinState RxA;
	GPIO_PinState TxB;
	GPIO_PinState TxA;
	GPIO_PinState EnaB;
};

typedef struct ControllerPortData CtrlPortReg;


void CtrlPortRegistersInit(void);

void CtrlPortRegistersTxInit(void);

void CtrlPortRegistersRxInit(void);

void CtrlPortReg_Init(void);

void RS485_Init(void);


#endif /*__CONTROLLERPORT_H */

/**
  * @}
  */

/**
  * @}
  */
