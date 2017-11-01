/**
  ******************************************************************************
  * File Name          : MU_PortData.h
  * Description        : This file provides data definition
  *                      of the Monitor Unit port interface.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MU_PORTDATA_H
#define __MU_PORTDATA_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"
#include "stm32f1xx_hal.h"
#include "UAdapterProxy.h"
#include "PortProxy.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
	/* class MU_PortProxy */
/* USER CODE BEGIN Private defines */
	 struct RS485_ctrl {
		uint8_t Dir;			// Set direction pin to Hi with 1 and Lo with 0
		uint8_t DirDelay;
		uint8_t DirEnabled;
	 };
	 typedef struct RS485_ctrl RS485_ctrl;
	 	/* This is the proxy for hardware which belongs to Monitor Unit RS485 connection  */
	 struct MU_PortProxy{
		 PortProxy* Port;
		 RS485_ctrl MU_RS485;
	 };
	 typedef struct MU_PortProxy MU_PortProxy;


/* USER CODE END Private defines */
/* Create Port belongs to Monitor Unit with USARTx interface*/
MU_PortProxy* MU_PortCreate(void);
/* Disable Port belongs to Monitor Unit */
void MU_PortDisable(MU_PortProxy* const me);
/* Enable Port belongs to Monitor Unit after disable */
void MU_PortEnable(MU_PortProxy* const me);
/* Toggle Dir pin for RS485 transmition */
void ToggleDirPin(MU_PortProxy* const me);
/* Write to Monitor Unit port */	 
void MU_PortWrite(MU_PortProxy* const me);
/* Read from Monitor Unit port */	 
void MU_PortRead(MU_PortProxy* const me);
/* Destroy Monitor Unit port object */
void MU_PortProxyDestroy(MU_PortProxy* me);
	 
#ifdef __cplusplus
}
#endif
#endif /*__ MU_PORTDATA_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
