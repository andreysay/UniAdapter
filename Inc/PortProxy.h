/**
  ******************************************************************************
  * File Name          : PortProxy.h
  * Description        : This file provides data definition
  *                      of the PortProxy UART interface.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MU_PORTPROXY_H
#define __MU_PORTPROXY_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "UAdapterProxy.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
	 
	/* class PortProxy */
typedef struct PortProxy PortProxy; // defined in UAdapterProxy.h
	/* This is the proxy for USART hardware which belongs to Monitor Unit RS485 connection  */	
struct PortProxy{
	UART_HandleTypeDef* 	UART_Handle;
};
/* Create Monitoring Unit Port proxy instance */
PortProxy* PortProxyCreate(UART_HandleTypeDef* USART_handle);
/* Enable Port interface to Monitoring Unit */
/* Should be called after Disable */
void PortProxyEnable(PortProxy* const me);
/* Disable Port interface to Monitoring Unit */
void PortProxyDisable(PortProxy* const me);
/* precondition: must be called AFTER configure() function */
/* turn on the hardware to a known default state */
void PortProxyInit(PortProxy* const me, UART_PortData* PortData);
/* Destroy PortProxy object */
void PortProxyDestroy(PortProxy* const me);

	 /* USER CODE END Private defines */

	 
#ifdef __cplusplus
}
#endif
#endif /*__ MU_PORTPROXY_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
