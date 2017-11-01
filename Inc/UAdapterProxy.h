/**
  ******************************************************************************
  * File Name          : UAdapterProxy.h
  * Description        : This file provides code for the configuration
  *                      of the Monitor Unit port interface.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UADAPTERPROXY_H
#define __UADAPTERPROXY_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stdlib.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

struct LedProxy;
struct LedProxyData;
struct PortProxy;											// struct of methods to initialize, manage and access to UART port interface 
struct PortData;											// struct to store data for UART port interface
struct MU_PortProxy;									// This is the proxy for hardware which belongs to Monitor Unit RS485 connection 
struct ControllerPortProxy;							// struct of methods to initialize, manage and access to controller port interface
struct ControllerPortData;					// struct to store data for controller port interface
struct CFG_PortProxy;										// struct of methods to initialize, manage and access to CFG port interface										
struct CFG_PortData;								// struct to store data for CFG port interface
struct UAdapterProxy;								// struct of methods to initialize, manage and access to UAdapterProxy interfaces
struct UAdapterProxyData;						// struct to store data for UAdapterProxy interfaces

struct UART_ErrorStatus {
	uint8_t errorStatus;
	uint8_t UART_InitError;
	uint8_t UART_SendError;
	uint8_t UART_ReceiveError;
};

typedef struct UART_ErrorStatus UART_ErrorStatus;

struct UART_PortData{
	USART_TypeDef* UART_Instance;
	uint32_t UART_BAUDRATE;
	uint32_t UART_WORDLEN;
	uint32_t UART_STOPBITS;
	uint32_t UART_PARITY;
	uint32_t UART_MODE;
	uint32_t UART_HWFLOWCTL;
	uint32_t UART_OVERSAMPLING;
};

typedef struct UART_PortData UART_PortData;
/* USER CODE END Private defines */

	 
#ifdef __cplusplus
}
#endif
#endif /*__ UADAPTERPROXY_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT SAY *****END OF FILE****/
