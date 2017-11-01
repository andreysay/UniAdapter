/**
  ******************************************************************************
  * File Name          : PortProxy.c
  * Description        : This file provides code for the implementation
  *                      of the PortProxy instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"
#include "PortProxy.h"

/* CLASS MU_PORTPROXY 0 */
/* Configure must be called first, since it set up the address of the device */
static void PortProxyConfigure(PortProxy* const me, UART_HandleTypeDef* USART_handle);

PortProxy* PortProxyCreate(UART_HandleTypeDef* USART_handle){
	PortProxy* Port = (PortProxy *)malloc(sizeof(PortProxy));
	PortProxyConfigure(Port, USART_handle);
	return Port;
}

void PortProxyConfigure(PortProxy* const me, UART_HandleTypeDef* USART_handle){
	me->UART_Handle = USART_handle;
}

void PortProxyInit(PortProxy* const me, UART_PortData* PortData){
	me->UART_Handle->Instance = PortData->UART_Instance;
	me->UART_Handle->Init.BaudRate = PortData->UART_BAUDRATE;
	me->UART_Handle->Init.WordLength = PortData->UART_WORDLEN;
	me->UART_Handle->Init.StopBits = PortData->UART_STOPBITS;
	me->UART_Handle->Init.Parity = PortData->UART_PARITY;
	me->UART_Handle->Init.Mode = PortData->UART_MODE;
	me->UART_Handle->Init.HwFlowCtl = PortData->UART_HWFLOWCTL;
	me->UART_Handle->Init.OverSampling = PortData->UART_OVERSAMPLING;
	if (HAL_UART_Init(me->UART_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void PortProxyDisable(PortProxy* const me){
	HAL_UART_MspDeInit(me->UART_Handle);
}

void PortProxyEnable(PortProxy* const me){
	HAL_UART_MspInit(me->UART_Handle);
}

void PortProxyDestroy(PortProxy* me){
	if(me != NULL)
	{
		PortProxyDisable(me);
	}
	
	free(me);
	me = NULL;
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
