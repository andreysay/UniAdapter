/**
  ******************************************************************************
  * File Name          : MU_PortProxy.c
  * Description        : This file provides code for the configuration
  *                      of the MU_Portproxy instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MU_PortProxy.h"
#include "usart.h"
/* USER CODE BEGIN 0 */
#define MU_UART_Instance			USART3
#define MU_UART_BaudRate 			9600
#define MU_UART_WordLength 		UART_WORDLENGTH_8B
#define MU_UART_StopBits 			UART_STOPBITS_1
#define MU_UART_Parity 				UART_PARITY_NONE
#define MU_UART_Mode 					UART_MODE_TX_RX
#define MU_UART_HwFlowCtl 		UART_HWCONTROL_NONE
#define MU_UART_OverSampling 	UART_OVERSAMPLING_16

MU_PortProxy* MU_PortCreate(void){
	UART_PortData MU_PortData;
	MU_PortData.UART_Instance = 		MU_UART_Instance;
	MU_PortData.UART_BAUDRATE = 		MU_UART_BaudRate;
	MU_PortData.UART_WORDLEN = 			MU_UART_WordLength;
	MU_PortData.UART_STOPBITS = 		MU_UART_StopBits;
	MU_PortData.UART_PARITY = 			MU_UART_Parity;
	MU_PortData.UART_MODE = 				MU_UART_Mode;
	MU_PortData.UART_HWFLOWCTL = 		MU_UART_HwFlowCtl;
	MU_PortData.UART_OVERSAMPLING = MU_UART_OverSampling;
	
	MU_PortProxy* MU_Port = (MU_PortProxy *)malloc(sizeof(MU_PortProxy));
	MU_Port->Port = PortProxyCreate(&huart3);
	PortProxyInit(MU_Port->Port, &MU_PortData);
	
	return MU_Port;
}

void MU_PortDisable(MU_PortProxy* const me){
	
}

void MU_PortEnable(MU_PortProxy* const me){
	
}

void ToggleDirPin(MU_PortProxy* const me){
	
}

void MU_PortWrite(MU_PortProxy* const me){
	
}

void MU_PortRead(MU_PortProxy* const me){
	
}

void MU_PortProxyDestroy(MU_PortProxy* me){
	
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
