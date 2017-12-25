/**
  ******************************************************************************
  * File Name          : ControllerPort.c
  * Description        : This file provides code for the configuration and usage
  *                    : controller port.  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ControllerPort.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

CtrlPortReg CtrlPortRegisters;


void RS485_Init(void){
	CtrlPortRegisters.RxA = 	GPIO_PIN_SET;
	CtrlPortRegisters.TxB = 	GPIO_PIN_SET;
	CtrlPortRegisters.TxA = 	GPIO_PIN_SET;
	CtrlPortRegisters.EnaB = 	GPIO_PIN_SET;
	HAL_GPIO_WritePin(GPIO_RxA, PIN_RxA, CtrlPortRegisters.RxA);
	HAL_GPIO_WritePin(GPIO_TxB, PIN_TxB, CtrlPortRegisters.TxB);	
	HAL_GPIO_WritePin(GPIO_TxA, PIN_TxA, CtrlPortRegisters.TxA);
	HAL_GPIO_WritePin(GPIO_EnaB, PIN_EnaB, CtrlPortRegisters.EnaB);
}

void Dixel_Init(void){
	CtrlPortRegisters.RxA   =	GPIO_PIN_RESET;
	CtrlPortRegisters.TxB   =	GPIO_PIN_RESET;
	CtrlPortRegisters.TxA   =	GPIO_PIN_SET;
	CtrlPortRegisters.EnaB  = GPIO_PIN_SET;
	CtrlPortRegisters.EnaRx = GPIO_PIN_SET;
	CtrlPortRegisters.EnaTx = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(GPIO_RxA, PIN_RxA, CtrlPortRegisters.RxA);
	HAL_GPIO_WritePin(GPIO_TxB, PIN_TxB, CtrlPortRegisters.TxB);	
	HAL_GPIO_WritePin(GPIO_TxA, PIN_TxA, CtrlPortRegisters.TxA);
	HAL_GPIO_WritePin(GPIO_EnaB, PIN_EnaB, CtrlPortRegisters.EnaB);
	HAL_GPIO_WritePin(GPIO_EnaRx, PIN_EnaRx, CtrlPortRegisters.EnaRx);
	HAL_GPIO_WritePin(GPIO_EnaTx, PIN_EnaTx, CtrlPortRegisters.EnaTx);
}

void CtrlPortRegistersTxInit(void){
	CtrlPortRegisters.EnaRx = GPIO_PIN_SET;
	CtrlPortRegisters.EnaTx = GPIO_PIN_SET;
	HAL_GPIO_WritePin(GPIO_EnaRx, PIN_EnaRx, CtrlPortRegisters.EnaRx);
	HAL_GPIO_WritePin(GPIO_EnaTx, PIN_EnaTx, CtrlPortRegisters.EnaTx);	
}

void CtrlPortRegistersRxInit(void){
	CtrlPortRegisters.EnaRx = GPIO_PIN_RESET;
	CtrlPortRegisters.EnaTx = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(GPIO_EnaRx, PIN_EnaRx, CtrlPortRegisters.EnaRx);
	HAL_GPIO_WritePin(GPIO_EnaTx, PIN_EnaTx, CtrlPortRegisters.EnaTx);	
}



/************************ (C) COPYRIGHT *****END OF FILE****/
