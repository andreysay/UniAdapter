/**
  ******************************************************************************
  * File Name          : Dixel.h
  * Description        : This file contains the defines of the Modbus protocol
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_THREADS_H
#define __MAIN_THREADS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

void EventThread1sec(void);

void EventThread100ms(void);

void MU_PortReceptionInit(void);

void MU_PortHandleContinuousReception(void);

void MU_PortSendMsg(void);

void CtrlPortRxInit(void);

void CtrlPortHandleContinuousReception(void);

void CtrlPortTxInit(void);

void CtrlPortSendMsg(void);

void IdleTask(void);

#endif /* __MAIN_THREADS_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
