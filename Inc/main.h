/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f1xx_ll_tim.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define GFG_OUT_PA2_Pin GPIO_PIN_2
#define GFG_OUT_PA2_GPIO_Port GPIOA
#define GPIO_OUT_LED1_PB1_Pin GPIO_PIN_1
#define GPIO_OUT_LED1_PB1_GPIO_Port GPIOB
#define GPIO_OUT_LED2_PB2_Pin GPIO_PIN_2
#define GPIO_OUT_LED2_PB2_GPIO_Port GPIOB
#define USART3_PB10_TX_Pin GPIO_PIN_10
#define USART3_PB10_TX_GPIO_Port GPIOB
#define USART3_PB11_RX_Pin GPIO_PIN_11
#define USART3_PB11_RX_GPIO_Port GPIOB
#define GPIO_OUT_PB12_Pin GPIO_PIN_12
#define GPIO_OUT_PB12_GPIO_Port GPIOB
#define GPIO_OUT_PB13_Pin GPIO_PIN_13
#define GPIO_OUT_PB13_GPIO_Port GPIOB
#define GPIO_OUT_PB15_Pin GPIO_PIN_15
#define GPIO_OUT_PB15_GPIO_Port GPIOB
#define USART1_PA9_TX_Pin GPIO_PIN_9
#define USART1_PA9_TX_GPIO_Port GPIOA
#define USART1_PA10_RX_Pin GPIO_PIN_10
#define USART1_PA10_RX_GPIO_Port GPIOA
#define GPIO_OUT_PA11_Pin GPIO_PIN_11
#define GPIO_OUT_PA11_GPIO_Port GPIOA
#define GPIO_OUT_PA12_Pin GPIO_PIN_12
#define GPIO_OUT_PA12_GPIO_Port GPIOA
#define SYS_JTMS_SWDIO_PA13_Pin GPIO_PIN_13
#define SYS_JTMS_SWDIO_PA13_GPIO_Port GPIOA
#define SYS_JTCK_SWCLK_PA14_Pin GPIO_PIN_14
#define SYS_JTCK_SWCLK_PA14_GPIO_Port GPIOA
#define GPIO_OUT_PA15_Pin GPIO_PIN_15
#define GPIO_OUT_PA15_GPIO_Port GPIOA
#define GPIO_OUT_PB3_Pin GPIO_PIN_3
#define GPIO_OUT_PB3_GPIO_Port GPIOB
#define GPIO_OUT_PB4_Pin GPIO_PIN_4
#define GPIO_OUT_PB4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define GPIO_LED_GRN			GPIO_OUT_LED1_PB1_GPIO_Port
#define LED_GRN_PIN				GPIO_OUT_LED1_PB1_Pin
#define GPIO_LED_RED			GPIO_OUT_LED2_PB2_GPIO_Port
#define LED_RED_PIN				GPIO_OUT_LED2_PB2_Pin
#define PIN_EnaTx					GPIO_OUT_PB3_Pin
#define GPIO_EnaTx				GPIO_OUT_PB3_GPIO_Port
#define PIN_EnaRx					GPIO_OUT_PB4_Pin
#define GPIO_EnaRx				GPIO_OUT_PB4_GPIO_Port
#define PIN_TXmb					USART3_PB10_TX_Pin
#define GPIO_TXmb					USART3_PB10_TX_GPIO_Port
#define PIN_RXmb					USART3_PB11_RX_Pin
#define GPIO_RXmb					USART3_PB11_RX_GPIO_Port
#define PIN_Dir						GPIO_OUT_PB12_Pin
#define GPIO_Dir					GPIO_OUT_PB12_GPIO_Port
#define PIN_TxB						GPIO_OUT_PB13_Pin
#define GPIO_TxB					GPIO_OUT_PB13_GPIO_Port
#define PIN_TxA						GPIO_OUT_PB15_Pin
#define GPIO_TxA					GPIO_OUT_PB15_GPIO_Port
#define PIN_RxA						GPIO_OUT_PA11_Pin
#define GPIO_RxA 					GPIO_OUT_PA11_GPIO_Port
#define PIN_Ena						GPIO_OUT_PA12_Pin
#define GPIO_Ena 					GPIO_OUT_PA12_GPIO_Port
#define PIN_EnaB					GPIO_OUT_PA15_Pin
#define GPIO_EnaB					GPIO_OUT_PA15_GPIO_Port
/* USER CODE END Private defines */
#define RX_BUFFER_SIZE   32
#define RX_MESSAGE_SIZE		8

#define DEBUGVIEW	1

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT *****END OF FILE****/
