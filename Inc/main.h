/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

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

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
