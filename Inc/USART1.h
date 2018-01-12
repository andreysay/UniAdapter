/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART1_H
#define __USART1_H
 /* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Private defines */
/* USART1 instance is used. (TX on PA.09, RX on PA.10)
   (requires wiring USART1 TX/Rx Pins to USB to UART adapter) */
#define USARTx_INSTANCE               USART1
#define USARTx_CLK_ENABLE()           LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define USARTx_IRQn                   USART1_IRQn
#define USARTx_IRQHandler             USART1_IRQHandler

#define USARTx_GPIO_CLK_ENABLE()      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USARTx_TX_PIN                 LL_GPIO_PIN_9
#define USARTx_TX_GPIO_PORT           GPIOA
#define USARTx_RX_PIN                 LL_GPIO_PIN_10
#define USARTx_RX_GPIO_PORT           GPIOA
#define USARTx_BAUDRATE 9600
#define USARTx_Carel_BAUDRATE 19200
#define APB_Div1 1

/* USER CODE END Private defines */

//------------UART_Init------------
// Initialize the UART for 9,600 baud rate (assuming 72 MHz clock),
// 8 bit word length, no parity bits, two stop bit
// Input: none
// Output: none
void USART1_Init(void);
// Initialize the UART1 for 19,200 baud rate (assuming 72 MHz clock),
// 8 bit word length, no parity bits, one stop bit, single wire communication
// Input: none
// Output: none
void USART1_CarelEasyInit(void);


#endif /* __USART1_H */
