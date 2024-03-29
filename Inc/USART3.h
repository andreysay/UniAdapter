/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART3_H
#define __USART3_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define USART3_BAUDRATE 9600
#define APB_Div3 2
/* USER CODE END Private defines */
//------------UART_Init------------
// Initialize the UART for 9,600 baud rate (assuming 72 MHz clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void USART3_Init(void);


#endif /* __USART3_H */
