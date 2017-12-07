// BSP.c


#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "BSP.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

/* Prescaler declaration */
static uint32_t uwPrescalerValue = 0;

// ------------BSP_Clock_GetFreq------------
// Return the current system clock frequency for the
// LaunchPad.
// Input: none
// Output: system clock frequency in cycles/second
uint32_t BSP_Clock_GetFreq(void){
  return SystemCoreClock;
}

// ------------BSP_PeriodicTask_Init------------
// Activate an interrupt to run a user task periodically.
// Give it a priority 0 to 6 with lower numbers
// signifying higher priority.  Equal priority is
// handled sequentially.
// Input:  task is a pointer to a user function
//         freq is number of interrupts per second
//           1 Hz to 10 kHz
//         priority is a number 0 to 6
// Output: none
//extern TIM_HandleTypeDef htim4;
void (*PeriodicTask)(void);   // user function
void BSP_PeriodicTask_Init(void(*task)(void), uint32_t freq, uint8_t priority){long sr;
  if((freq == 0) || (freq > 10000)){
    return;                        // invalid input
  }
  if(priority > 6){
    priority = 6;
  }
  sr = StartCritical();
  PeriodicTask = task;             // user function
	  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
	Update rate = TIM counter clock / (Period + 1) = 10 Hz,
  */
	

	uwPrescalerValue = (uint32_t)(SystemCoreClock / (freq * 10)) - 1;
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = uwPrescalerValue;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;	
	htim4.Init.Period = 10 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim4.Init.RepetitionCounter = 0;
  if(htim4.State == HAL_TIM_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htim4.Lock = HAL_UNLOCKED;
    
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
        /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, priority, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }

  /* Set the TIM state */
  htim4.State= HAL_TIM_STATE_BUSY;

  /* Set the Time Base configuration */
  TIM_Base_SetConfig(htim4.Instance, &htim4.Init);

  /* Initialize the TIM state*/
  htim4.State= HAL_TIM_STATE_READY;
	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    /* Starting Error */
    _Error_Handler(__FILE__, __LINE__);
  }
  EndCritical(sr);
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	(*PeriodicTask)();               // execute user task
  /* USER CODE END TIM4_IRQn 1 */
}

// ------------BSP_PeriodicTask_Stop------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
void BSP_PeriodicTask_Stop(void){
	HAL_TIM_Base_Stop_IT(&htim4);
}
// ------------BSP_PeriodicTask_Restart------------
// Reactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void BSP_PeriodicTask_Restart(void){
	HAL_TIM_Base_Start_IT(&htim4);
}

// ------------BSP_PeriodicTask_InitB------------
// Activate an interrupt to run a user task periodically.
// Give it a priority 0 to 6 with lower numbers
// signifying higher priority.  Equal priority is
// handled sequentially.
// Input:  task is a pointer to a user function
//         freq is number of interrupts per second
//           1 Hz to 10 kHz
//         priority is a number 0 to 6
// Output: none

void (*PeriodicTaskB)(void);   // user function
void BSP_PeriodicTask_InitB(void(*task)(void), uint32_t freq, uint8_t priority){long sr;
  if((freq == 0) || (freq > 10000)){
    return;                        // invalid input
  }
  if(priority > 6){
    priority = 6;
  }
  sr = StartCritical();
  PeriodicTaskB = task;             // user function
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / (freq * 10) ) - 1;
  htim3.Instance = TIM3;
	htim3.Init.Period = 10 - 1;
  htim3.Init.Prescaler = uwPrescalerValue;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim3.Init.RepetitionCounter = 0;
  if(htim3.State == HAL_TIM_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htim3.Lock = HAL_UNLOCKED;
    
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
        /* TIM4 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, priority, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }

  /* Set the TIM state */
  htim3.State= HAL_TIM_STATE_BUSY;

  /* Set the Time Base configuration */
  TIM_Base_SetConfig(htim3.Instance, &htim3.Init);

  /* Initialize the TIM state*/
  htim3.State= HAL_TIM_STATE_READY;
	  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    /* Starting Error */
    _Error_Handler(__FILE__, __LINE__);
  }
  EndCritical(sr);
}

void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	  (*PeriodicTaskB)();               // execute user task
  /* USER CODE END TIM3_IRQn 1 */
}

// ------------BSP_PeriodicTask_StopB------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
void BSP_PeriodicTask_StopB(void){
	HAL_TIM_Base_Stop_IT(&htim3);
}


// ------------BSP_PeriodicTask_InitC------------
// Activate an interrupt to run a user task periodically.
// Give it a priority 0 to 6 with lower numbers
// signifying higher priority.  Equal priority is
// handled sequentially.
// Input:  task is a pointer to a user function
//         freq is number of interrupts per second
//           1 Hz to 10 kHz
//         priority is a number 0 to 6
// Output: none
//void (*PeriodicTaskC)(void);   // user function
//void BSP_PeriodicTaskC_Init(void(*task)(void), uint32_t freq, uint8_t priority){long sr;
//  if((freq == 0) || (freq > 10000)){
//    return;                        // invalid input
//  }
//  if(priority > 6){
//    priority = 6;
//  }
//  sr = StartCritical();
//  PeriodicTaskC = task;             // user function
//  // ***************** Wide Timer3A initialization *****************
//  SYSCTL_RCGCWTIMER_R |= 0x08;     // activate clock for Wide Timer3
//  while((SYSCTL_PRWTIMER_R&0x08) == 0){};// allow time for clock to stabilize
//  WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;// disable Wide Timer3A during setup
//  WTIMER3_CFG_R = TIMER_CFG_16_BIT;// configure for 32-bit timer mode
//                                   // configure for periodic mode, default down-count settings
//  WTIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
//  WTIMER3_TAILR_R = (ClockFrequency/freq - 1); // reload value
//  WTIMER3_TAPR_R = 0;              // bus clock resolution
//  WTIMER3_ICR_R = TIMER_ICR_TATOCINT;// clear WTIMER3A timeout flag
//  WTIMER3_IMR_R |= TIMER_IMR_TATOIM;// arm timeout interrupt
////PRIn Bit   Interrupt
////Bits 31:29 Interrupt [4n+3]
////Bits 23:21 Interrupt [4n+2]
////Bits 15:13 Interrupt [4n+1]
////Bits 7:5   Interrupt [4n]  , n=25 => (4n+0)=100
//  NVIC_PRI25_R = (NVIC_PRI25_R&0xFFFFFF00)|(priority<<5); // priority
//// interrupts enabled in the main program after all devices initialized
//// vector number 116, interrupt number 100
//// 32 bits in each NVIC_ENx_R register, 100/32 = 3 remainder 4
//  NVIC_EN3_R = 1<<4;               // enable IRQ 100 in NVIC
//  WTIMER3_CTL_R |= TIMER_CTL_TAEN; // enable Wide Timer3A 32-b
//  EndCritical(sr);
//}

//void WideTimer3A_Handler(void){
//  WTIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge Wide Timer3A timeout
//  (*PeriodicTaskC)();               // execute user task
//}

// ------------BSP_PeriodicTask_StopC------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
//void BSP_PeriodicTaskC_Stop(void){
//  WTIMER3_ICR_R = TIMER_ICR_TATOCINT;// clear WTIMER3A timeout flag
//  NVIC_DIS3_R = 1<<4;              // disable IRQ 100 in NVIC
//}

// ------------BSP_Time_Init------------
// Activate a 32-bit timer to count the number of
// microseconds since the timer was initialized.
// Input: none
// Output: none
// Assumes: BSP_Clock_InitFastest() has been called
//          so clock = 80/80 = 1 MHz
void BSP_Time_Init(void){long sr;
  sr = StartCritical();
  // ***************** Wide Timer4 initialization *****************
	MX_TIM4_Init();
  EndCritical(sr);
}

