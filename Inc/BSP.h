// BSP.h
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

// ------------BSP_Clock_GetFreq------------
// Return the current system clock frequency for the
// LaunchPad.
// Input: none
// Output: system clock frequency in cycles/second
uint32_t BSP_Clock_GetFreq(void);

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
void BSP_PeriodicTask_Init(void(*task)(void), uint32_t freq, uint8_t priority);

// ------------BSP_PeriodicTask_Stop------------
// Deactivate the interrupt running a user task
// periodically.
// The automatic grader in TExaS calls this function and
// uses this hardware.  Do not call this function if using
// the automatic grader.
// Input: none
// Output: none
void BSP_PeriodicTask_Stop(void);

// ------------BSP_PeriodicTask_Restart------------
// Reactivate the interrupt running a user task periodically.
// Input: none
// Output: none
void BSP_PeriodicTask_Restart(void);

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
void BSP_PeriodicTask_InitB(void(*task)(void), uint32_t freq, uint8_t priority);
 
// ------------BSP_PeriodicTask_StopB------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
void BSP_PeriodicTask_StopB(void);

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
void BSP_PeriodicTask_InitC(void(*task)(void), uint32_t freq, uint8_t priority);

// ------------BSP_PeriodicTask_StopC------------
// Deactivate the interrupt running a user task
// periodically.
// Input: none
// Output: none
void BSP_PeriodicTask_StopC(void);

// ------------BSP_Time_Init------------
// Activate a 32-bit timer to count the number of
// microseconds since the timer was initialized.
// Input: none
// Output: none
// Assumes: BSP_Clock_InitFastest() has been called
void BSP_Time_Init(void);

// ------------BSP_Time_Get------------
// Return the system time in microseconds, which is the
// number of 32-bit timer counts since the timer was
// initialized.  This will work properly for at least 23
// minutes after which it could roll over.
// Input: none
// Output: system time in microseconds
// Assumes: BSP_Time_Init() has been called
uint32_t BSP_Time_Get(void);

// ------------BSP_Delay1ms------------
// Simple delay function which delays about n
// milliseconds.
// Inputs: n  number of 1 msec to wait
// Outputs: none
void BSP_Delay1ms(uint32_t n);
#endif /* __BSP_H */
