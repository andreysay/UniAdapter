#ifndef __STARTUP_H
#define __STARTUP_H  1
// these functions are defined in the startup file

//******DisableInterrupts************
// sets the I bit in the PRIMASK to disable interrupts
// Inputs: none
// Outputs: none
void DisableInterrupts(void); // Disable interrupts

//******EnableInterrupts************
// clears the I bit in the PRIMASK to enable interrupts
// Inputs: none
// Outputs: none
void EnableInterrupts(void);  // Enable interrupts

//******StartCritical************
// StartCritical saves a copy of PRIMASK and disables interrupts
// Code between StartCritical and EndCritical is run atomically
// Inputs: none
// Outputs: copy of the PRIMASK (I bit) before StartCritical called
long StartCritical(void);    

//******EndCritical************
// EndCritical sets PRIMASK with value passed in
// Code between StartCritical and EndCritical is run atomically
// Inputs: PRIMASK (I bit) before StartCritical called
// Outputs: none
void EndCritical(long sr);    // restore I bit to previous value

//******WaitForInterrupt************
// enters low power sleep mode waiting for interrupt (WFI instruction)
// processor sleeps until next hardware interrupt
// returns after ISR has been run
// Inputs: none
// Outputs: none
void WaitForInterrupt(void);  

// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n);

#endif
