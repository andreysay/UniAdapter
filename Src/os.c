#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "ErrorHandler.h"
#include "os.h"
#include "StartUP.h"
#include "BSP.h"


#define INTCTRL         (*((volatile uint32_t *)0xE000ED04))

/* Private define ------------------------------------------------------------*/
#define NUMTHREADS  30       // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread


// Thread data type
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  uint32_t Id;       // 0 means TCB is free
  int32_t *BlockPt;  // nonzero if blocked on this semaphore
  uint32_t Sleep;    // nonzero if this thread is sleeping
  uint32_t Priority; // 0 is highest
};
typedef struct tcb tcbType;


tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];
// function definitions in osasm.s
void StartOS(void);

void (*PeriodicTask1)(void); // pointer to periodic function

void static runperiodicevents(void);
uint32_t NumThread=0;  // number of threads
uint32_t static ThreadId=0;   // thread Ids are sequential from 1

void SystemClock_Config(void);
void static runperiodicevents(void);

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 72 MHz PLL
// input:  none
// output: none
void OS_Init(void){
	DisableInterrupts();
	NumThread=0;  // number of threads
  ThreadId=0;   // thread Ids are sequential from 1
	// set up periodic timer to run runperiodicevents to implement sleeping
  BSP_PeriodicTask_InitB(&runperiodicevents, 1000, 0);
}

//******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(void){
  /* Set Systick to 1ms in using frequency set to SystemCoreClock */
  LL_Init1msTick(SystemCoreClock);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 7);
	/* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 7);	
  StartOS();                   // start on the first task
}

// ****OS_Id**********
// returns the Id for the currently running thread
// Input:  none
// Output: Thread Id (1 to NUMTHREADS)
uint32_t OS_Id(void){
  return RunPt->Id;
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground function
//         priority (0 is highest)
// Outputs: Thread ID if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
int OS_AddThread(void(*task)(void), uint32_t priority){ int status;
  int n;         // index to new thread TCB 
  tcbType *NewPt;  // Pointer to nex thread TCB
  tcbType *LastPt; // Pointer to last thread TCB
  int32_t *sp;      // stack pointer
  status = StartCritical();
//  NewPt = malloc(stackSize+10);    // 9 extra bytes needed
  for(n=0; n< NUMTHREADS; n++){
    if(tcbs[n].Id == 0) break; // found it 
    if(n == NUMTHREADS-1){  
      EndCritical(status);
      return 0;          // heap is full
    }
  }
  NewPt = &tcbs[n]; 
  if(NumThread==0){
    RunPt = NewPt;  // points to first thread created
  } else{
    LastPt = RunPt;
    while(LastPt->next != RunPt){
      LastPt = LastPt->next;
    }
    LastPt->next = NewPt; // Pointer to Next  
  }
  NewPt->Priority =  priority;
  NumThread++;
  ThreadId++;
  NewPt->Id = ThreadId;
  NewPt->BlockPt =  0;    // not blocked
  NewPt->Sleep =  0;      // not sleeping

  sp = &Stacks[n][STACKSIZE-1];      // last entry of stack


// derived from uCOS-II
                                            /* Registers stacked as if auto-saved on exception    */
  *(sp)    = (long)0x01000000L;             /* xPSR , T=1                                         */
  *(--sp)  = (long)task;                    /* Entry Point                                        */
  *(--sp)  = (long)0x14141414L;             /* R14 (LR) (init value does not matter)              */
  *(--sp)  = (long)0x12121212L;             /* R12                                                */
  *(--sp)  = (long)0x03030303L;             /* R3                                                 */
  *(--sp)  = (long)0x02020202L;             /* R2                                                 */
  *(--sp)  = (long)0x01010101L;             /* R1                                                 */
  *(--sp)  = (long)0x000000000;             /* R0                                                 */

                                            /* Remaining registers saved on process stack         */
  *(--sp)  = (long)0x11111111L;             /* R11                                                */
  *(--sp)  = (long)0x10101010L;             /* R10                                                */
  *(--sp)  = (long)0x09090909L;             /* R9                                                 */
  *(--sp)  = (long)0x08080808L;             /* R8                                                 */
  *(--sp)  = (long)0x07070707L;             /* R7                                                 */
  *(--sp)  = (long)0x06060606L;             /* R6                                                 */
  *(--sp)  = (long)0x05050505L;             /* R5                                                 */
  *(--sp)  = (long)0x04040404L;             /* R4                                                 */
  NewPt->sp = sp;        // make stack "look like it was previously suspended"
  NewPt->next = RunPt;   // Pointer to first, circular linked list 
  EndCritical(status);
  return 1;
}

// runs every ms
void Scheduler(void){      // every time slice
// look at all threads in TCB list choose
// highest priority thread not blocked and not sleeping 
// If there are multiple highest priority (not blocked, not sleeping) run these round robin
	uint32_t max = 255; // max priority
	tcbType *pt;
	tcbType *bestPt;
	pt = RunPt;					// search for highest priority thread not blocked or sleeping
	do{
		pt = pt->next;    // skips at least one
		if((pt->Priority < max)&&((pt->BlockPt)==0)&&((pt->Sleep)==0)){
			max = pt->Priority;
			bestPt = pt;
		}
	} while(RunPt != pt); // look at all possible threads
	RunPt = bestPt;
//  RunPt->Priority = 250; // this thread will not run untill this activated by OS_Signal
}

//******** OS_Suspend ***************
// temporarily suspend the running thread and launch another
// Inputs: none
// Outputs: none
void OS_Suspend(void){
	SysTick->VAL = 0;
  SCB->ICSR = 0x04000000; // trigger SysTick by writing 1 to 26 bit -> http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/CIHFDJCA.html
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB memory
// input:  none
// output: none
tcbType *previousPt;   // Pointer to previous thread in list before current thread
tcbType *nextPt;       // Pointer to next thread in list after current thread
tcbType *killPt;       // Pointer to thread being killed
// RunPt will point to thread will be killed 
void OS_Kill(void){  // no local variables allowed
  DisableInterrupts();        // atomic
  NumThread--;
  if(NumThread==0){
    for(;;){};     // crash
  }
  RunPt->Sleep = 0xFFFFFFFF;  // can't rerun this thread, it will be dead
  killPt = RunPt;             // kill current thread
	Scheduler();                // RunPt points to thread to run next
//********initially RunPt points to thread to kill********
//               /----\           /----\          /----\
//               |    |  killPt-> |    |          |    |
//               |next----------> |next---------> |next--->
//               |    |           |    |          |    |
//               \----/           \----/          \----/
  killPt->Id = 0;         // mark as free
 
  previousPt = killPt;    // eventually, it will point to previous thread 
  nextPt = killPt->next;  // nextPt points to the thread after 
  while((previousPt->next)!=killPt){
    previousPt = previousPt->next;  
  }
//****previousPt -> one before, RunPt to thread to kill *********
//               /----\           /----\          /----\
// previousPt -> |    |  killPt-> |    | nextPt-> |    |
//               |next----------> |next---------> |next--->
//               |    |           |    |          |    |
//               \----/           \----/          \----/
  previousPt->next = nextPt; // remove from list
//****remove thread which we are killing *********
//               /----\                           /----\
// previousPt -> |    |                  nextPt-> |    |
//               |next--------------------------> |next--->
//               |    |                           |    |
//               \----/                           \----/
  SysTick->VAL = 0;        // next thread get full slice
  EnableInterrupts();
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; // Set PendSV to pending
//  INTCTRL = 0x10000000; // trigger pendSV to start next thread
  for(;;){};            // can not return
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
// set sleep parameter in TCB
// suspend, stops running
	RunPt->Sleep = sleepTime;
	OS_Suspend();
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
	*semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t* semPt){
	DisableInterrupts();
	(*semPt) = (*semPt) - 1;	
	if((*semPt) < 0)
	{
		RunPt->BlockPt = semPt;
		EnableInterrupts(); 
		OS_Suspend();// run thread switcher
	}
	EnableInterrupts();
}

// ******** OS_Signal ************
// Increment semaphore
// wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semPt){
	tcbType *pt;
	DisableInterrupts();
	(*semPt) = (*semPt) + 1;
	if((*semPt) <= 0){
		pt=RunPt->next;		// search for one blocked on this
		while(pt->BlockPt != semPt){
			pt = pt->next;
		}
		pt->BlockPt = 0; // wakeup this one
	}
	EnableInterrupts();
}

#define FSIZE 32    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint8_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initialize semaphores to track properties of the FIFO
// such as size and busy status for Put and Get operations,
// which is important if there are multiple data producers
// or multiple data consumers.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
	PutI = GetI = 0;
	OS_InitSemaphore(&CurrentSize, 0);
	LostData = 0; 
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is putting data into the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
	if(CurrentSize == FSIZE){
		LostData++;
		return -1;		// FIFO full
	} else{
		Fifo[PutI] = data; // Put
		PutI = (PutI+1)%FSIZE;
		OS_Signal(&CurrentSize);
	}	 
 return 0; // success
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is getting data from the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
	OS_Wait(&CurrentSize);	//block if empty
	data = Fifo[GetI]; // get
	GetI = (GetI+1)%FSIZE;
  return data;
  
}

void static runperiodicevents(void){
// **DECREMENT SLEEP COUNTERS
	for(int i = 0; i < NUMTHREADS; i++){
		if(tcbs[i].Sleep){
			tcbs[i].Sleep--;
		}
	}		
}


// *****periodic events****************
int32_t *PeriodicSemaphore0;
uint32_t Period0; // time between signals
int32_t *PeriodicSemaphore1;
uint32_t Period1; // time between signals
void RealTimeEvents(void){int flag=0;
  static int32_t realCount = -10; // let all the threads execute once
  // we had to let the system run for a time so all user threads ran at least one
  // before signalling the periodic tasks
  realCount++;
  if(realCount >= 0){
    if((realCount%Period0)==0){
      OS_Signal(PeriodicSemaphore0);
      flag = 1;
    }
    if((realCount%Period1)==0){
      OS_Signal(PeriodicSemaphore1);
      flag=1;
    }
    if(flag){
      OS_Suspend();
    }
  }
}
//****NOTE: this uses PeriodicTimer, not PeriodicTimerC***********
// ******** OS_PeriodTrigger0_Init ************
// Initialize periodic timer interrupt to signal 
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest)
// Outputs: none
void OS_PeriodTrigger0_Init(int32_t *semaPt, uint32_t period){
  PeriodicSemaphore0 = semaPt;
  Period0 = period;
  BSP_PeriodicTask_Init(&RealTimeEvents,1000,0);
}
// ******** OS_PeriodTrigger1_Init ************
// Initialize periodic timer interrupt to signal 
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest)
// Outputs: none
void OS_PeriodTrigger1_Init(int32_t *semaPt, uint32_t period){
  PeriodicSemaphore1 = semaPt;
  Period1 = period;
  BSP_PeriodicTask_Init(&RealTimeEvents,1000,0);
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
/* Variable to store PLL parameters */
/* Configuration will allow to reach a SYSCLK frequency set to 72MHz: 
   Syst freq = ((HSE_VALUE / LL_RCC_PLLSOURCE_HSI_DIV_1) * LL_RCC_PLL_MUL_9)
               ((8MHz /1) * 9)             = 72MHz                     */
LL_UTILS_PLLInitTypeDef sUTILS_PLLInitStruct = {LL_RCC_PLL_MUL_9, LL_RCC_PREDIV_DIV_1};

/* Variable to store AHB and APB buses clock configuration */
/* Settings to have HCLK set to 72MHz, APB1 to 36MHz and APB2 to 72MHz */
LL_UTILS_ClkInitTypeDef sUTILS_ClkInitStruct = {LL_RCC_SYSCLK_DIV_1, LL_RCC_APB1_DIV_2, LL_RCC_APB2_DIV_1};

void SystemClock_Config(void)
{
  /* System started with default clock used after reset */
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Switch to PLL with HSE as clock source             */
  LL_PLL_ConfigSystemClock_HSE(HSE_VALUE, LL_UTILS_HSEBYPASS_OFF, &sUTILS_PLLInitStruct, &sUTILS_ClkInitStruct);
  
  /* 
     CMSIS variable automatically updated according to new configuration.
     SystemCoreClock should be equal to calculated HCLK frequency.
     FLASH latency is also tuned according to system constraints described 
     in the reference manual.           
  */
	 /* Set Systick to 1ms in using frequency set to SystemCoreClock */
  LL_Init1msTick(SystemCoreClock);
}

