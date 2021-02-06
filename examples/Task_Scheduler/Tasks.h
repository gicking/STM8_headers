/**
  \file     Tasks.h
  \brief    Library providing a simple task scheduler for multitasking.
  \details  This library implements a very basic scheduler that is executed via a 1ms timer 
            interrupt and also supports millis(), micros() etc. functions.
            It enables users to define cyclic tasks or tasks that should be executed in the future in 
            parallel to the normal program execution inside the main loop.
            <br>The task scheduler is executed every 1ms.
            <br>The currently running task is always interrupted by this and only continued to be executed
            after all succeeding tasks have finished.
            This means that always the task started last has the highest priority.
            This effect needs to be kept in mind when programming a software using this library.
            <br>Deadlocks can appear when one task waits for another taks which was started before.
            Additionally it is likely that timing critical tasks will not execute properly when they are
            interrupted for too long by other tasks.
            Thus it is recommended to keep the tasks as small and fast as possible.
            <br>This library is a STM8 port of the Arduino Task_Scheduler library available from 
            https://github.com/kcl93/Tasks which is published under MIT license.
            <br>As used STM8 timer TIM4 only supports an overflow interrupt, this port also implements
            standard Arduino time-keeping functions millis(), micros(), delay() and delayMicroseconds() 
  \author   Georg Icking-Konert
  \date     2020-02-17
  \version  1.0
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef TASKS_H
#define TASKS_H


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"      // STM8 selection


/*-----------------------------------------------------------------------------
    GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_TASKS_MAIN_'
#if defined(_TASKS_MAIN_)
  volatile uint8_t          g_flagMilli;       //!< flag for 1ms timer interrupt. Set in TIM4 ISR
  volatile uint32_t         g_millis;          //!< 1ms counter. Increased in TIM4 ISR
  volatile uint32_t         g_micros;          //!< 1000us counter. Increased in TIM4 ISR
#else // _TASKS_MAIN_
  extern volatile uint8_t   g_flagMilli;
  extern volatile uint32_t  g_millis;
  extern volatile uint32_t  g_micros;
#endif // _TASKS_MAIN_


/*-----------------------------------------------------------------------------
    GLOBAL MACROS
-----------------------------------------------------------------------------*/

#define flagMilli()         g_flagMilli        //!< 1ms flag. Set in 1ms ISR
#define clearFlagMilli()    g_flagMilli=0      //!< clear 1ms flag

#define MAX_TASK_CNT        8                  //!< Maximum number of parallel tasks


/*-----------------------------------------------------------------------------
    GLOBAL TYPEDEF
-----------------------------------------------------------------------------*/

/// Example prototype for a function than can be executed as a task
typedef void (*Task)(void);


/*-----------------------------------------------------------------------------
    GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
  \brief      Get microseconds since start of program
  \details    This function returns the microseconds since start of program. Resolution is 4us.
              Value overruns every ~1.2 hours.
              <br><br>Used HW blocks: TIM4
  \return     Microseconds since program start (resolution 4us)
*/
INLINE uint32_t micros(void) {

  uint8_t   cnt, uif;
  uint32_t  us;
  
  // for consistency of CNT ans SR briefly stop timer
  sfr_TIM4.CR1.CEN = 0;

  // get current us value, TIM4 counter, and TIM4 overflow flag
  cnt = sfr_TIM4.CNTR.byte;
  #if defined(STM8L_DISCOVERY)
    uif = sfr_TIM4.SR1.byte;
  #elif defined(SDUINO)
    uif = sfr_TIM4.SR.byte;
  #endif
  
  // restart timmer immediately to minimize time gap
  sfr_TIM4.CR1.CEN = 1;
  
  // calculate current time [us], including global variable (1000us steps) and counter value (4us steps)
  us  = g_micros;
  #if defined(__CSMC__)          // Cosmic compiler has a re-entrance bug with bitshift
    us += 4 * (uint16_t) cnt;
  #else
    us += ((uint16_t) cnt) << 2;
  #endif
  
  // account for possible overflow of TIM4 --> check UIF (= bit 0)
  if ((uif & 0x01) && (cnt != 250))
  us += 1000L;

  return(us);

} // micros()


/**
  \brief      Get milliseconds since start of program
  \details    This function returns the milliseconds since start of program. Resolution is 1ms.
              Value overruns every ~49.7 days.
              <br><br>Used HW blocks: TIM4
  \return     Milliseconds since program start (resolution 1ms)
*/
INLINE uint32_t millis(void) {

  return(g_millis);

} // millis()



/**
  \brief      Delay code execution for 'ms'
  \details    This function delays code execution for 'ms' milliseconds in steps of 1ms.
              <br><br>Used HW blocks: TIM4
  \param[in]  ms    Milliseconds to wait
*/
void delay(uint32_t ms);


/**
  \brief      Delay code execution for 'us'
  \details    This function delays code execution for 'us' microseconds in steps of 4us.
              <br><br>Used HW blocks: TIM4
  \param[in]  us    Microseconds to wait
*/
void delayMicroseconds(uint32_t us);



/**
  \brief      Initialize timer and reset the tasks scheduler at first call.
  \details    This function initializes the related timer and clears the task scheduler at first call.
              <br><br>Used HW blocks: TIM4
*/
void Tasks_Init(void);


/**
  \brief      Reset the tasks schedulder.
  \details    This function clears the task scheduler. Use with caution!
              <br><br>Used HW blocks:
              <br><br>Used HW blocks: TIM4
*/
void Tasks_Clear(void);


/**
  \brief      Add a task to the task scheduler.
  \details    A new task is added to the scheduler with a given execution period and delay until first execution.
              <br>If 0 delay is given the task is executed at once or after starting the task scheduler 
              (see Tasks_Start())
              <br>If a period of 0ms is given, the task is executed only once and then removed automatically.
              <br>To avoid ambiguities, a function can only be added once to the scheduler.
              Trying to add it a second time will reset and overwrite the settings of the existing task.
              <br><br>Used HW blocks:
              <br><br>Used HW blocks: TIM4
  \param[in]  func  Function to be executed.<br>The function prototype should be similar to this:
                    "void userFunction(void)"
  \param[in]  period  Execution period of the task in ms (0 to 32767; 0 = task only executes once) 
  \param[in]  delay   Delay until first execution of task in ms (0 to 32767)
  \return     true in case of success,
              false in case of failure (max. number of tasks reached, or duplicate function)
  \note       The maximum number of tasks is defined as <tt>MAX_TASK_CNT</tt> above
*/
bool Tasks_Add(Task func, int16_t period, int16_t delay);


/**
  \brief      Remove a task from the task scheduler.
  \details    Remove the specified task from the scheduler and free the slot again.
              <br><br>Used HW blocks:
              <br><br>Used HW blocks: TIM4
  \param[in]  func  Function name that should be removed.
  \return     true in case of success, 
              false in case of failure (e.g. function not in not in scheduler table)
*/
bool Tasks_Remove(Task func);


/**
  \brief      Delay execution of a task
  \details    The task is delayed starting from the last 1ms timer tick which means the delay time 
              is accurate to -1ms to 0ms.
              <br>This overwrites any previously set delay setting for this task and thus even allows
              earlier execution of a task.
              Delaying the task by <2ms forces it to be executed during the next 1ms timer tick.
              This means that the task might be called at any time anyway in case it was added multiple 
              times to the task scheduler.
              <br><br>Used HW blocks: TIM4
  \param[in]  func  Function that should be delayed
  \param[in]  delay Delay in ms (0 to 32767)
  \return     true in case of success, 
              false in case of failure (e.g. function not in not in scheduler table)
*/
bool Tasks_Delay(Task func, int16_t delay);


/**
  \brief      Enable or disable the execution of a task
  \details    Temporary pause or resume function for execution of single tasks by scheduler.
              This will not stop the task in case it is currently being executed but just prevents 
              the task from being executed again in case its state is set to 'false' (inactive).
              <br><br>Used HW blocks: TIM4
  \param[in]  func  Function to be paused/resumed.
                    <br>The function prototype should be similar to this: "void userFunction(void)"
  \param[in]  state New function state (false=pause, true=resume)
  \return     'true' in case of success, else 'false' (e.g. function not in not in scheduler table)
*/
bool Tasks_SetState(Task func, bool state);


/**
  \brief      Activate a task in the scheduler
  \details    Resume execution of the specified task. Possible parallel tasks are not affected. 
              This is a simple inlined function setting the 'state' argument for Tasks_SetState().
              <br><br>Used HW blocks: TIM4
  \param[in]  func  Function to be activated 
  \return     true in case of success, 
              false in case of failure (e.g. function not in not in scheduler table)
*/
INLINE bool Tasks_Start_Task(Task func)
{
  return Tasks_SetState(func, true);
}


/**
  \brief      Deactivate a task in the scheduler
  \details    Pause execution of the specified task. Possible parallel tasks are not affected. 
              This is a simple inlined function setting the 'state' argument for Tasks_SetState().
              <br><br>Used HW blocks: TIM4
  \param[in]  func  Function to be paused 
  \return     true in case of success, 
              false in case of failure (e.g. function not in not in scheduler table)
*/
INLINE bool Tasks_Pause_Task(Task func)
{
  return Tasks_SetState(func, false);
}


/**
  \brief      Start the task scheduler
  \details    Resume execution of the scheduler. All active tasks are resumed. 
              <br><br>Used HW blocks: TIM4
*/
void Tasks_Start(void);


/**
  \brief      Pause the task scheduler
  \details    Pause execution of the scheduler. All tasks are paused. 
              <br><br>Used HW blocks: TIM4
*/
void Tasks_Pause(void);


/// ISR for timer 4 (1ms master clock)
#if defined(_TIM4_OVR_UIF_VECTOR_)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_);
#elif defined(_TIM4_UIF_VECTOR_)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_UIF_VECTOR_);
#else
  #error TIM4 vector undefined
#endif

#endif // TASKS_H