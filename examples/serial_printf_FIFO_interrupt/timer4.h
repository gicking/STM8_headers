/**
  \file timer4.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of timer TIM4 (1ms clock) functions/macros
   
  declaration of timer TIM4 functions as 1ms master clock.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _TIMER4_H_
#define _TIMER4_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>
#include "config.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_MAIN_'
#if defined(_MAIN_)
  volatile uint32_t           g_millis;                    ///< 1ms counter. Increased in TIM4 ISR
#else // _MAIN_
  extern volatile uint32_t    g_millis;
#endif // _MAIN_


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init timer 4 (1ms master clock)
void TIM4_init(void);

/// ISR for timer 4 (1ms master clock)
ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _TIMER4_H_
