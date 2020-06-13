/**
  \file timer3.h
   
  \brief declaration of TIM3 functions/macros 
   
  supported hardware:
    - Sduino
  
  declaration of functions for PWM generation via TIM3
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _TIMER3_H_
#define _TIMER3_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "config.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init timer 3
void TIM3_init(void);

/// set PWM frequency [0.01Hz] for all channels
void TIM3_setFrequency(uint32_t centHz);

/// set PWM duty cycle [0.1%] for single compare channel
void TIM3_setDutyCycle(uint8_t channel, uint16_t deciPrc);

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _TIMER3_H_
