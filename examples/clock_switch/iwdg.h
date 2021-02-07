/**
  \file iwdg.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of IWDG functions/macros
   
  declaration of functions for the indepent watchdog (IWDG)
  IWDG runs on 128kHz slow clock and is a timeout watchdog.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _IWDG_H_
#define _IWDG_H_


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

// includes
#include <stdint.h>
#include "config.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize and start IWDG watchdog
void iwdg_init(uint8_t period);


/**
  \fn void iwdg_service(void)
   
  \brief service IWDG watchdog
  
  service the independent timeout watchdog (IWDG)
*/
INLINE void iwdg_service(void) {

  // trigger service of IWDG
  sfr_IWDG.KR.byte = (uint8_t) 0xAA;
    
} // iwdg_service


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _IWDG_H_
