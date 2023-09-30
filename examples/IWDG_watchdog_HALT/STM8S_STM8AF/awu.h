/**
  \file awu.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of auto-wake functions/macros
   
  declaration of auto-wake functions/macros.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _AWU_H_
#define _AWU_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// ISR for AWU
ISR_HANDLER(AWU_ISR, _AWU_VECTOR_);

/// configure auto-wake
void AWU_setTime(uint16_t ms);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _AWU_H_
