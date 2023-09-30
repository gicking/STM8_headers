/**
  \file power_saving.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of power-saving mode functions/macros
   
  declaration of power-saving mode functions/macros.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _POWER_SAVING_H_
#define _POWER_SAVING_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// enter WAIT mode: only stop CPU, wake via any interrupt
void lowPower_Wait(void);

/// enter HALT mode: all clocks off, wake only via EXINT
void lowPower_Halt(void);

/// enter active HALT mode: LSI active, wake via AWU or EXINT
void lowPower_HaltAWU(uint16_t ms);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _POWER_SAVING_H_
