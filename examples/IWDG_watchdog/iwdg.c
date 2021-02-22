/**
  \file iwdg.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of IWDG functions/macros
   
  implementation of functions for the indepent watchdog (IWDG)
  IWDG runs on 128kHz slow clock and is a timeout watchdog.
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "iwdg.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void iwdg_init(uint8_t period)
   
  \brief initialize and start IWDG watchdog
  
  \param[in]  period  IWDG timeout period in [ms]
   
  initialize and start independent timeout watchdog (IWDG). Notes:
    - IWDG can be started by SW or option bytes (OPT3/NOPT3)
    - once started, IWDG cannot be stopped by software
*/
void iwdg_init(uint8_t period) {

  // start IDWG (must be the first value written to this register, see UM)
  sfr_IWDG.KR.byte  = (uint8_t) 0xCC;     
  
  // unlock write access to prescaler and reload registers
  sfr_IWDG.KR.byte  = (uint8_t) 0x55;
  
  // set clock to 1kHz (=64kHz/2^(PR+2))
  sfr_IWDG.PR.byte  = (uint8_t) 0x04;
  
  // set timeout period
  sfr_IWDG.RLR.byte = period;
  
  // start IDWG
  sfr_IWDG.KR.byte  = (uint8_t) 0xCC;
    
} // iwdg_init

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
