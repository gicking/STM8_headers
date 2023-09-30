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
  
  // set IWDG frequency to 1kHz and period in ms
  iwdg_set_period(0x04, period);
    
} // iwdg_init


/**
  \fn void iwdg_set_period(uint8_t prescaler, uint8_t reload)
   
  \brief change IWDG timeout period
  
  \param[in]  prescaler   IWDG clock prescaler (freq=64kHz/2^(PR+2))
  \param[in]  reload      IWDG reload value
   
  set independent watchdog (IWDG) timeout period
*/
void iwdg_set_period(uint8_t prescaler, uint8_t reload) {

  // unlock write access to prescaler and reload registers
  sfr_IWDG.KR.byte  = (uint8_t) 0x55;
  
  // set clock prescaler (freq=64kHz/2^(PR+2))
  sfr_IWDG.PR.byte  = prescaler;
  
  // set reload counter
  sfr_IWDG.RLR.byte = reload;
  
  // re-lock IWDG registers and start new period
  sfr_IWDG.KR.byte  = (uint8_t) 0xAA;
    
} // iwdg_set_period

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
