/**
  \file power_saving.c
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief implementation of power-saving mode functions/macros
   
  implementation of power-saving mode functions/macros.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "power_saving.h"
#include "awu.h"


/**
  \fn void lowPower_Wait(void)
   
  \brief enter WAIT mode: only stop CPU, wake via any interrupt
  
  Enter CPU WAIT mode: only stop the CPU, all clocks are still running.
  Wake is possible via any internal or external interrupt.
*/
void lowPower_Wait(void) {

  uint8_t  tmp;
  
  // disable the 1ms interrupt. Remember current state 
  tmp = sfr_TIM4.IER.UIE;
  sfr_TIM4.IER.UIE = 0;

  // enter WAIT mode
  WAIT_FOR_INTERRUPT();  

  // after wake revert 1ms ISR state
  sfr_TIM4.IER.UIE = tmp;

} // lowPower_Wait
     


/**
  \fn void lowPower_Halt(void)
   
  \brief enter HALT mode: all clocks off, wake only via EXINT
  
  Enter active HALT mode: all clocks off, wake only via EXINT
*/
void lowPower_Halt(void) {

  // switch off main regulator during halt mode
  sfr_CLK.ICKR.REGAH = 1;
  
  // power down flash during halt mode
  sfr_FLASH.CR1.AHALT = 1;

  // enter HALT mode
  ENTER_HALT();  

} // lowPower_Halt
       


/**
  \fn void lowPower_HaltAWU(uint16_t ms)
   
  \brief enter active HALT mode: LSI active, wake via AWU or EXINT
  
  \param[in]  ms    sleep duration [ms] within [1;30000]

  Enter active HALT mode: only LSI active, wake via AWU or EXINT.
  If no external interrupt occurs within 'ms', the AWU resumes
  operation automatically.
  
  Note: requires a AWU_ISR to be implemented
*/
void lowPower_HaltAWU(uint16_t ms) {

  // configure AWU for wake after 'ms'
  AWU_setTime(ms);
  
  // enter HALT mode
  lowPower_Halt();

} // lowPower_HaltAWU
   

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
