/**
  \file clock.c
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief implementation of clock functions/macros
   
  implementation of clock switching functions
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "clock.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn uint8_t clock_switch(uint8_t clk_source)
   
  \brief switch to another clock source
  
  \param[in] clk_source  new clock source
  
  \return active clock source after (attempted) switch 
   
  Switch to another clock source (with timout). If switch fails, the clock setting
  is not changed  
*/
uint8_t clock_switch(uint8_t clk_source)
{
  volatile uint16_t  timeout;
  
  // clear pending ISR flag
  sfr_CLK.SWCR.SWIF = 0;

  // select new clock source
  sfr_CLK.SWR.SWI = clk_source;

  // wait for external clock stable with ~82ms timeout @ 16MHz
  timeout = 0xFFFF;
  while ((!sfr_CLK.SWCR.SWIF) && (--timeout));
  
  // check if new clock stable
  if (sfr_CLK.SWCR.SWIF)
  {
    // clear clk ISR flag
    sfr_CLK.SWCR.SWIF = 0;
    
    // execute clock switch
    sfr_CLK.SWCR.SWEN = 1;
  }

  // return active clock source
  return clock_get();
  
} // clock_switch

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
