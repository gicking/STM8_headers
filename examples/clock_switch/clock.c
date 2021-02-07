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



/**
  \fn void clock_init_css(void)

  \brief enable clock security system (supervise HSE)

  Enable clock security system for HSE. In case of HSE fails, the CSS
  falls back to 16MHz HSI and triggers an interrupt.
  Once CSS triggers, HSE remains disabled until reset (see AN3265)
*/
void clock_init_css(void)
{
  // for low-power device enable smooth fall-back to HSI
  #if defined(FAMILY_STM8L)
    sfr_CLK.CSSR.CSSDGON = 1;
  #endif

  // enable CSS interrupt
  sfr_CLK.CSSR.CSSDIE = 1;

  // clear CSS interrupt flag
  sfr_CLK.CSSR.CSSD = 0;

  // activate CSS
  sfr_CLK.CSSR.CSSEN = 1;

} // clock_init_css



/**
  \fn void CLK_CSS_ISR(void)

  \brief ISR for clock fail

  interrupt service routine for clock security system.
  Used to reset system to safe state in case HSE fails.
  Once CSS triggers, HSE remains disabled until reset (see AN3265)

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(CLK_CSS_ISR, _CLK_CSS_VECTOR_)
{
  // clear CSS interrupt flag
  sfr_CLK.CSSR.CSSD = 0;

  // optionally trigger reset (faster than IWDG timeout)
  //SW_RESET();

  // switch HSI prescaler to 1 (default is 8) -> 16MHz
  // note: not recommended for HSE!=16MHz
  sfr_CLK.CKDIVR.byte = 0x00;

  return;

} // CLK_CSS_ISR

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
