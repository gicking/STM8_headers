/**
  \file clock.h

  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1

  \brief declaration of clock functions/macros

  declaration of clock switching functions
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _CLOCK_H_
#define _CLOCK_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>
#include "config.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL MACROS
-----------------------------------------------------------------------------*/

// STM8S / STM8AF family
#if defined(FAMILY_STM8S)
  #define CLK_HSI               0xE1                        ///< clock source: internal high-speed clock (16MHz)
  #define CLK_LSI               0xD2                        ///< clock source: internal low-speed clock
  #define CLK_HSE               0xB4                        ///< clock source: external high-speed clock

  #define clock_HSE_fail()      ( sfr_CLK.CSSR.AUX )        ///< CSS triggered -> HSE failed
  #define clock_get()           ( sfr_CLK.CMSR.byte )       ///< active clock source

// STM8L / STM8AL family
#elif defined(FAMILY_STM8L)
  #define CLK_HSI               0x01                        ///< clock source: internal high-speed clock (16MHz)
  #define CLK_LSI               0x02                        ///< clock source: internal low-speed clock
  #define CLK_HSE               0x04                        ///< clock source: external high-speed clock
  #define CLK_LSE               0x08                        ///< clock source: external low-speed clock

  #define clock_HSE_fail()      ( sfr_CLK.CSSR.AUX )        ///< CSS triggered -> HSE failed
  #define clock_get()           ( sfr_CLK.SCSR.byte )       ///< active clock source

// family not yet supported
#else
  #error unsupported STM8 family
#endif


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// switch clock source
uint8_t clock_switch(uint8_t clk_source);

/// enable clock security system (supervise HSE)
void clock_init_css(void);

/// ISR for clock security interrupt
ISR_HANDLER(CLK_CSS_ISR, _CLK_CSS_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CLOCK_H_
