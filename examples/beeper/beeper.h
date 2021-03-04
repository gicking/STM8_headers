/**
  \file beeper.h

  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1

  \brief declaration of BEEPER functions/macros

  declaration of BEEPER functions.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _BEEPER_H_
#define _BEEPER_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

/// configure beeper clock divider
#define beeper_divider(div)	{ sfr_BEEP.CSR.BEEPDIV = div; }

/// select beeper frequency
#define beeper_freq(sel)	{ sfr_BEEP.CSR.BEEPSEL = sel; }

/// enable beeper output
#define beeper_enable()		{ sfr_BEEP.CSR.BEEPEN  = 1; }

/// disable beeper output
#define beeper_disable()	{ sfr_BEEP.CSR.BEEPEN  = 0; }


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _BEEPER_H_
