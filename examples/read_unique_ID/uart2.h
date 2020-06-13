/**
  \file uart2.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of UART2 functions/macros
   
  declaration of UART2 functions.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _UART2_H_
#define _UART2_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart2.h"
#undef _MAIN_


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

/// read received byte from UART2
#define UART2_read()      (sfr_UART2.DR.byte)

/// send byte via UART2
#define UART2_write(x)	  { while (!(sfr_UART2.SR.TXE)); sfr_UART2.DR.byte = x; }

/// flush UART2
#define UART2_flush()	  { while (!(sfr_UART2.SR.TC)); }


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// initialize UART2
void UART2_begin(uint32_t BR);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _UART2_H_
