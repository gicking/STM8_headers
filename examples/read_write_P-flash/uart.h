/**
  \file uart.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of UART functions/macros
   
  declaration of UART functions.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _UART_H_
#define _UART_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

/// read received byte from UART
#define UART_read()      (sfr_UART.DR.byte)

/// send byte via UART
#define UART_write(x)	  { while (!(sfr_UART.SR.TXE)); sfr_UART.DR.byte = x; }

/// flush UART
#define UART_flush()	  { while (!(sfr_UART.SR.TC)); }


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// initialize UART
void UART_begin(uint32_t BR);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _UART_H_
