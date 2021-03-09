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


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_MAIN_'
#if defined(_MAIN_)
  volatile uint8_t            g_key;                    ///< byte received. Stored in Rx ISR
#else // _MAIN_
  extern volatile uint8_t     g_key;
#endif // _MAIN_


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

/// check if byte received
#define UART_available()   ( sfr_UART.SR.RXNE )

/// read received byte from UART
#define UART_read()        ( sfr_UART.DR.byte )

/// send byte via UART
#define UART_write(x)      { while (!(sfr_UART.SR.TXE)); sfr_UART.DR.byte = x; }

/// flush UART Tx
#define UART_flush()       { while (!(sfr_UART.SR.TC)); }
  

/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// initialize UART
void UART_begin(uint32_t BR);

/// ISR for UART receive
ISR_HANDLER(UART_RXNE_ISR, _UART_RXNE_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _UART2_H_
