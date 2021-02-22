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

// STM8L
#if defined(sfr_USART1)

  /// check if byte received via USART1
  #define UART_available()   ( sfr_USART1.SR.RXNE )

  /// read received byte from USART1
  #define UART_read()        ( sfr_USART1.DR.byte )

  /// send byte via UART2
  #define UART_write(x)	     { while (!(sfr_USART1.SR.TXE)); sfr_USART1.DR.byte = x; }

  /// flush UART2
  #define UART_flush()	     { while (!(sfr_USART1.SR.TC)); }
  
// STM8S
#elif defined(sfr_UART2)

  /// check if byte received via UART2
  #define UART_available()   ( sfr_UART2.SR.RXNE )

  /// read received byte from UART2
  #define UART_read()        ( sfr_UART2.DR.byte )

  /// send byte via UART2
  #define UART_write(x)	     { while (!(sfr_UART2.SR.TXE)); sfr_UART2.DR.byte = x; }

  /// flush UART2
  #define UART_flush()	     { while (!(sfr_UART2.SR.TC)); }

// error 
#else
  #error UART not defined
#endif



/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// initialize UART
void UART_begin(uint32_t BR);

/// ISR for UART receive
#if defined(_UART2_R_RXNE_VECTOR_)
  ISR_HANDLER(UART_RXNE_ISR, _UART2_R_RXNE_VECTOR_);
#elif defined(_USART_R_RXNE_VECTOR_)
  ISR_HANDLER(UART_RXNE_ISR, _USART_R_RXNE_VECTOR_);
#else
  #error UART_RXNE vector undefined
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _UART2_H_
