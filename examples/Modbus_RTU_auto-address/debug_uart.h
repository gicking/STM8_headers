/**
  \file debug_uart.h

  \author G. Icking-Konert
  \date 2020-05-24
  \version 0.1

  \brief declaration of debug UART functions/macros using FIFO and interrupts

  declaration of debug UART functions and macros using FIFO and interrupts
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _DEBUG_UART_H_
#define _DEBUG_UART_H_

#define FIFO_BUFFER_SIZE 128

#include <stdio.h>
#include <stdint.h>
#include "config.h"
#include "sw_fifo.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize debug UART for interrupt based communication
void  UART_debug_begin(uint32_t BR);

/// send byte via debug UART
void  UART_debug_send_byte(uint8_t data);

/// send array of bytes via debug UART
void  UART_debug_send_buf(uint16_t num, uint8_t *buf);

/// check if data was received via debug UART
uint8_t UART_debug_check_Rx(void);

/// read data from debug UART receive FIFO
uint8_t UART_debug_receive(void);

/// peek next received byte from debug UART (keep data in FIFO)
uint8_t UART_debug_peek(void);

/// debug UART transmit ISR
ISR_HANDLER(UART_DEBUG_TXE_ISR, _UART_DEBUG_T_TXE_VECTOR_);

/// debug UART receive ISR
ISR_HANDLER(UART_DEBUG_RXNE_ISR, _UART_DEBUG_R_RXNE_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _DEBUG_UART_H_
