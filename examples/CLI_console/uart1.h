/**
  \file uart1.h

  \author G. Icking-Konert
  \date 2020-05-24
  \version 0.1

  \brief declaration of UART1 functions/macros using FIFO and interrupts

  declaration of UART1 functions and macros using FIFO and interrupts
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _UART1_H_
#define _UART1_H_

#define FIFO_BUFFER_SIZE 128

#include <stdint.h>
#include "config.h"
#include "sw_fifo.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize UART1 for interrupt based communication
void  UART1_begin(uint32_t BR);

/// send byte via UART1
void  UART1_send_byte(uint8_t data);

/// send array of bytes via UART1
void  UART1_send_buf(uint16_t num, uint8_t *buf);

/// check if data was received via UART1
uint8_t UART1_check_Rx(void);

/// read data from UART1 receive FIFO
uint8_t UART1_receive(void);

/// peek next received byte from UART1 (keep data in FIFO)
uint8_t UART1_peek(void);

/// UART1 transmit ISR
ISR_HANDLER(UART1_TXE_ISR, _UART1_T_TXE_VECTOR_);

/// UART1 receive ISR
ISR_HANDLER(UART1_RXNE_ISR, _UART1_R_RXNE_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _UART1_H_
