/**
  \file uart2.h
   
  \author G. Icking-Konert
  \date 2020-05-24
  \version 0.1
   
  \brief declaration of UART2 functions/macros using FIFO and interrupts 
   
  declaration of UART2 functions and macros using FIFO and interrupts 
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _UART2_H_
#define _UART2_H_

#define FIFO_BUFFER_SIZE 128

#include <stdint.h>
#include "config.h"
#include "sw_fifo.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize UART2 for interrupt based communication 
void  UART2_begin(uint32_t BR);

/// send byte via UART2
void  UART2_send_byte(uint8_t data);

/// send array of bytes via UART2
void  UART2_send_buf(uint16_t num, uint8_t *buf);

/// check if data was received via UART2 
uint8_t UART2_check_Rx(void);

/// read data from UART2 receive FIFO
uint8_t UART2_receive(void);

/// peek next received byte from UART2 (keep data in FIFO)
uint8_t UART2_peek(void);

/// UART2 transmit ISR
ISR_HANDLER(UART2_TXE_ISR, _UART2_T_TXE_VECTOR_);
  
/// UART2 receive ISR
ISR_HANDLER(UART2_RXNE_ISR, _UART2_R_RXNE_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _UART2_H_
