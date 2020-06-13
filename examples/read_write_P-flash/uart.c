/**
  \file uart.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of UART functions/macros
   
  implementation of UART functions.
    
  Note: board selection in Makefile / project options
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"
#include "uart.h"



/**
  \fn void UART_begin(uint32_t BR)
   
  \brief initialize UART for blocking transmission, polling reception
  
  \param[in]  BR    baudrate [Baud]

  initialize UART for communication with specified baudrate.
  Use 1 start, 8 data and 1 stop bit; no parity or flow control.
  Use blocking Tx, and polling Rx.
*/
void UART_begin(uint32_t BR) {

  uint16_t  val16;
  
  // set UART behaviour
  sfr_UART.CR1.byte = 0x00;       // enable UART, 8 data bits, no parity control
  sfr_UART.CR2.byte = 0x00;       // no interrupts, disable sender/receiver 
  sfr_UART.CR3.byte = 0x00;       // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
  sfr_UART.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  
  // enable transmission, no transmission
  sfr_UART.CR2.REN  = 1;  // enable receiver
  sfr_UART.CR2.TEN  = 1;  // enable sender
  //sfr_UART.CR2.TIEN = 1;  // enable transmit interrupt
  //sfr_UART.CR2.RIEN = 1;  // enable receive interrupt

} // UART_begin


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
