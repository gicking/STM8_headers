/**
  \file uart2.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of UART2 functions/macros
   
  implementation of UART2 functions.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "uart2.h"



/**
  \fn void UART2_begin(uint32_t BR)
   
  \brief initialize UART2 for blocking transmission, background reception
  
  \param[in]  BR    baudrate [Baud]

  initialize UART2 for communication with specified baudrate.
  Use 1 start, 8 data and 1 stop bit; no parity or flow control.
  Use blocking Tx, and Rx interrupt.
*/
void UART2_begin(uint32_t BR) {

  uint16_t  val16;
  
  // set UART2 behaviour
  sfr_UART2.CR1.byte = sfr_UART2_CR1_RESET_VALUE;  // enable UART2, 8 data bits, no parity control
  sfr_UART2.CR2.byte = sfr_UART2_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver 
  sfr_UART2.CR3.byte = sfr_UART2_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
  sfr_UART2.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART2.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  
  // enable transmission, no transmission
  sfr_UART2.CR2.REN  = 1;  // enable receiver
  sfr_UART2.CR2.TEN  = 1;  // enable sender
  //sfr_UART2.CR2.TIEN = 1;  // enable transmit interrupt
  sfr_UART2.CR2.RIEN = 1;  // enable receive interrupt

} // UART2_begin



/**
  \fn void UART2_RXNE_ISR(void)
   
  \brief ISR for UART2 receive
   
  interrupt service routine for UART2 receive.
  Store key in global variable

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART2_RXNE_ISR, _UART2_R_RXNE_VECTOR_)
{
  // save received byte
  g_key = sfr_UART2.DR.byte;
  
  // clean UART2 receive flag
  sfr_UART2.SR.RXNE = 0;
  
  return;

} // UART2_RXNE_ISR


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
