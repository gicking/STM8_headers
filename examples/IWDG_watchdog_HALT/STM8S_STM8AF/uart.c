/**
  \file uart.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of UART functions/macros
   
  implementation of UART functions.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
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
  
  // for low-power device enable clock gating to USART1
  #if defined(FAMILY_STM8L)
    sfr_CLK.PCKENR1.PCKEN15 = 1;
  #endif
  
  // reset UART
  sfr_UART.CR1.byte = 0x00;
  sfr_UART.CR2.byte = 0x00;
  sfr_UART.CR3.byte = 0x00;

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
  sfr_UART.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  
  // enable transmission, no transmission interrupt
  sfr_UART.CR2.REN  = 1;  // enable receiver
  sfr_UART.CR2.TEN  = 1;  // enable sender
  //sfr_UART.CR2.TIEN = 1;  // enable transmit interrupt
  sfr_UART.CR2.RIEN = 1;  // enable receive interrupt
  
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
ISR_HANDLER(UART_RXNE_ISR, _UART_RXNE_VECTOR_)
{
  // clean UART2 receive flag
  sfr_UART.SR.RXNE = 0;

  // save received byte
  g_key = sfr_UART.DR.byte;
  
  return;

} // UART_RXNE_ISR


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
