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
  
  // STM8L
  #if defined(sfr_USART1)
  
    // for low-power device enable clock gating to USART1
    sfr_CLK.PCKENR1.PCKEN15 = 1;
    
    // set UART behaviour
    sfr_USART1.CR1.byte = sfr_USART1_CR1_RESET_VALUE;  // enable UART2, 8 data bits, no parity control
    sfr_USART1.CR2.byte = sfr_USART1_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver 
    sfr_USART1.CR3.byte = sfr_USART1_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

    // set baudrate (note: BRR2 must be written before BRR1!)
    val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
    sfr_USART1.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
    sfr_USART1.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  
    // enable transmission, no transmission
    sfr_USART1.CR2.REN  = 1;  // enable receiver
    sfr_USART1.CR2.TEN  = 1;  // enable sender
    //sfr_USART1.CR2.TIEN = 1;  // enable transmit interrupt
    //sfr_USART1.CR2.RIEN = 1;  // enable receive interrupt
  
  // STM8S
  #elif defined(sfr_UART2)
    
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
    //sfr_UART2.CR2.RIEN = 1;  // enable receive interrupt

  // error 
  #else
    #error UART not defined
  #endif

} // UART2_begin


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
