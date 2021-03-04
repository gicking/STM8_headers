/**********************
  Serial communication using FIFO and interrupts
  similar to Arduino Serial class

  supported hardware:
    - Nucleo-8S208RB (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)

  Functionality:
    - send millis every 500ms
    - use FIFO and interrupts for transmit & receive
    - functionality is like Arduino Serial class
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart1.h"
  #include "cli.h"
#undef _MAIN_



/**
  \fn int putchar(int byte)

  \brief output routine for printf()

  \param[in]  data   byte to send

  \return  sent byte

  implementation of putchar() for printf(), using selected output channel.
  Use send routine set via putchar_attach()
  Return type depends on used compiler (see respective stdio.h)
*/
#if defined(__CSMC__)
  char putchar(char data) {
#else // Standard C
  int putchar(int data) {
#endif

  // queue byte in Tx FIFO
  UART1_send_byte(data);

  // return sent byte
  return(data);

} // putchar



/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;

  // init UART1 for 19.2kBaud
  UART1_begin(19200);

  // configure LED pin (=PC5) as output
  sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
  sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  sfr_PORTC.ODR.ODR5 = 0;     // LED off

  // enable interrupts
  ENABLE_INTERRUPTS();

  // print console greeting message
  cli_greeting();

  // main loop
  while(1) {

    // handle commandline interface
    cli_handler();

  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
