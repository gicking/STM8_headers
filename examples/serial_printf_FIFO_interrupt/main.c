/**********************
  Serial communication using FIFO and interrupts
  similar to Arduino Serial class
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
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
  #include "uart2.h"
  #include "timer4.h"
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
  UART2_send_byte(data);
  
  // return sent byte
  return(data);
  
} // putchar



/////////////////
//    main routine
/////////////////
void main (void) {

  uint32_t  nextPrint = 0;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // configure pin 13 (=PC5=LED) and pin 12 (=PC7) as output
  sfr_PORTC.DDR.byte = (uint8_t) ((1<<5) | (1<<7));     // input(=0) or output(=1)
  sfr_PORTC.CR1.byte = (uint8_t) ((1<<5) | (1<<7));     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.byte = (uint8_t) ((1<<5) | (1<<7));     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // init 1ms interrupt
  TIM4_init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // main loop
  while(1) {
  
    // toggle pin 12
    sfr_PORTC.ODR.ODR7 ^= 1;
    
    // if bytes were received, print them
    while (UART2_check_Rx())
      printf("received '%c'\n", (char) UART2_receive());

    // print time every 500ms
    if (g_millis >= nextPrint) {
      nextPrint += 500;
      
      // toggle pin 13 / LED
      sfr_PORTC.ODR.ODR5 ^= 1;
    
      // print millis
      printf("  time: %ld\n", g_millis);
      
    } // 500ms loop   
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
