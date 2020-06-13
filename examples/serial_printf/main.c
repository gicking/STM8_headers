/**********************
  Test serial communication. Write via printf
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - send millis every 500ms
    - echo received bytes
    - if 'r' received, reset controller
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

  // send byte
  UART2_write(data);
  
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
  
  // LED connected to PC5
  sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
  sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // init 1ms interrupt
  TIM4_init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // print instruction  
  printf("\npress 'r' to reset controller\n\n");
    
  // main loop
  while(1) {
  
    // print time every 500ms
    if (g_millis >= nextPrint) {
      nextPrint += 500;
      
      // toggle LED
      sfr_PORTC.ODR.ODR5 ^= 1;
    
      // print millis
      printf("  time: %ld\n", g_millis);
      
    } // 500ms loop
    

    // received a byte via UART (stored in receive ISR)
    if (g_key) {
      
      // on 'r' reset
      if (g_key == 'r') {
        printf("reset...\n");
        UART2_flush();
        SW_RESET();
      }

      // else echo byte
      printf("received '%c'\n", (char) g_key);

      g_key = 0;
    } // byte received
    
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
