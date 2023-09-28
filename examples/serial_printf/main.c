/**********************
  Test serial communication. Write via printf
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - STM8L Discovery (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)
    - STM8S Discovery (https://www.st.com/en/evaluation-tools/stm8s-discovery.html)
    - NUCLEO-8S207K8 (https://www.st.com/en/evaluation-tools/nucleo-8s207k8.html)
    - NUCLEO-8S207K8 (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)

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
  #include "timer4.h"
  #include "uart.h"
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
  UART_write(data);
  
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
    
  // configure LED pin as output
  #if defined(SDUINO)
    sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
    sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    #define ODR_LED  sfr_PORTC.ODR.ODR5
  #elif defined(STM8L_DISCOVERY)
    sfr_PORTE.DDR.DDR7 = 1;     // input(=0) or output(=1)
    sfr_PORTE.CR1.C17  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTE.CR2.C27  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    #define ODR_LED  sfr_PORTE.ODR.ODR7
  #elif defined(STM8S_DISCOVERY)
    sfr_PORTD.DDR.DDR0 = 1;     // input(=0) or output(=1)
    sfr_PORTD.CR1.C10  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTD.CR2.C20  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    #define ODR_LED  sfr_PORTD.ODR.ODR0
  #elif defined(NUCLEO_8S207K8)
    sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
    sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    #define ODR_LED  sfr_PORTC.ODR.ODR5
  #elif defined(NUCLEO_8S208RB)
    sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
    sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    #define ODR_LED  sfr_PORTC.ODR.ODR5
  #else
    #error undefined board
  #endif
  
  // init UART for 19.2kBaud
  UART_begin(19200);
  
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
      ODR_LED ^= 1;
    
      // print millis
      printf("  time: %ld\n", g_millis);
      
    } // 500ms loop
    

    // received a byte via UART (stored in receive ISR)
    if (g_key) {
      
      // on 'r' reset
      if (g_key == 'r') {
        printf("reset...\n");
        UART_flush();
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
