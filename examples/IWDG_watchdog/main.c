/**********************
  Test independent watchdog IWDG
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - STM8L Discovery (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)
    - STM8S Discovery (https://www.st.com/en/evaluation-tools/stm8s-discovery.html)
    - NUCLEO-8S207K8 (https://www.st.com/en/evaluation-tools/nucleo-8s207k8.html)
    - NUCLEO-8S208RB (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)

  Functionality:
    - initialize IWDG to 100ms, service every 50ms
    - print millis to UART every 500ms
    - if 'r' received, stop watchdog service --> reset 
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart.h"
  #include "timer4.h"
  #include "iwdg.h"
#undef _MAIN_


/*----------------------------------------------------------
    LOCAL MACROS
----------------------------------------------------------*/
#define PERIOD_IWDG     100     // IWDG timeout period [ms]  
#define SCHEDULE_IWDG    50     // scheduler period [ms] for IWDG service  
#define SCHEDULE_PRINT  500     // scheduler period for printing time


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

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

  uint32_t  millis;
  uint32_t  lastIWDG = 0;
  uint32_t  lastPrint = 0;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART_begin(19200);
  
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
  
  // init 1ms interrupt
  TIM4_init();
  
  // start IWDG watchdog and set timeout period
  iwdg_init(PERIOD_IWDG);

  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // print instruction  
  printf("\npress 'r' to reset controller\n\n");
    
  // main loop
  while(1) {
  
    // avoid race condition
    DISABLE_INTERRUPTS();
    millis = g_millis;
    ENABLE_INTERRUPTS();
    
    // service IWDG every 50ms
    if (millis - lastIWDG > SCHEDULE_IWDG) {
      lastIWDG = millis;
      
      // service IWDG
      iwdg_service();
      
    } // 50ms task
    

    // print time every 500ms
    if (millis - lastPrint > SCHEDULE_PRINT) {
      lastPrint = millis;
      
      // toggle LED
      ODR_LED ^= 1;
    
      // print millis
      printf("  time: %ld\n", millis);
      
    } // 500ms task
    

    // received a byte via UART (stored in receive ISR)
    if (g_key) {
      
      // on 'r' stop servicing IWDG --> reset
      if (g_key == 'r') {
        iwdg_service();
        printf("stop IWDG service ...\n");
        while(1);
      }
      
      // clear key
      g_key = 0;

    } // byte received    
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
