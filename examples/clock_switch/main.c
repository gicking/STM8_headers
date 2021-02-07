/**********************
  Test serial communication. Write via printf

  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - STM8L Discovery board (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)

  Functionality:
    - periodically:
      - switch between internal and external clock with timeout
      - print active clock via UART
    - for extra safety use independent watchdog (IWDG)
    notes:
      - STM8L Discovery has no external resonator -> only used to demonstrate timeout
      - if HSE failed (supervised by CSS), HSI remains active until reset
      - in case of HSE>16MHz or safety relevant applications, revert to safe state and/or reset in CLK_CSS_ISR()
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "clock.h"
  #include "iwdg.h"
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
  char putchar(char data)
#else // Standard C
  int putchar(int data)
#endif
{
  // send byte
  UART_write(data);

  // return sent byte
  return(data);

} // putchar



/////////////////
//    main routine
/////////////////
void main (void)
{
  uint32_t  nextIWDG = 0;
  uint32_t  nextPrint = 0;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch fMaster prescaler to 1 (default is 8)
  sfr_CLK.CKDIVR.byte = 0x00;

  // configure LED pin as output
  LED_PORT.DDR.byte |= LED_PIN;     // input(=0) or output(=1)
  LED_PORT.CR1.byte |= LED_PIN;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  LED_PORT.CR2.byte |= LED_PIN;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // init UART for 19.2kBaud
  UART_begin(19200);

  // init 1ms interrupt
  TIM4_init();

  // switch to external clock
  clock_switch(CLK_HSE);

  // enable HSE supervision (CSS)
  // Note: if CSS triggers, HSE remains disable until reset
  clock_init_css();

  // init IWDG watchdog with 100ms period and start (cannot be stopped)
  iwdg_init(100);

  // enable interrupts
  ENABLE_INTERRUPTS();

  // print message
  printf("reset\n");

  // main loop
  while(1) {

    // service IWDG every 50ms
    if (g_millis >= nextIWDG) {
      nextIWDG += 50;

      // service IWDG
      iwdg_service();

    } // 50ms task


    // print time every 500ms
    if (g_millis >= nextPrint) {
      nextPrint += 500;

      // alternate betwwen clock sources
      if (clock_get() == CLK_HSI)
      {
        LED_PORT.ODR.byte &= ~LED_PIN; // LED=off -> HSI
        if (!clock_HSE_fail())
          printf("clock HSI\n");         // print active clock
        else
          printf("HSE fail -> HSI\n");
      	iwdg_service();                // service IWDG to avoid due to HSE startup
        clock_switch(CLK_HSE);         // switch to other clock
        iwdg_service();                // service IWDG to avoid due to HSE startup
      }
      else if (clock_get() == CLK_HSE)
      {
        LED_PORT.ODR.byte |= LED_PIN;  // LED=on -> HSE
        printf("clock HSE\n");         // print active clock
      	clock_switch(CLK_HSI);         // switch to other clock
      }
      else
      	printf("  unknown clock source 0x%02x\n", clock_get());

    } // 500ms loop

  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
