/**********************
  Measure ADC via interrupt. Send averaged result periodically via printf.
  Note: this generates a high CPU load of ~30% due to ISR every 16µs!

  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)

  Functionality:
    - measure ADC continuously
    - cumulate result in ADC ISR -> high CPU load!
    - send average result every 1s
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "adc1.h"
  #include "timer4.h"
  #include "uart2.h"
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

  // configure output testpin PC5(=D13=LED)
  sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
  sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // init ADC for measuring AIN0(=A0) in continuous mode without interrupt
  ADC1_init(0);

  // init timer TIM4 for 1ms
  TIM4_init();

  // init UART2 for 19.2kBaud
  UART2_begin(19200);

  // enable interrupts
  ENABLE_INTERRUPTS();


  // main loop. Just toggle pin to measure CPU load
  while(1) {

    // print time every 500ms
    if (g_millis >= nextPrint) {
      nextPrint += 500;

      // toggle LED
      sfr_PORTC.ODR.ODR5 ^= 1;

      // print millis
      printf("  time: %ld  ADC: %d\n", g_millis, (int) g_ADC_result);

    } // 500ms loop

  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
