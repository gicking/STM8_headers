/**********************
  ADC voltage measurement
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - measure voltage via ADC every ~500ms
    - print to UART
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "adc1.h"
  #include "uart2.h"
#undef _MAIN_


/// re-map a number from one range to another
#define map(x,inMin,inMax,outMin,outMax)    ((int32_t)(x-inMin)*(int32_t)(outMax-outMin)/(inMax-inMin)+outMin)



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

  uint32_t    i;
  uint16_t    value;
	
	
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init ADC1 for single-shot measurement
  ADC1_init();
  
  // select ADC pin A0 (=PB0/AIN0) on sduino
  ADC1_set_channel(0);
  
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // main loop
  while(1) {
  
    // measure ADC value
    value = ADC1_measure();

    // print value
    printf("voltage = %d mV\n", (int) map(value, 0,1023, 0,5000));

    // simple wait
    for (i=0; i<300000L; i++)
      NOP();
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
