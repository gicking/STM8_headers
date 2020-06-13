/**********************
  Implementation of Option bytes read/write routines
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - activate/deactivate ROM-bootloader, depending on pin 8
   
  Notes: 
    - an accidentally deactivated ROM bootloader can be re-activated via SWIM
    - most option bytes require the bitwise inverse at address+1. A mismatch results 
      in endless reset cycles, which can only be fixed by SWIM
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart2.h"
  #include "option_bytes.h"
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

  uint8_t  err;
  
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // configure LED pin 13 (=PC5) as output, and pin 8 (=PC1) as input_pull-up
  sfr_PORTC.DDR.byte = (1<<5) | (0<<1);    // input(=0) or output(=1)
  sfr_PORTC.CR1.byte = (1<<5) | (1<<1);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.byte = (1<<5) | (0<<1);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  sfr_PORTC.ODR.ODR5 = 0;                  // LED off

  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // if pin 8 (=PC1) is high, activate ROM-bootloader (write 0x55AA to 0x487E)
  if (sfr_PORTC.IDR.IDR1) {
    err  = write_option_byte((uint16_t)(&(sfr_OPT.OPTBL)),  0x55);
    err += write_option_byte((uint16_t)(&(sfr_OPT.NOPTBL)), 0xAA);
    printf("\nactivate bootloader: ");
    sfr_PORTC.ODR.ODR5 = 1;
  }

  // if pin 8 (=PC1) is low, deactivate ROM-bootloader
  else {
    err  = write_option_byte((uint16_t)(&(sfr_OPT.OPTBL)),  0x00);
    err += write_option_byte((uint16_t)(&(sfr_OPT.NOPTBL)), 0xFF);
    printf("\ndeactivate bootloader: ");
    sfr_PORTC.ODR.ODR5 = 0;
  }
  
  // check result of option byte operation
  if (err == 0)
    printf(" no change\n");
  else if (err == 2)
    printf(" ok\n");
  else
    printf(" error\n");

  // dummy main loop
  while(1);
  
} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
