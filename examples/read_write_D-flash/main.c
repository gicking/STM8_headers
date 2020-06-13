/**********************
  Implementation of EEPROM read/write routines
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - save data to EEPROM
    - read from EEPROM and print to terminal 
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart2.h"
  #include "eeprom.h"
#undef _MAIN_

// number of bytes to save/read
#define NUM_DATA  10


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

  int i, count;
  
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // save data  
  printf("\nsave data to EEPROM ... ");
  count = 0;
  for (i=0; i<NUM_DATA; i++)
    count += EEPROM_writeByte(i, i+1);
  printf("wrote %dB\n", count);

  // read data  
  printf("\nread data from EEPROM\n");
  for (i=0; i<NUM_DATA; i++)
    printf("  %3d  %d\n", i, (int) (EEPROM_readByte(i)));
  printf("done\n");
  
  // dummy main loop
  while(1);
  
} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
