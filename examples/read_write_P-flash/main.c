/**********************
  Implementation of EEPROM read/write routines
  
  supported hardware:
    - Sduino Uno 32kB (https://github.com/roybaer/sduino_uno)
    - muBoard 128kB (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
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
  #include "uart.h"
  #include "flash.h"
  #include "memory_access.h"
#undef _MAIN_


// starting address and number of bytes to save/read
#define NUM_DATA     20
#define START_ADDR   (FLASH_ADDR_END - NUM_DATA + 1)



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

  uint32_t    i;
  int         count;
  
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART for 19.2kBaud
  UART_begin(19200);
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
	// save data  
  printf("\nsave data to P-flash ... ");
  count = 0;
  for (i=0; i<NUM_DATA; i++)
    count += FLASH_writeByte(START_ADDR+i, i+1);
  printf("wrote %dB\n", count);

  // read data  
  printf("\nread data from P-flash\n");
  for (i=0; i<NUM_DATA; i++)
    printf("  0x%lx  %d\n", (START_ADDR+i), (int) (read_1B(START_ADDR+i)));
  printf("done\n");
  
  // dummy main loop
  while(1);
  
} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
