/**********************
  Read unique identifier and print via UART
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - read unique identifier and print to serial
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart2.h"
#undef _MAIN_

// check if device supports UID
#if !defined(UID_ADDR_START)
  #error UID is not supported by this device
#endif



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

  int  i;
  
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // print UID  
  printf("\nunique identifier:\n");
  for (i=0; i<UID_SIZE; i++)
    printf("  byte %2d = %3d (0x%02x)\n", i, (int) UID(i), (int) UID(i));
    
  // dummy main loop
  while(1);

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
