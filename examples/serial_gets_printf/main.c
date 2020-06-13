/**********************
  Test serial communication. Read via gets, write via printf
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - read string from UART, convert to number and send back
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart2.h"
#undef _MAIN_



/**
  \fn char getchar(void) 
   
  \brief receive byte
  
  \return  character received or zero on error.

  implementation of getchar() for gets(), using selected input channel.
  Return type depends on used compiler (see respective stdio.h)
*/
#if defined(__CSMC__)
  char getchar(void) {
#else // Standard C
  int getchar(void) {
#endif

  char c;

  // wait until byte received
  while (!UART2_available());

  // read Rx buffer
  c = UART2_read();

  // Cosmic and IAR require some special treatment
  #if defined(__CSMC__) || defined(__ICCSTM8__)
    
    // echo to console
    putchar(c);
    
    // Cosmic gets() terminates on NL
    if (c == 13)
      c = 10;
      
  #endif // __CSMC__ || __ICCSTM8__
  
  // return received byte
  return c;
  
} // getchar



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

  char    str[20];
  int     num;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // main loop
  while(1) {
  
    // read a string via UART2
    printf("Enter a number: ");
    gets(str);
    //for (num=0; num<strlen(str); num++)  printf("%d  %d\n", num, str[num]);
  
    // convert to integer [-2^16; 2^16-1]
    num = atoi(str);
  
    // print result via UART2
    printf("value: %d\n\n", num);
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
