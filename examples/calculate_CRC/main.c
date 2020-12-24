/**********************
  Various CRC calculations from https://github.com/basilhussain/stm8-crc
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - periodically calculate CRC over pre-defined data
    - measure time for CRC calculation
    - print time and calculated CRC to UART
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart2.h"
  #include "timer4.h"
#undef _MAIN_
#include "crc.h"


// data to calculate CRC over
static const uint8_t data[] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39 };

// select CRC method and data width (see folder crc)
#define CRC_TYPE         uint32_t
#define CRC_INIT()       crc32_init()
#define CRC_UPDATE(c,d)  crc32_update(c,d)
#define CRC_FINAL(c)     crc32_final(c)



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

  CRC_TYPE    crc;
  uint32_t    timeStart, timeStop;
  uint32_t    i;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART2_begin(19200);
  
  // init 1ms interrupt
  TIM4_init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // print instruction  
  printf("\nt[us]  CRC\n");
    
  // main loop
  while(1) {
  
    // get start time
    timeStart = micros();

    // initialize CRC
    crc = CRC_INIT();

    // calculate CRC over data array
    #if (1)
      for(i = 0; i < (sizeof(data) / sizeof(data[0])); i++)
        crc = CRC_UPDATE(crc, data[i]);

    // calculate CRC over 32kB flash
    #else
      for(i = 0x8000; i <= 0xFFFF; i++)
        crc = CRC_UPDATE(crc, *((uint8_t*) i));
    #endif

    // finalize CRC
    crc = CRC_FINAL(crc);

    // get end time
    timeStop = micros();
    
    // print duration and CRC result
    printf("%ldus  0x%8lX\n\n", (long)(timeStop-timeStart), crc, crc<<16);
    
    // wait a bit
    delay(500);
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
