/**
  \file awu.c
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief implementation of auto-wake functions/macros
   
  implementation of auto-wake functions/macros.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "awu.h"


/**
  \fn void AWU_ISR(void)
   
  \brief ISR for AWU
   
  interrupt service routine for auto-wakeup.
  Reset AWU flag (is mandatory!)

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(AWU_ISR, _AWU_VECTOR_) {

  // toggle red LED to indicate wake event
  sfr_PORTH.ODR.ODR3 ^= 1;
  
  // reset wakeup flag
  sfr_AWU.CSR1.AWUF = 0;
  
} // AWU_ISR



/**
  \fn void AWU_setTime(uint16_t ms)
   
  \brief configure auto-wake
  
  \param[in]  ms    sleep duration [ms] within [1;30000]
  
  configure auto-wake after specified number of milliseconds.
  Do not enter HALT mode here.

  \note the LSI low speed clock is quite inaccurate, and the granularity
        of the AWU is coarse -> don't use for time-critical tasks!
*/
void AWU_setTime(uint16_t ms) {

  uint8_t    APR, AWUTB;

  // skip optional measurement of LSI with fCPU

  // clip to valid AWU range
  if (ms < 1)     ms = 1;
  if (ms > 30000) ms = 30000;

  // calculate AWU parameters
  if (ms>=5120) {
    AWUTB = 15;
    APR   = (uint8_t) ((ms >> 4)/30L);
  }
  else if (ms>=2048) {
    AWUTB = 14;
    APR   = (uint8_t) ((ms >> 4)/5L);
  }
  else {
    AWUTB = 0;    // calculate log2(ms)
    while ((1L<<AWUTB) < ms)
      AWUTB++;
    AWUTB += 2;
    if (AWUTB < 3)  AWUTB = 3;  // clip value
    if (AWUTB > 13) AWUTB = 13;
    APR   = (uint8_t) ((((uint32_t) ms) << 8) >> AWUTB);
  }
  
  // clip ARP
  if (APR < 0x3E) APR = 0x3E;

  // set (N+2) prescaler for 128kHz LSI clock
  sfr_AWU.APR.APR   = APR;
  
  // set AWU counter 2^0..2^12, 5*2^11, 30*2^11
  sfr_AWU.TBR.AWUTB = AWUTB;
  
  // enable wake and enable AWU interrupt
  sfr_AWU.CSR1.AWUEN = 1;
  
} // AWU_setTime


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
