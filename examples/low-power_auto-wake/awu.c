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

  #define AWU_NUM  9
  const uint16_t  timeMax[AWU_NUM]  = {    32,   64,  128,  256,  512, 1024, 2048, 5120, 30720 };  // max. wake period [ms]
  const uint16_t  scalFreq[AWU_NUM] = { 16384, 8192, 4096, 2048, 1024,  512,  256,  102,    17 };  // AWU frequency [Hz*8.192]
  uint8_t         TBR, APR;

  // find smallest TBR for best accuracy 
  for (TBR = 0; TBR < AWU_NUM-1; TBR++)
    if (ms <= timeMax[TBR])
      break;

  // find corresponding APR counter value. Use scaled frequency and bit shift operation 
  APR = (uint8_t) ((((uint32_t) ms * (uint32_t) scalFreq[TBR]) >> 13)) - 2;

  // clip APR to valid range
  if (APR > 0x3E) APR = 0x3E;

  // add TBR offset (above windows ignore lowest 6 TBR settings with dt<1ms)
  TBR += 7; 


  // set AWU timebase selection register
  sfr_AWU.TBR.AWUTB = TBR;
  
  // set AWU asynchronous prescaler register
  sfr_AWU.APR.APR   = APR;
  
  // enable wake and AWU interrupt
  sfr_AWU.CSR1.AWUEN = 1;

} // AWU_setTime


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
