/**
  \file timer4.c

  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1

  \brief implementation of timer TIM4 (1ms clock) functions/macros

  implementation of timer TIM4 functions as 1ms master clock.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "timer4.h"
#include "adc1.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void TIM4_init(void)

  \brief init timer 4 for 1ms master clock with interrupt

  init 8-bit timer TIM4 with 1ms tick. Is used for SW master clock
  via below interrupt.
*/
void TIM4_init(void) {

  // stop the timer
  sfr_TIM4.CR1.CEN = 0;

  // initialize global clock variables
  g_flagMilli = 0;
  g_millis    = 0;
  g_micros    = 0;

  // clear counter
  sfr_TIM4.CNTR.byte = 0x00;

  // auto-reload value buffered
  sfr_TIM4.CR1.ARPE = 1;

  // clear pending events
  sfr_TIM4.EGR.byte  = 0x00;

  // set clock to 16Mhz/2^6 = 250kHz -> 4us period
  sfr_TIM4.PSCR.PSC = 6;

  // set autoreload value for 1ms (=250*4us)
  sfr_TIM4.ARR.byte  = 250;

  // enable timer 4 interrupt
  sfr_TIM4.IER.UIE = 1;

  // start the timer
  sfr_TIM4.CR1.CEN = 1;

} // TIM4_init



/**
  \fn void delay(uint32_t ms)

  \brief delay code execution for 'ms'

  \param[in]  ms   duration[ms] to halt code

  delay code execution for 'ms'.
  Requires TIM4 interrupt -> is not vulnerable to
  interrupts (within limits).
  Note:
    - for ISR-free functions use sw_delay() (uses NOPs)
    - for high accuracy use highRez_delay() (uses HW timer 3)
*/
void delay(uint32_t ms) {

  uint32_t start = micros();

  // wait until time [us] has passed
  ms *= 1000L;
  while (micros() - start < ms)
    NOP();

} // delay()



/**
  \fn void delayMicroseconds(uint32_t us)

  \brief delay code execution for 'us'

  \param[in]  us   duration[us] to halt code

  delay code execution for 'us'.
  Requires TIM4 interrupt -> is not vulnerable to
  interrupts (within limits).
  Note:
    - for ISR-free functions use sw_delayMicroseconds() (uses NOPs)
    - for high accuracy use highRez_delayMicroseconds() (uses HW timer 3)
*/
void delayMicroseconds(uint32_t us) {

  uint32_t start = micros();

  // wait until time [us] has passed
  while (micros() - start < us)
    NOP();

} // delayMicroseconds()



/**
  \fn void TIM4_UPD_ISR(void)

  \brief ISR for timer 4 (1ms master clock)

  interrupt service routine for timer TIM4.
  Used for 1ms system clock.

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_)
{
  // clear timer 4 interrupt flag
  sfr_TIM4.SR.UIF = 0;

  // set/increase global variables
  g_micros += 1000L;
  g_millis++;
  g_flagMilli = 1;

  // read ADC result
  g_ADC_result =  (uint16_t) sfr_ADC1.DRL.byte;
  g_ADC_result += ((uint16_t) sfr_ADC1.DRH.byte) << 8;

  return;

} // TIM4_UPD_ISR

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
