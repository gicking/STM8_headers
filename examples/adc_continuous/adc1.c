/**
  \file adc1.c

  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1

  \brief implementation of ADC1 functions/macros

  implementation of ADC1 functions.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "adc1.h"



/**
  \fn void ADC1_init(uint8_t channel)

  \brief initialize ADC

  \param[in]  channel   AINx channel to measure

  initialize ADC for continuous mode with interrupt.
  For accuracy configure for slowest conversion speed (16µs)
*/
void ADC1_init(uint8_t channel)
{
  // stop ADC during configuration
  ADC1_power_down();

  // init global variables
  g_ADC_result = 0;

  // set ADC clock to 1/18*fMaster for optimum accuracy
  // Conversion takes 14 cycl -> 16µs
  sfr_ADC1.CR1.SPSEL = 7;

  // enable continuous conversion mode
  sfr_ADC1.CR1.CONT = 1;

  // right alignment (read DRL, then DRH), no external trigger
  sfr_ADC1.CR2.ALIGN = 1;

  // select measurement channel
  ADC1_set_channel(channel);

  // don't enable ADC EOC interrupt (extreme ISR load)
  //sfr_ADC1.CSR.EOC   = 0;
  //sfr_ADC1.CSR.EOCIE = 1;

  // start ADC. Required 2x
  ADC1_power_on();
  ADC1_power_on();

} // ADC1_init

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
