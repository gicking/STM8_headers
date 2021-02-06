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
  \fn void ADC1_start(void)

  \brief start ADC1 measurement of AIN0..AINx in scan mode

  start single scan measurements of AIN0..AINx.
  For accuracy configure for slowest conversion speed (16µs)

  Note: registers mut be re-initialize for each scan, see
        Mojzesz in https://community.st.com/s/question/0D50X00009XkatO/adc-1-problem-with-stm8sdiscovery
*/
void ADC1_start(void)
{
  // stop ADC during configuration
  ADC1_power_down();

  // set ADC clock to 1/18*fMaster for optimum accuracy
  // Conversion takes 14 cycl -> 16µs
  sfr_ADC1.CR1.SPSEL = 7;

  // right alignment (read DRL, then DRH), no external trigger
  sfr_ADC1.CR2.ALIGN = 1;

  // use single-shot conversion mode
  sfr_ADC1.CR1.CONT = 0;

  // enable scan mode (ADC1 only)
  sfr_ADC1.CR2.SCAN = 1;

  // store results in N buffers (ADC1 only)
  sfr_ADC1.CR3.DBUF = 1;

  // set scan mode channel -> measure AIN0..AINx
  sfr_ADC1.CSR.CH = ADC1_CHANNELS;

  // don't enable ADC EOC interrupt
  //sfr_ADC1.CSR.EOC   = 0;
  //sfr_ADC1.CSR.EOCIE = 1;

  // start ADC. Required 2x
  ADC1_power_on();
  ADC1_power_on();

} // ADC1_init

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
