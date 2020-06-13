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
  \fn void ADC1_init(void)
   
  \brief initialize ADC
   
  initialize ADC for single-shot mode.
  For accuracy configure for slowest conversion speed (16µs) 
*/
void ADC1_init(void) {
  
  // set ADC clock to 1/18*fMaster to minimize influence between channels 
  // Conversion takes 14 cycl -> 16µs
  sfr_ADC1.CR1.SPSEL = 7;
  
  // right alignment (read DRL, then DRH), no external trigger
  sfr_ADC1.CR2.ALIGN = 1;
  
  // disable all Schmitt trigger, which is recommended for EMC noisy conditions (skip for simplicity)
  //sfr_ADC1.TDRH.byte = 0xFF;
  //sfr_ADC1.TDRL.byte = 0xFF;
  
} // ADC1_init



/**
  \fn uint16_t ADC_measure(void)
   
  \brief measure selected ADC channel (single shot)
   
  \return           ADC value in INC
  
  measure selected ADC channel in single shot mode.
*/
uint16_t ADC1_measure(void) {
 
  uint16_t  result;

  // clear HW & SW conversion ready flags, start conversion, wait until EOC-ISR finished
  sfr_ADC1.CSR.EOC = 0;       // reset HW EOC flag, set by ADC
  sfr_ADC1.CR1.ADON=1;        // start ADC conversion. Need to do this 2x (empirically)
  sfr_ADC1.CR1.ADON=1;
  while(!sfr_ADC1.CSR.EOC);   // wait for HW "conversion ready" flag

  // get ADC result (read low byte first for right alignment!)
  result  =  (uint16_t) sfr_ADC1.DRL.byte;
  result += ((uint16_t) sfr_ADC1.DRH.byte) << 8;
  
  // return result
  return(result);

} // ADC1_measure


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
