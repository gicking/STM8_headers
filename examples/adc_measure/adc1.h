/**
  \file adc1.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of ADC1 functions/macros
   
  declaration of ADC1 functions.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _ADC1_H_
#define _ADC1_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

/// set ADC channel to AINx
#define ADC1_set_channel(ch)      ( sfr_ADC1.CSR.CH = ch )

/// get ADC channel AINx
#define ADC1_get_channel()        ( sfr_ADC1.CSR.CH )


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// initialize ADC1
void ADC1_init(void);

/// perform 1-shot measurement
uint16_t ADC1_measure(void);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _ADC1_H_
