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

/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_MAIN_'
#if defined(_MAIN_)
  volatile uint16_t           g_ADC_result;                 ///< cumulated ADC results
#else // _MAIN_
  extern volatile uint16_t    g_ADC_result;
#endif // _MAIN_


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

/// set ADC channel to AINx
#define ADC1_set_channel(ch)      ( sfr_ADC1.CSR.CH = ch )

/// get ADC channel AINx
#define ADC1_get_channel()        ( sfr_ADC1.CSR.CH )

/// switch off ADC
#define ADC1_power_down()         ( sfr_ADC1.CR1.ADON = 0 )

/// switch on ADC
#define ADC1_power_on()           ( sfr_ADC1.CR1.ADON = 1 )



/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// initialize ADC1
void ADC1_init(uint8_t channel);

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _ADC1_H_
