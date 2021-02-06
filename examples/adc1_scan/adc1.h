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

/// number of scan channels (AIN0...AINx)s
#define ADC1_CHANNELS             4

/// switch off ADC
#define ADC1_power_down()         ( sfr_ADC1.CR1.ADON = 0 )

/// switch on ADC
#define ADC1_power_on()           ( sfr_ADC1.CR1.ADON = 1 )


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_MAIN_'
#if defined(_MAIN_)
  volatile uint16_t           g_ADC_result[ADC1_CHANNELS];
#else // _MAIN_
  extern volatile uint16_t    g_ADC_result[];
#endif // _MAIN_


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// start ADC1 measurement of AIN0..AINx in scan mode
void ADC1_start(void);

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _ADC1_H_
