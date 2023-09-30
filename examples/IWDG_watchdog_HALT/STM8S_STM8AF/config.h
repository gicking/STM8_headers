/**
  \file config.h
   
  \brief set project configurations
   
  set project configurations like used device or board etc.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _CONFIG_H_
#define _CONFIG_H_


/*----------------------------------------------------------
    SELECT BOARD
----------------------------------------------------------*/
//#define MUBOARD
//#define NUCLEO_8S207K8
#define NUCLEO_8S208RB


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(MUBOARD)
  #include "../../../include/STM8S207MB.h"
  #define   EXTI_VECTOR   _EXTI4_VECTOR_
  #define   ODR_LED       sfr_PORTH.ODR.ODR2

#elif defined(NUCLEO_8S207K8)
  #include "../../../include/STM8S207K8.h"
  #define   EXTI_VECTOR   _EXTI3_VECTOR_
  #define   ODR_LED       sfr_PORTC.ODR.ODR5

#elif defined(NUCLEO_8S208RB)
  #include "../../../include/STM8S208RB.h"
  #define   EXTI_VECTOR   _EXTI4_VECTOR_
  #define   ODR_LED       sfr_PORTC.ODR.ODR5

#else
  #error undefined board
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
