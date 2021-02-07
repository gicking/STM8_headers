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
//#define STM8L_DISCOVERY
#define SDUINO


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
  #define LED_PORT   sfr_PORTE
  #define LED_PIN    PIN7
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#else
  #error undefined board
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
