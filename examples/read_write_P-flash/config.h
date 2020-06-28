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
    SELECT BOARD (for Cosmic and IAR also set in IDE!)
----------------------------------------------------------*/
#define SDUINO
//#define MUBOARD


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

// select respective UART 
#if defined(SDUINO)
  #define sfr_UART    sfr_UART2
#elif defined(MUBOARD)
  #define sfr_UART    sfr_UART1
#else
  #error board not supported
#endif


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(SDUINO)
  #include "../../include/STM8S105K6.h"
#elif defined(MUBOARD)
  #include "../../include/STM8S207MB.h"
#else
  #error board not supported
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
