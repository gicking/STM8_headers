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
#define NUCLEO_8S208RB
//#define STM8L_DISCOVERY
//#define SDUINO


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(NUCLEO_8S208RB)
  #include "../../include/STM8S208RB.h"
  #define sfr_UART             sfr_UART1
  #define _UART_RXNE_VECTOR_   _UART1_R_RXNE_VECTOR_

#elif defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
  #define sfr_UART             sfr_USART1
  #define _UART_RXNE_VECTOR_   _USART_R_RXNE_VECTOR_
  
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
  #define sfr_UART             sfr_UART2
  #define _UART_RXNE_VECTOR_   _UART2_R_RXNE_VECTOR_

#else
  #error undefined board
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
