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
#define STM8S_DISCOVERY
//#define STM8L_DISCOVERY


/*----------------------------------------------------------
    PROJECT SETTINGS
----------------------------------------------------------*/
#define FSYS_FREQ         16000000L    // system frequency [Hz]

/*
* Use 485 or 232, default: 485
* Use 232 for testing purposes or very simple applications that do not require RS485 and bus topology.
*/
#define PHYSICAL_TYPE    485


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"

  #define clientAddress 0x01

  // enable sending PA6 = CN1, pin 12
  #define TRANSCEIVER_ENABLE_PORT      sfr_PORTA
  #define TRANSCEIVER_ENABLE_PIN       PIN6

  // LED pin
  #define LED_PORT   sfr_PORTD
  #define LED_PIN    PIN0

  // select UART
  #define sfr_UART                sfr_UART2
  #define _UART_T_TXE_VECTOR_     _UART2_T_TXE_VECTOR_
  #define _UART_R_RXNE_VECTOR_    _UART2_R_RXNE_VECTOR_
  #define UART_CR1_RESET_VALUE    sfr_UART2_CR1_RESET_VALUE
  #define UART_CR2_RESET_VALUE    sfr_UART2_CR2_RESET_VALUE
  #define UART_CR3_RESET_VALUE    sfr_UART2_CR3_RESET_VALUE

#elif defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"

  #define clientAddress 0x02

  // enable sending PD7
  #define TRANSCEIVER_ENABLE_PORT      sfr_PORTD
  #define TRANSCEIVER_ENABLE_PIN       PIN7

  // LED pin
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN7

  // select UART
  #define sfr_UART                sfr_USART1
  #define _UART_T_TXE_VECTOR_     _USART_T_TXE_VECTOR_
  #define _UART_R_RXNE_VECTOR_    _USART_R_RXNE_VECTOR_
  #define UART_CR1_RESET_VALUE    sfr_USART1_CR1_RESET_VALUE
  #define UART_CR2_RESET_VALUE    sfr_USART1_CR2_RESET_VALUE
  #define UART_CR3_RESET_VALUE    sfr_USART1_CR3_RESET_VALUE

#else
  #error undefined board
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
