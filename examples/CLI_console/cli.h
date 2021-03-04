/**
  \file cli.h

  \author G. Icking-Konert
  \date 2021-03-04
  \version 0.1

  \brief declaration of a command line interface base

  declaration of a command line interface base to be used with a serial terminal

  from: https://www.avrfreaks.net/forum/simple-command-interpreter (search for Graynomad)

  Execute call-back function with up to MAX_PARMS optional parameters in HEX or DEC
  Echoes all characters to the serial port.
  Handles backspace for editing. TAB loads the last command into the command buffer
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _CLI_H_
#define _CLI_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>



/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL MACROS / TYPEDEFS
-----------------------------------------------------------------------------*/

/*------------------
  GENERAL
------------------*/

/// CLI parameter type
#define CLI_DEC           0
#define CLI_HEX           1

/// max. length of command keyword
#define CLI_LEN_CMD       15

/// max. length of command line
#define CLI_LEN_CMDLINE   50

/// max. number of commandline parameters
#define CLI_MAX_PRM       5

// ASCII codes of keys
#define CLI_NULL          0
#define CLI_LF            10
#define CLI_CR            13
#define CLI_TAB           9
#define CLI_SPACE         32
#define CLI_ESC           27
#define CLI_BACKSPACE     8


/*------------------
  PROJECT SPECIFIC
------------------*/

/// printed version number
#define CLI_VERSION       "1.0"

/// login password (hex)
#define CLI_LOGIN_PWD     0x1234

/// printed version number
#define CLI_PROMPT        "> "


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// print inital greeting message
void    cli_greeting(void);

/// handler for keyboard input
void    cli_handler(void);

/// get login status
uint8_t cli_read_login(void);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CLI_H_
