/**
  \file cli.c

  \author G. Icking-Konert
  \date 2021-03-04
  \version 0.1

  \brief implementation of a command line interface base

  implementation of a command line interface base to be used with a serial terminal

  from: https://www.avrfreaks.net/forum/simple-command-interpreter (search for Graynomad)

  Execute call-back function with up to MAX_PARMS optional parameters in HEX or DEC
  Echoes all characters to the serial port.
  Handles backspace for editing. TAB loads the last command into the command buffer
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "cli.h"
#include "uart1.h"


/*----------------------------------------------------------
    MODULE VARIABLES
----------------------------------------------------------*/

/// number of parameters entered after the command
static int16_t          cli_num_parameter;

/// buffer containing current command
static char             cli_cmd_buffer[CLI_LEN_CMDLINE];

/// pointer to next char in cli_cmd_buffer
static int16_t          cli_cmd_buffer_index = 0;

/// pointers to comandline parameters
static char             *cli_parameter[CLI_MAX_PRM];

/// buffer containing last command (for "history")
static char             cli_last_cmd[CLI_LEN_CMDLINE];

/// flag indicating whether user is logged in
static uint8_t          cli_login = 0;


/// structure for command properties
typedef struct
{
  const char    command[CLI_LEN_CMD];         ///< command keyword
  uint8_t       numParameter;                 ///< number of fuction parameters
  void          (*func) (void);               ///< callback function
} cli_command_t;


// declaration of callback functions for below cli_command[]
void  cli_cmd_Help(void);
void  cli_cmd_Login(void);
void  cli_cmd_Logout(void);
void  cli_cmd_Led(void);

/// list of command keywords with corresponding functions
cli_command_t cli_command[] = {
  {"help",    0, cli_cmd_Help},
  {"login",   1, cli_cmd_Login},
  {"logout",  0, cli_cmd_Logout},
  {"led",     1, cli_cmd_Led}
};

/// length of command list
static const int16_t    cli_num_commands = sizeof (cli_command)/ sizeof (cli_command_t);


/*----------------------------------------------------------
    MODULE FUNCTIONS
----------------------------------------------------------*/

/**
  \fn uint16_t cli_hex_to_int(const char *ptr)

  \brief convert hex string to int

  \param ptr  string containing hex number

  \return converted hex string as int

  convert a hex string without leading '0x' to int16_t.
  Any non-hex character terminates the conversion
*/
uint16_t cli_hex_to_int(const char *ptr)
{
  uint16_t  value = 0;
  char      ch = *ptr;

  // skip leading whitespaces
  while (ch == ' ' || ch == '\t')
    ch = *(++ptr);

  // actual string conversion
  for (;;)
  {
    if (ch >= '0' && ch <= '9')
      value = (value << 4) + (ch - '0');
    else if (ch >= 'A' && ch <= 'F')
      value = (value << 4) + (ch - 'A' + 10);
    else if (ch >= 'a' && ch <= 'f')
      value = (value << 4) + (ch - 'a' + 10);
    else
      break;
    ch = *(++ptr);
  }

  // return converted value
  return value;

} // cli_hex_to_int()



/**
  \fn int16_t cli_get_parameter(uint8_t parameter, uint8_t base)

  \brief convert command parameter acc. to given base

  \param parameter  string containing parameter (dec or hex)
  \param base       base to convert with (CLI_HEX or CLI_DEC)

  \return converted parameter string as int

  convert a parameter string to a number acc. to the specified base.
  In case of an invalid base, return 2^15-1 as invalid
*/
int16_t cli_get_parameter(uint8_t parm, uint8_t base)
{
  int16_t result;

  switch (base)
  {
    case CLI_HEX:
      result = cli_hex_to_int(cli_parameter[parm]);
      break;

    case CLI_DEC:
      result = atoi(cli_parameter[parm]);
      break;

    default:
      result = INT16_MAX;

  } // switch (base)

  // return converted parameter
  return result;

} // cli_get_parameter()



/**
  \fn void cli_split_command(void)

  \brief split commandline into command and parameters

  split commandline into command and parameters.
*/
void cli_split_command(void)
{
  uint8_t i, p;

  // clear the pointer array
  cli_num_parameter = 0;
  for (i = 0; i < CLI_MAX_PRM; i++)
    cli_parameter[i] = NULL;

  // scan the command line, replace spaces with '\0'
  // and save the location of the first char after null
  for (i = 0, p = 0; cli_cmd_buffer[i] != CLI_NULL; i++)
  {
    if (cli_cmd_buffer[i] == CLI_SPACE)
    {
      cli_cmd_buffer[i] = CLI_NULL;
      cli_parameter[p++] = &(cli_cmd_buffer[i]) + 1;
      cli_num_parameter++;
    }

  } // loop over commandline

} // cli_split_command()



/**
  \fn void cli_process_command(void)

  \brief execute commandline command

  Execute commandline command with specified parameters.
*/
void cli_process_command(void)
{
  uint8_t   cmd;
  uint8_t   flagCmd;

  printf("\n");

  /////////////////////////////////////////////////////
  // trap just a CRLF
  if (cli_cmd_buffer[0] == CLI_NULL)
  {
    printf(CLI_PROMPT);
    return;
  }

  /////////////////////////////////////////////////////
  // save this command for later use with TAB
  memcpy(cli_last_cmd, cli_cmd_buffer, sizeof(cli_last_cmd));

  /////////////////////////////////////////////////////
  // Chop the command line into substrings by
  // replacing ' ' with '\0'
  // Also adds pointers to the substrings
  cli_split_command();

  /////////////////////////////////////////////////////
  // Scan the command table looking for a match
  flagCmd = 0;
  for (cmd = 0; cmd < cli_num_commands; cmd++)
  {
    if (strcmp(cli_command[cmd].command, (char*)cli_cmd_buffer) == 0)
    {
      // marks as processed
      flagCmd = 1;

      // correct number of commandline parameters
      if (cli_command[cmd].numParameter == cli_num_parameter)
      {
        cli_command[cmd].func();  // command found, run its function
      }

      // wrong number of commandline parameters
      else
      {
        printf("wrong number of parameters (exp. %d, read %d)\n\n",
          cli_command[cmd].numParameter, cli_num_parameter);
      }
    }
  }

  /////////////////////////////////////////////////////
  // if we get here no valid command was found
  if (flagCmd == 0)
  {
    printf("command unknown '%s'\n\n", (char*) cli_cmd_buffer);
  }

  // re-initialize command buffer
  cli_cmd_buffer_index = 0;
  cli_cmd_buffer[0] = CLI_NULL;
  printf(CLI_PROMPT);

} // cli_process_command()



/*----------------------------------------------------------
    CALLBACK FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void cli_cmd_Help(void)

  \brief print list of available commands

  print list of available commands.
*/
void cli_cmd_Help(void)
{
  uint8_t  i;
	
	//int16_t pin = cli_get_parameter(0, CLI_HEX);
  //int16_t val = cli_get_parameter(1, HEX);

  printf("List of available commands: ");
  for (i=0; i<cli_num_commands; i++)
  {
    printf("%s ", cli_command[i].command);
  }
  printf("\n\n");

} // cli_cmd_Help()



/**
  \fn void cli_cmd_Login(void)

  \brief log in to session

  log in to session with password CLI_LOGIN_PWD (hex).
  Unlock resources.
*/
void cli_cmd_Login(void)
{
  //int16_t pin = cli_get_parameter(0, CLI_HEX);
  //int16_t val = cli_get_parameter(1, HEX);

  if (CLI_LOGIN_PWD == cli_get_parameter(0, CLI_HEX))
  {
    printf("login ok\n\n");
    cli_login = 1;
  }
  else
  {
    printf("wrong password, login failed (try 1234)\n\n");
    cli_login = 0;
  }

} // cli_cmd_Login()



/**
  \fn void cli_cmd_Logout(void)

  \brief log out of session

  log out of session. Lock resources again
*/
void cli_cmd_Logout(void)
{
  //int16_t pin = cli_get_parameter(0, CLI_HEX);
  //int16_t val = cli_get_parameter(1, HEX);

  cli_login = 0;
  printf("logout\n\n");

} // cli_cmd_Logout()



/**
  \fn void cli_cmd_Led(void)

  \brief control LED state

  control LED state
*/
void cli_cmd_Led(void)
{
  //int16_t pin = cli_get_parameter(0, CLI_HEX);
  //int16_t val = cli_get_parameter(1, HEX);

  if (cli_login == 1)
  {
    if (cli_get_parameter(0, CLI_DEC))
    {
      sfr_PORTC.ODR.ODR5 = 1;
      printf("LED ON\n\n");
    }
    else
    {
      sfr_PORTC.ODR.ODR5 = 0;
      printf("LED OFF\n\n");
    }
  }
  else
  {
    printf("not logged in\n\n");
  }
} // cli_cmd_Led()



/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void cli_greeting(void)

  \brief print inital greeting message

  print inital greeting message
*/
void cli_greeting()
{
  printf("\n");
  printf("Debug Console v%s\n\n", CLI_VERSION);
  printf("type 'help' to get a list of available commands\n");
  printf("press TAB for last command\n\n");
  printf(CLI_PROMPT);

} // cli_greeting()



/**
  \fn void cli_handler(void)

  \brief handler for keyboard input

  main commandline handler. Read keyboard input and react accordingly
*/
void cli_handler()
{
  char c;

  if (UART1_check_Rx())
  {
    c = UART1_receive();

    c = tolower(c);
    switch (c) {

      case CLI_TAB:
        // copy the last command into the command buffer
        // then echo it to the terminal and set the
        // the buffer's index pointer to the end
        memcpy(cli_cmd_buffer, cli_last_cmd, sizeof(cli_last_cmd));
        cli_cmd_buffer_index = strlen(cli_cmd_buffer);
        printf("%s", cli_cmd_buffer);
        break;

      case CLI_BACKSPACE:
        if (cli_cmd_buffer_index > 0)
        {
          cli_cmd_buffer_index--;
          cli_cmd_buffer[cli_cmd_buffer_index] = CLI_NULL;
          UART1_send_byte(CLI_BACKSPACE);
          UART1_send_byte(CLI_SPACE);
          UART1_send_byte(CLI_BACKSPACE);
        }
        break;

      case CLI_LF:
        cli_process_command ();
        while (UART1_check_Rx())    // remove any following CR
          UART1_receive();
        break;

      case CLI_CR:
        cli_process_command ();
        while (UART1_check_Rx())    // remove any following LF
          UART1_receive();
        break;

      default:  // just put the char in the buffer
        cli_cmd_buffer[cli_cmd_buffer_index++] = c;
        cli_cmd_buffer[cli_cmd_buffer_index] = CLI_NULL;
        UART1_send_byte(c);

    } // switch received char

  } // if char received

} // cli_handler()



/**
  \fn uint8_t cli_read_login(void)

  \brief get login status

  get current login status. Not required for CLI functionality
*/
uint8_t cli_read_login()
{
  return cli_login;

} // cli_handler(void)


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
