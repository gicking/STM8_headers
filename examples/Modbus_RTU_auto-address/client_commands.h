/**
  \file client_commands.h

  \author G. Icking-Konert
  \date 2021-02-27
  \version 0.1

  \brief declaration of Modbus client commands

  declaration of Modbus client commands
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _CLIENT_COMMANDS_H_
#define _CLIENT_COMMANDS_H_


// implement client command codes
// "Command pending" is indicated by highest bit set (bit15). Is cleared after command is completed
// Error is indicated by client by setting bit14. In this case reg[1] contains the error code (see below)
#define CMD_SET_LED								0x8001			///< set LED
#define CMD_REG_RANDOMIZE					0x8002			///< fill registers with random numbers
#define CMD_ID_RANDOMIZE					0x8003			///< randomize Modbus IDs


#define ERROR_ILLEGAL_COMMAND			1						///< unknown command code
#define ERROR_PARAMETER_NUMBER		2						///< wrong number of parameters for command

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _CLIENT_COMMANDS_H_
