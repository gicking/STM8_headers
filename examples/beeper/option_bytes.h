/**
  \file option_bytes.h

  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1

  \brief declaration of OPT functions/macros

  declaration of OPT functions.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _EEPROM_H_
#define _EEPROM_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// write option byte
uint8_t write_option_byte(uint16_t addr, uint8_t value);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _EEPROM_H_
