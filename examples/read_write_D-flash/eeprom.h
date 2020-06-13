/**
  \file eeprom.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of EEPROM functions/macros
   
  declaration of EEPROM functions.
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

/// write 1B to D-flash / EEPROM
uint8_t EEPROM_writeByte(uint16_t logAddr, uint8_t data);

/// read 1B from D-flash / EEPROM
uint8_t EEPROM_readByte(uint16_t logAddr);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _EEPROM_H_
