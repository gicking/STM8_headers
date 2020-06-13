/**
  \file flash.h
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief declaration of P-FLASH functions/macros
   
  declaration of P-FLASH functions.
  Access to >16b address range (= P-flash above 32kB due to flash starts @ 0x8000)
  requires far pointers (Cosmic & IAR) or helper routines (SDCC) 
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _PFLASH_H_
#define _PFLASH_H_

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/// write 1B to D-flash / EEPROM
uint8_t FLASH_writeByte(uint32_t addr, uint8_t data);



/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _PFLASH_H_
