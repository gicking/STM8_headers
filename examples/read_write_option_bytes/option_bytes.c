/**
  \file option_bytes.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of OPT functions/macros
   
  implementation of OPT functions.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "option_bytes.h"



/**
  \fn uint8_t write_option_byte(uint16_t addr, uint8_t value)
   
  \brief write option byte
  
  \param[in]  addr   address of option byte
  \param[in]  value  new value
  
  \return operation status (0=value unchanged, 1=value changed, 2=error)

  Change value of option byte, if required.
*/
uint8_t write_option_byte(uint16_t addr, uint8_t value) {

  volatile uint8_t  *addr_near;    // pointer to OPT (in 16-bit address space)

  // check address is in OPT range
  if ((addr < OPTION_ADDR_START) || (addr > OPTION_ADDR_END))
    return(2);
  
  // skip if old value = new value
  addr_near = (uint8_t*) addr;
  if ((*addr_near) == value)
    return(0);
  
  // disable interrupts
  DISABLE_INTERRUPTS();

  // unlock w/e access to EEPROM & option bytes
  sfr_FLASH.DUKR.byte = 0xAE;
  sfr_FLASH.DUKR.byte = 0x56;
  
  // additionally required
  sfr_FLASH.CR2.byte  |= 0x80;
  sfr_FLASH.NCR2.byte &= 0x7F;
  
  // wait until access granted
  while(!(sfr_FLASH.IAPSR.DUL));

  // write new value to option byte
  *(addr_near++) = value;
  
  // wait until done
  while (!(sfr_FLASH.IAPSR.EOP));
  
  // lock EEPROM again against accidental erase/write
  sfr_FLASH.IAPSR.DUL = 0;
  
  // additional lock
  sfr_FLASH.CR2.byte  &= 0x7F;
  sfr_FLASH.NCR2.byte |= 0x80;
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // return success
  return(1);

} // write_option_byte


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
