/**
  \file eeprom.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of EEPROM functions/macros
   
  implementation of EEPROM functions.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "eeprom.h"



/**
  \fn uint8_t EEPROM_writeByte(uint16_t logAddr, uint8_t data)
  
  \brief write 1B to D-flash / EEPROM
  
  \param[in] logAddr    logical address to write to (starting from EEPROM_ADDR_START)
  \param[in] data       byte to program
  
  \return write successful(=1) or error(=0)

  write single byte to logical address in D-flash / EEPROM
*/
uint8_t EEPROM_writeByte(uint16_t logAddr, uint8_t data) {

  uint16_t   addr = EEPROM_ADDR_START + logAddr;  // physical address 
  uint8_t    countTimeout;                        // timeout counter
  
  // address range check
  if (logAddr > EEPROM_SIZE)
    return(0);
  
  // begin critical cection (disable interrupts)
  DISABLE_INTERRUPTS();
  
  // unlock w/e access to EEPROM
  sfr_FLASH.DUKR.byte = 0xAE;
  sfr_FLASH.DUKR.byte = 0x56;
    
  // wait until access granted
  while(!sfr_FLASH.IAPSR.DUL);
  
  // write byte in 16-bit address range
  *((uint8_t*) addr) = data;

  // wait until done or timeout (normal flash write measured with 0 --> 100 is more than sufficient)
  countTimeout = 100;                                // ~0.95us/inc -> ~0.1ms
  while ((!sfr_FLASH.IAPSR.EOP) && (countTimeout--));
    
  // lock EEPROM again against accidental erase/write
  sfr_FLASH.IAPSR.DUL = 0;
  
  // critical section (enable interrupts)
  ENABLE_INTERRUPTS();

  // write successful -> return 1
  return(countTimeout != 0);

} // EEPROM_writeByte



/**
  \fn uint8_t EEPROM_readByte(uint16_t logAddr)
  
  \brief read 1B from D-flash / EEPROM
  
  \param[in] logAddr    logical address to read from (starting from EEPROM_ADDR_START)
  
  \return read byte (0xFF on error)

  read single byte from logical address in D-flash / EEPROM
*/
uint8_t EEPROM_readByte(uint16_t logAddr) {

  uint16_t   addr = EEPROM_ADDR_START + logAddr;  // physical address 
  
  // address range check
  if (logAddr > EEPROM_SIZE)
    return(0xFF);
  
  // write successful -> return 1
  return(*((uint8_t*) addr));

} // EEPROM_readByte


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
