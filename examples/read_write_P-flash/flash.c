/**
  \file flash.c
   
  \author G. Icking-Konert
  \date 2017-02-19
  \version 0.1
   
  \brief implementation of P-FLASH functions/macros
   
  implementation of P-FLASH functions.
    
  Note: board selection in Makefile / project options
*/

/*----------------------------------------------------------
    SELECT BOARD (via Makefile / IDE project options)
----------------------------------------------------------*/


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "config.h"
#include "flash.h"
#include "memory_access.h"



/**
  \fn uint8_t FLASH_writeByte(uint32_t addr, uint8_t data)
  
  \brief write 1B to P-flash
  
  \param[in] addr       physical address to write to
  \param[in] data       byte to program
  
  \return write successful(=1) or error(=0)

  write single byte to physical address in P-flash
*/
uint8_t FLASH_writeByte(uint32_t addr, uint8_t data) {

  uint8_t    countTimeout;                        // timeout counter
  
  // address range check
  if ((addr < FLASH_ADDR_START) || (addr > FLASH_ADDR_END))
    return(0);
  
  // begin critical cection (disable interrupts)
  DISABLE_INTERRUPTS();
  
  // unlock w/e access to P-flash
  sfr_FLASH.PUKR.byte = 0x56;
  sfr_FLASH.PUKR.byte = 0xAE;
  
  // wait until access granted
  while(!sfr_FLASH.IAPSR.PUL);
  
  // write byte using 16-bit or 32-bit macro/function
  write_1B(addr, data);
    
  // wait until done or timeout (normal flash write measured with 0 --> 100 is more than sufficient)
  countTimeout = 100;                                // ~0.95us/inc -> ~0.1ms
  while ((!sfr_FLASH.IAPSR.EOP) && (countTimeout--));

  // lock P-flash again against accidental erase/write
  sfr_FLASH.IAPSR.PUL = 0;
  
  // critical section (enable interrupts)
  ENABLE_INTERRUPTS();

  // write successful -> return 1
  return(countTimeout != 0);

} // FLASH_writeByte



/**
  \fn uint8_t FLASH_writeDWord(uint32_t addr, uint32_t data)
  
  \brief write 4B to P-flash (big-endian)
  
  \param[in] addr       physical address to write to
  \param[in] data       double word (4B) to program
  
  \return write successful(=1) or error(=0)

  write 4 bytes to physical address in P-flash (big-endian). Note: ECC is over 4B
*/
uint8_t FLASH_writeDWord(uint32_t addr, uint32_t data) {

  uint8_t    countTimeout;                        // timeout counter
  
  // address range check
  if ((addr < FLASH_ADDR_START) || (addr-3 > FLASH_ADDR_END))
    return(0);
  
  // begin critical cection (disable interrupts)
  DISABLE_INTERRUPTS();
  
  // unlock w/e access to P-flash
  sfr_FLASH.PUKR.byte = 0x56;
  sfr_FLASH.PUKR.byte = 0xAE;
  
  // wait until access granted
  while(!sfr_FLASH.IAPSR.PUL);
  
  // enable DWord programming mode
  sfr_FLASH.CR2.WPRG = 1;
  sfr_FLASH.NCR2.NWPRG = 0;
  
  // write 4 bytes using 16-bit or 32-bit macro/function (big-endian)
  write_4B(addr, data);

  // wait until done or timeout (normal flash write measured with 0 --> 100 is more than sufficient)
  countTimeout = 100;                                // ~0.95us/inc -> ~0.1ms
  while ((!sfr_FLASH.IAPSR.EOP) && (countTimeout--));
    
  // lock P-flash again against accidental erase/write
  sfr_FLASH.IAPSR.PUL = 0;
   
  // reset programming mode
  sfr_FLASH.CR2.WPRG = 0;
  sfr_FLASH.NCR2.NWPRG = 1;
 
  // critical section (enable interrupts)
  ENABLE_INTERRUPTS();

  // write successful -> return 1
  return(countTimeout != 0);

} // FLASH_writeDWord


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
