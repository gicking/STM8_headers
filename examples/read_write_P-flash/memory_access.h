/**
  \file memory_access.h
   
  \author G. Icking-Konert
  \date 2014-04-22
  \version 0.1
   
  \brief declaration of memory read/write routines
   
  declaration of memory read and write routines.
  Access to >16b address range (= P-flash above 32kB due to flash starts @ 0x8000)
  requires far pointers (Cosmic & IAR) or helper routines (SDCC) 
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _MEMORY_ACCESS_H_
#define _MEMORY_ACCESS_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>


///////
// Cosmic compiler read/write macros. Required for missing far pointes in below SDCC 
///////
#if defined(__CSMC__)
  
  // read & write data from memory (16-bit address). For size use 16b pointers
  #if (FLASH_ADDR_WIDTH==16)
    #define read_1B(addr)       (*((uint8_t*) (addr)))                     /**< read 1B from 16-bit address */
    #define read_2B(addr)       (*((uint16_t*) (addr)))                    /**< read 2B from 16-bit address */
    #define read_4B(addr)       (*((uint32_t*) (addr)))                    /**< read 4B from 16-bit address */
    #define write_1B(addr,val)  *((uint8_t*) (addr)) = val                 /**< write 1B to 16-bit address */
    #define write_2B(addr,val)  *((uint16_t*) (addr)) = val                /**< write 1B to 16-bit address */
    #define write_4B(addr,val)  *((uint32_t*) (addr)) = val                /**< write 1B to 16-bit address */
  
  // read & write data from memory (24-bit address). Use 24b far pointers
  #else
    #define read_1B(addr)       (*((@far uint8_t*) (addr)))                /**< read 1B from 24-bit address */
    #define read_2B(addr)       (*((@far uint16_t*) (addr)))               /**< read 2B from 24-bit address */
    #define read_4B(addr)       (*((@far uint32_t*) (addr)))               /**< read 4B from 24-bit address */
    #define write_1B(addr,val)  *((@far uint8_t*) (addr)) = val            /**< write 1B to 24-bit address */
    #define write_2B(addr,val)  *((@far uint16_t*) (addr)) = val           /**< write 1B to 24-bit address */
    #define write_4B(addr,val)  *((@far uint32_t*) (addr)) = val           /**< write 1B to 24-bit address */
  #endif


///////
// IAR compiler read/write macros. Required for missing far pointes in below SDCC 
///////
#elif defined(__ICCSTM8__)
  
  // read & write data from memory (16-bit address). For size use 16b pointers
  #if (FLASH_ADDR_WIDTH==16)
    #define read_1B(addr)       (*((uint8_t*) (uint16_t) (addr)))          /**< read 1B from 16-bit address */
    #define read_2B(addr)       (*((uint16_t*) (uint16_t) (addr)))         /**< read 2B from 16-bit address */
    #define read_4B(addr)       (*((uint32_t*) (uint16_t) (addr)))         /**< read 4B from 16-bit address */
    #define write_1B(addr,val)  *((uint8_t*) (uint16_t) (addr)) = val      /**< write 1B to 16-bit address */
    #define write_2B(addr,val)  *((uint16_t*)(uint16_t)  (addr)) = val     /**< write 1B to 16-bit address */
    #define write_4B(addr,val)  *((uint32_t*) (uint16_t) (addr)) = val     /**< write 1B to 16-bit address */
  
  // read & write data from memory (24-bit address). Use 24b far pointers
  #else
    #define read_1B(addr)       (*((uint8_t __far*) (addr)))               /**< read 1B from 24-bit address */
    #define read_2B(addr)       (*((uint16_t __far*) (addr)))              /**< read 2B from 24-bit address */
    #define read_4B(addr)       (*((uint32_t __far*) (addr)))              /**< read 4B from 24-bit address */
    #define write_1B(addr,val)  *((uint8_t __far*) (addr)) = val           /**< write 1B to 24-bit address */
    #define write_2B(addr,val)  *((uint16_t __far*) (addr)) = val          /**< write 1B to 24-bit address */
    #define write_4B(addr,val)  *((uint32_t __far*) (addr)) = val          /**< write 1B to 24-bit address */
  #endif


///////
// SDCC compiler read/write macros. Required for missing far pointes in SDCC 
///////
#elif defined(__SDCC)

  // read & write data from memory (16-bit address)
  #if (FLASH_ADDR_WIDTH==16)
    #define read_1B(addr)       (*((uint8_t*) (addr)))                     /**< read 1B from 16-bit address */
    #define read_2B(addr)       (*((uint16_t*) (addr)))                    /**< read 2B from 16-bit address */
    #define read_4B(addr)       (*((uint32_t*) (addr)))                    /**< read 4B from 16-bit address */
    #define write_1B(addr,val)  *((uint8_t*) (addr)) = val                 /**< write 1B to 16-bit address */
    #define write_2B(addr,val)  *((uint16_t*) (addr)) = val                /**< write 1B to 16-bit address */
    #define write_4B(addr,val)  *((uint32_t*) (addr)) = val                /**< write 1B to 16-bit address */

  // read & write data from memory (24-bit address). SDCC doesn't support far pointers -> use inline assembly
  #else
    
    // global variables for interfacing with SDCC assembler
    #if defined(_MAIN_)
      volatile uint32_t          g_mem_addr;     ///< address for interfacing to below assembler
      volatile uint32_t          g_mem_val;      ///< data for r/w via below assembler
    #else // _MAIN_
      extern volatile uint32_t   g_mem_addr;
      extern volatile uint32_t   g_mem_val;
    #endif // _MAIN_


    /**
      \fn uint8_t read_1B(uint32_t addr)
  
      \brief read 1 byte from memory (inline)
      
      \param[in] addr  address to read from

      \return 1B data read from memory

      Inline function to read 1B from memory. 
      Required for SDCC and >64kB due to lack of far pointers
    */
    inline uint8_t read_1B(uint32_t addr) {
      
      // pass data between C and assembler via global variables
      extern volatile uint32_t g_mem_addr;
      extern volatile uint8_t  g_mem_val;      // use lowest 8bit of 32bit variable
      
      // set address
      g_mem_addr = addr;
      
      // use inline assembler for actual read
      __asm
        push a 
        ldf  a,[_g_mem_addr+1].e
        ld   _g_mem_val, a
        pop  a
      __endasm;

      // return data
      return(g_mem_val);

    } // read_1B


    /**
      \fn uint16_t read_2B(uint32_t addr)
  
      \brief read 2 bytes from memory (inline)
      
      \param[in] addr  address to read from

      \return 2B data read from memory

      Inline function to read 2B from memory. 
      Required for SDCC and >64kB due to lack of far pointers
    */
    inline uint16_t read_2B(uint32_t addr) {
      
      // pass data between C and assembler via global variables
      extern volatile uint32_t g_mem_addr;
      extern volatile uint16_t g_mem_val;      // use lowest 16bit of 32bit variable

      // set address
      g_mem_addr = addr;
      
      // use inline assembler for actual read
      __asm
        push a 
        ldf  a,[_g_mem_addr+1].e
        ld   _g_mem_val,a
        ldw  x,#1
        ldf  a,([_g_mem_addr+1].e,x)
        ld   _g_mem_val+1,a
        pop  a
      __endasm;

      // return data
      return(g_mem_val);

    } // read_2B
  

    /**
      \fn uint16_t read_4B(uint32_t addr)
  
      \brief read 4 bytes from memory (inline)
      
      \param[in] addr  address to read from

      \return 4B data read from memory

      Inline function to read 4B from memory. 
      Required for SDCC and >64kB due to lack of far pointers
    */
    inline uint32_t read_4B(uint32_t addr) {
      
      // pass data between C and assembler via global variables
      extern volatile uint32_t g_mem_addr;
      extern volatile uint32_t g_mem_val;

      // set address
      g_mem_addr = addr;
      
      // use inline assembler for actual read
      __asm
        push a
        ldf  a,[_g_mem_addr+1].e
        ld   _g_mem_val,a
        ldw  x,#1
        ldf  a,([_g_mem_addr+1].e,x)
        ld  _g_mem_val+1,a
        ldw  x,#2
        ldf  a,([_g_mem_addr+1].e,x)
        ld   _g_mem_val+2,a
        ldw  x,#3
        ldf  a,([_g_mem_addr+1].e,x)
        ld   _g_mem_val+3,a
        pop  a
      __endasm;

      // return data
      return(g_mem_val);

    } // read_4B
  

    /**
      \fn void write_1B(uint32_t addr, uint8_t val)
  
      \brief write 1 byte to memory (inline)
      
      \param[in] addr  address to read from
      \param[in] val   data to write

      Inline function to write 1B to memory. 
      Required for SDCC and >64kB due to lack of far pointers
    */
    inline void write_1B(uint32_t addr, uint8_t val) {
      
      // pass data between C and assembler via global variables
      extern volatile uint32_t g_mem_addr;
      extern volatile uint8_t  g_mem_val;      // use lowest 8bit of 32bit variable
      
      // set address & value
      g_mem_addr = addr;
      g_mem_val  = val;
      
      // use inline assembler for actual write
      __asm
        push a
        ld   a,_g_mem_val
        ldf  [_g_mem_addr+1].e,a
        pop  a
      __endasm;

    } // write_1B

    
    /**
      \fn void write_2B(uint32_t addr, uint16_t val)
  
      \brief write 2 bytes to memory (inline)
      
      \param[in] addr  address to read from
      \param[in] val   data to write

      Inline function to write 2B to memory. 
      Required for SDCC and >64kB due to lack of far pointers
    */
    inline void write_2B(uint32_t addr, uint16_t val) {
      
      // pass data between C and assembler via global variables
      extern volatile uint32_t g_mem_addr;
      extern volatile uint16_t  g_mem_val;      // use lowest 16bit of 32bit variable
      
      // set address & value
      g_mem_addr = addr;
      g_mem_val  = val;
      
      // use inline assembler for actual write
      __asm
        push a
        ld   a,_g_mem_val
        ldf  [_g_mem_addr+1].e,a
        ld   a,_g_mem_val+1
        ldw  x,#1
        ldf  ([_g_mem_addr+1].e,x),a
        pop  a
      __endasm;

    } // write_2B
  

    /**
      \fn void write_2B(uint32_t addr, uint32_t val)
  
      \brief write 4 bytes to memory (inline)
      
      \param[in] addr  address to read from
      \param[in] val   data to write

      Inline function to write 4B to memory. 
      Required for SDCC and >64kB due to lack of far pointers
    */
    inline void write_4B(uint32_t addr, uint32_t val) {
      
      // pass data between C and assembler via global variables
      extern volatile uint32_t g_mem_addr;
      extern volatile uint32_t g_mem_val;

      // set address & value
      g_mem_addr = addr;
      g_mem_val  = val;
      
      // use inline assembler for actual write
      __asm
        push a
        ld   a,_g_mem_val
        ldf  [_g_mem_addr+1].e,a
        ld   a,_g_mem_val+1
        ldw  x,#1
        ldf  ([_g_mem_addr+1].e,x),a
        ld   a,_g_mem_val+2
        ldw  x,#2
        ldf  ([_g_mem_addr+1].e,x),a
        ld   a,_g_mem_val+3
        ldw  x,#3
        ldf  ([_g_mem_addr+1].e,x),a
        pop  a
      __endasm;

    } // write_4B

  #endif // (FLASH_ADDR_WIDTH==32)

#endif // __SDCC

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _MEMORY_ACCESS_H_
