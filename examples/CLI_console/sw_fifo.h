/**
  \file sw_fifo.h
   
  \author G. Icking-Konert
  \date 2015-04-06
  \version 0.1
   
  \brief declaration and implementation of inline functions and macros for a SW FIFO
   
  declares and implements a generic SW FIFO buffer, e.g. for sending and receiving via UART.
  Most action is done inside of interrupt service routines. For speed functions are declared 
  as inline. This FIFO is a generalized version of code by Scott Schmit available at 
  https://eewiki.net/display/microcontroller/Software+FIFO+Buffer+for+UART+Communication
  
  \note
  - FIFO buffer size is configured via FIFO_BUFFER_SIZE (default=32)
    - for different size re-define FIFO_BUFFER_SIZE in the calling C-file
    - several FIFOs within the calling C-file have the same buffer size FIFO_BUFFER_SIZE
    - FIFO buffers in different C-files can have different sizes set via FIFO_BUFFER_SIZE
  - status handling can be optimized for RAM size or flash size & speed via FIFO_OPTIMIZE_RAM (default=1)
    - for a different setting re-define FIFO_OPTIMIZE_RAM in the calling C-file
    - FIFO_OPTIMIZE_RAM=1 --> status bits are stored in 1 byte --> save 2B RAM/FIFO
    - FIFO_OPTIMIZE_RAM=0 --> each status flag is stored in 1 byte --> save ~25B flash/FIFO and gain some speed
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _FIFO_H_
#define _FIFO_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

// default FIFO size in bytes; can be overwritten by calling file
#ifndef FIFO_BUFFER_SIZE
  #define FIFO_BUFFER_SIZE 32
#endif

// default handling of FIFO status. 1: save 2B RAM/FIFO; 0: save ~25B flash/FIFO and gain some speed
#ifndef FIFO_OPTIMIZE_RAM
  #define FIFO_OPTIMIZE_RAM 0
#endif

// read FIFO state from 1B status byte -> optimize RAM size
#if FIFO_OPTIMIZE_RAM
  #define FIFO_NOT_EMPTY(a)           (a.flags & 0x01)
  #define FIFO_FULL(a)                (a.flags & 0x02)
  #define FIFO_OVERFLOW(a)            (a.flags & 0x04)

  // set FIFO status flags
  #define FIFO_SET_NOT_EMPTY(a)       (a->flags |= 0x01)
  #define FIFO_SET_FULL(a)            (a->flags |= 0x02)
  #define FIFO_SET_OVERFLOW(a)        (a->flags |= 0x04)

  // clear FIFO status flags
  #define FIFO_CLEAR_NOT_EMPTY(a)     (a->flags &= ~0x01)
  #define FIFO_CLEAR_FULL(a)          (a->flags &= ~0x02)
  #define FIFO_CLEAR_OVERFLOW(a)      (a->flags &= ~0x04)

// read FIFO state individual status bytes -> optimize flash size & speed
#else
  #define FIFO_NOT_EMPTY(a)           (a.fifo_not_empty)
  #define FIFO_FULL(a)                (a.fifo_full)
  #define FIFO_OVERFLOW(a)            (a.fifo_overflow)

  // set FIFO status flags
  #define FIFO_SET_NOT_EMPTY(a)       (a->fifo_not_empty = 1)
  #define FIFO_SET_FULL(a)            (a->fifo_full      = 1)
  #define FIFO_SET_OVERFLOW(a)        (a->fifo_overflow  = 1)

  // clear FIFO status flags
  #define FIFO_CLEAR_NOT_EMPTY(a)     (a->fifo_not_empty = 0)
  #define FIFO_CLEAR_FULL(a)          (a->fifo_full      = 0)
  #define FIFO_CLEAR_OVERFLOW(a)      (a->fifo_overflow  = 0)
  
#endif // FIFO_OPTIMIZE_RAM


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL TYPEDEFS
-----------------------------------------------------------------------------*/

/// data structure of the SW FIFO buffer. To safe RAM, encode flags in 1B and use size dependent pointer type
typedef struct {
  uint8_t  buffer[FIFO_BUFFER_SIZE];  // FIFO data buffer
#if FIFO_OPTIMIZE_RAM
  uint8_t  flags;                     // 1B status bits: b0=not empty, b1=full, b2=overflow
#else
  uint8_t  fifo_not_empty;            // status flag for FIFO not empty
  uint8_t  fifo_full;                 // status flag for FIFO full
  uint8_t  fifo_overflow;             // status flag for FIFO overflow
#endif // FIFO_OPTIMIZE_RAM
#if (FIFO_BUFFER_SIZE<256)            // to save RAM, use 1B or 2B pointers
  uint8_t idxFirst;                   // index of oldest byte in buffer
  uint8_t idxLast;                    // index of newest byte in buffer
  uint8_t numBytes;                   // number of bytes in buffer
#else
  uint16_t idxFirst;
  uint16_t idxLast; 
  uint16_t numBytes;
#endif // FIFO_BUFFER_SIZE
} fifo_t;
 
 
/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL INLINE FUNCTIONS
-----------------------------------------------------------------------------*/

/**
  \fn void fifo_init(fifo_t *buf)
   
  \brief init FIFO data structure
  
  \param[in]  buf   pointer to FIFO structure
  
  init FIFO data structure. Actions:
    - reset FIFO index pointers
    - reset status bits

  \note
  to ensure data consistency, disable connected interrupts if
  called from outside interrupt service routine 
   
*/
#if defined(__CSMC__)
  @inline void fifo_init(fifo_t *buf) {
#else // SDCC & IAR
  static inline void fifo_init(fifo_t *buf) {
#endif

  // reset FIFO data
  FIFO_CLEAR_NOT_EMPTY(buf);  // set "FIFO empty" status
  FIFO_CLEAR_FULL(buf);       // reset "FIFO full" status
  FIFO_CLEAR_OVERFLOW(buf);   // set "FIFO overflow" status
  buf->idxFirst = 0;          // index of oldest byte in buffer
  buf->idxLast  = 0;          // index of newest byte in buffer
  buf->numBytes = 0;          // number of bytes in buffer

} // fifo_init


/**
  \fn void fifo_enqueue(fifo_t *buf, uint8_t data)
   
  \brief add a new byte to the FIFO buffer 
  
  \param[in]  buf   pointer to FIFO structure
  \param[in]  data  byte to add to SW FIFO buffer
  
  add a new byte to the SW FIFO buffer. Actions:
    - if space available in buffer, add data to it
    - set "not empty" bit
    - if buffer is full afterwards, add data and set "full" warning bit
    - on buffer overflows discard new data and set "overflow" error bit

  \note
  to ensure data consistency, disable connected interrupts if
  called from outside interrupt service routine 
   
*/
#if defined(__CSMC__)
  @inline void fifo_enqueue(fifo_t *buf, uint8_t data) {
#else // SDCC & IAR
  static inline void fifo_enqueue(fifo_t *buf, uint8_t data) {
#endif
  
  // if the FIFO buffer is full set overflow bit and return immediately
  if(buf->numBytes >= FIFO_BUFFER_SIZE) {
    FIFO_SET_OVERFLOW(buf);
    return;
  }

  // store data as newest element in the buffer
  buf->buffer[buf->idxLast] = data;
     
  // increment index of newest element. Clip to buffer size
  if ((++(buf->idxLast)) >= FIFO_BUFFER_SIZE)
    buf->idxLast = 0;
  
  // increment the bytes counter. If FIFO is full, set warning bit
  if ((++(buf->numBytes)) == FIFO_BUFFER_SIZE)
    FIFO_SET_FULL(buf);
  
  // set "FIFO not empty" bit
  FIFO_SET_NOT_EMPTY(buf);

} // fifo_enqueue



/**
  \fn uint8_t fifo_dequeue(fifo_t *buf)
   
  \brief get oldest byte from the FIFO buffer
  
  \param[in]  buf   pointer to FIFO structure
  
  \return oldest byte from the FIFO buffer. If empty, return 255
  
  get oldest byte from the FIFO buffer. Actions:
    - if data in buffer, return the oldest element and remove it from buffer 
    - if buffer empty, clear the "FIFO not empty" bit
    - clear "FIFO full" warning bit (no longer true)
    - do not change "FIFO overflow" bit to keep track of errors. Needs to be cleared by SW

  \note
  to ensure data consistency, disable connected interrupts if
  called from outside interrupt service routine 
   
*/
#if defined(__CSMC__)
  @inline uint8_t fifo_dequeue(fifo_t *buf) {
#else // SDCC & IAR
  static inline uint8_t fifo_dequeue(fifo_t *buf) {
#endif
  
  uint8_t data = 255;     // =FIFO empty
  
  // if FIFO is empty clear the "FIFO not empty" bit and return immediately
  if(buf->numBytes == 0) {
    FIFO_CLEAR_NOT_EMPTY(buf);
    return(data);
  }

  // grab the oldest element in the buffer
  data = buf->buffer[buf->idxFirst];
  
  // increment index of oldest element. Clip to buffer size
  if ((++(buf->idxFirst)) >= FIFO_BUFFER_SIZE)
    buf->idxFirst = 0;
  
  // decrement the bytes counter. If FIFO is empty, clear "not empty" bit
  if ((--(buf->numBytes)) == 0)
    FIFO_CLEAR_NOT_EMPTY(buf);
 
  // clear full bit, since we just made space
  FIFO_CLEAR_FULL(buf);
  
  // do not clear overflow bit to keep track of error. Needs to be cleared by SW

  // return the read byte
  return(data);
  
} // fifo_dequeue



/**
  \fn uint8_t fifo_peek(fifo_t *buf)
   
  \brief peek oldest byte in FIFO buffer without removing it
  
  \param[in]  buf   pointer to FIFO structure
  
  \return oldest byte from the FIFO buffer. If empty, return 255
  
  peek oldest byte in FIFO buffer without removing it. Actions:
    - if data in buffer, return the oldest element but keep it in buffer 
    - if buffer empty, clear the "FIFO not empty" bit

  \note
  to ensure data consistency, disable connected interrupts if
  called from outside interrupt service routine 
   
*/
#if defined(__CSMC__)
  @inline uint8_t fifo_peek(fifo_t *buf) {
#else // SDCC & IAR
  inline uint8_t fifo_peek(fifo_t *buf) {
#endif

  uint8_t data = 255;   // =FIFO empty
  
  // if FIFO is empty clear the "FIFO not empty" bit and return immediately
  if(buf->numBytes == 0) {
    FIFO_CLEAR_NOT_EMPTY(buf);
    return(data);
  }

  // grab the oldest element in the buffer
  data = buf->buffer[buf->idxFirst];

  // return the read byte
  return(data);
  
} // fifo_peek



/**
  \fn void fifo_print(fifo_t *buf)
   
  \brief for debugging print FIFO content to stdio (requires putchar()!)
  
  \param[in]  buf   pointer to FIFO structure
  
  print FIFO content for debugging using printf(). This needs putchar() to be implemented.
  
  \note
  to ensure data consistency, disable connected interrupts if
  called from outside interrupt service routine 
   
*/
/*
#if defined(__CSMC__)
  @inline void fifo_print(fifo_t *buf) {
#else // SDCC & IAR
  inline void fifo_print(fifo_t *buf) {
#endif

  uint8_t   c;
  
  printf("num=%d;  ", (int) buf->numBytes);
  printf("first=%d; last=%d;  ", (int) buf->idxFirst, (int) buf->idxLast);
#if FIFO_OPTIMIZE_RAM
  printf("flags: 0x%02x;  ", (int) buf->flags);
#else
  printf("empty=%d full=%d overflow=%d;  ", (int) buf->fifo_not_empty, (int) buf->fifo_full, (int) buf->fifo_overflow);
#endif // FIFO_OPTIMIZE_RAM
  printf("data=");
  while (FIFO_NOT_EMPTY((*buf))) {
    c = fifo_dequeue(buf);
    printf("%d ", (int) c);
  }

} // fifo_print
*/

#endif // _FIFO_H_
