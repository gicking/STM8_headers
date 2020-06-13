/**
  \file i2c.h
   
  \brief declaration of I2C functions/macros
   
  declaration of functions for I2C bus communication
  For I2C bus, see http://en.wikipedia.org/wiki/I2C
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _I2C_H_
#define _I2C_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "config.h"


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/// check I2C busy flag
#define I2C_BUSY  (sfr_I2C.SR3.BUSY)


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// configure I2C bus as master in standard mode
void      i2c_init(void);

/// wait until bus is free
uint8_t   i2c_waitFree(void);

/// generate I2C start condition
uint8_t   i2c_start(void);

/// generate I2C stop condition
uint8_t   i2c_stop(void);

/// write data via I2C
uint8_t   i2c_send(uint8_t addr, uint8_t numTx, uint8_t *Tx);

/// request data via I2C as master
uint8_t   i2c_request(uint8_t addr, uint8_t numRx, uint8_t *Rx);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _I2C_H_
