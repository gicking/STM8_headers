/************************************************************************
  \file       yaMBSistm8.h
  \title:     Yet another (small) Modbus (server) implementation for the STM8.
  \authors:   Max Brueggemann (original AVR implementation), Georg Icking-Konert (STM8 port)

  \brief declaration of Modbus RTU functions/macros

  Interrupt-based Modbus implementation for STM8 microcontrollers.
  The Modbus implementation guidelines at modbus.org call for response
  timeouts in the range of several seconds, hence only timing critical
  parts have been implemented within ISRs. The actual handling of the Modbus
  frame can easily be done in the main while loop.

  Hardware: any STM8 with hardware UART, and TIM4
  License:  BSD-3-Clause

  LICENSE:

  Copyright 2017 Max Brueggemann (original AVR implementation)
  Copyright 2021 Georg Icking-Konert (STM8 port)

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software without
  specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
  THE POSSIBILITY OF SUCH DAMAGE.

************************************************************************/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef YAMBISTM8_H
#define YAMBISTM8_H


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include "config.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL MACROS
-----------------------------------------------------------------------------*/

// allow multiple addresses. This is useful for building gateways, routers or clients that for whatever reason need multiple addresses.
//#define MB_MULTIPLE_ADR


// Maximum Modbus frame size. Size must be within 8..255.
#define MB_MAX_FRAME_INDEX    100
#if (MB_MAX_FRAME_INDEX < 8) || (MB_MAX_FRAME_INDEX > 255)
  #error MB_MAX_FRAME_INDEX must be within 8..255
#endif


// Bit definition for Modbus state
#define MB_STATE_BUS_TIMEOUT                0
#define MB_STATE_RECEIVING                  1
#define MB_STATE_TRANSMITTING               2
#define MB_STATE_RECEIVE_COMPLETED          3
#define MB_STATE_TRANSMIT_REQUESTED         4
#define MB_STATE_TIMER_ACTIVE               5
#define MB_STATE_GAP_DETECTED               6


// Modbus timeouts [ms]. Not fully acc. to specification, but reduces interrupt load
#define MB_INTER_CHAR_TIMEOUT               1
#define MB_INTER_FRAME_DELAY_RECEIVE_START  2
#define MB_INTER_FRAME_DELAY_RECEIVE_END    3


/**
 * @enum MB_FC
 * @brief Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * Refer to modbus.org for further information.
 * It's good practice to return exception code 01 in case you receive a function code
 * that you haven't implemented in your application.
 */
enum MB_FC
{
  MB_FC_NONE                      = 0,      ///< null operator
  MB_FC_READ_COILS                = 1,      ///< FCT=1 -> read coils or digital outputs
  MB_FC_READ_DISCRETE_INPUT       = 2,      ///< FCT=2 -> read digital inputs
  MB_FC_READ_REGISTERS            = 3,      ///< FCT=3 -> read registers or analog outputs
  MB_FC_READ_INPUT_REGISTER       = 4,      ///< FCT=4 -> read analog inputs
  MB_FC_WRITE_COIL                = 5,      ///< FCT=5 -> write single coil or output
  MB_FC_WRITE_REGISTER            = 6,      ///< FCT=6 -> write single register
  MB_FC_WRITE_MULTIPLE_COILS      = 15,     ///< FCT=15 -> write multiple coils or outputs
  MB_FC_WRITE_MULTIPLE_REGISTERS  = 16      ///< FCT=16 -> write multiple registers
};


/**
 * @enum MB_EC
 * @brief Modbus Exception Codes
 *
 * Refer to modbus.org for further information.
 * It's good practice to return exception code 01 in case you receive a function code
 * that you haven't implemented in your application.
 */
enum MB_EC
{
  MB_EC_ILLEGAL_FUNCTION          = 1,      ///< EC=1 -> function code not supported
  MB_EC_ILLEGAL_DATA_ADDRESS      = 2,      ///< EC=2 -> data pointer out of range
  MB_EC_ILLEGAL_DATA_VALUE        = 3,      ///< EC=3 -> data value out of valid range
  MB_EC_SLAVE_DEVICE_FAILURE      = 4,      ///< EC=4 -> unrecoverable  error  occurred
  MB_EC_ACKNOWLEDGE               = 5,      ///< EC=5 -> special EC for commands which take a long time to process
  MB_EC_SLAVE_DEVICE_BUSY         = 6,      ///< EC=6 -> special EC for commands which take a long time to process
  MB_EC_NEGATIVE_ACKNOWLEDGE      = 7,      ///< EC=7 -> special EC for commands which take a long time to process
  MB_EC_MEMORY_PARITY_ERROR       = 8       ///< EC=8 -> memory check error
};


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_MAIN_'
#if defined(_MAIN_)
  volatile uint8_t            g_modbus_buffer[MB_MAX_FRAME_INDEX+1];  ///< receive/transmit data array
  volatile uint16_t           g_modbus_dataAmount = 0;                ///< number of bits/registers requested
  volatile uint16_t           g_modbus_dataLocation = 0;              ///< address of the first requested bit/register
#else // _MAIN_
  extern volatile uint8_t     g_modbus_buffer[];
  extern volatile uint16_t    g_modbus_dataAmount;
  extern volatile uint16_t    g_modbus_dataLocation;
#endif // _MAIN_


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize serial communication
void modbus_init(uint32_t BR);

// this only applies to single address mode.
#ifndef MB_MULTIPLE_ADR

  /// read device Modbus address
  uint8_t modbus_getAddress(void);

  /// set device Modbus address
  void modbus_setAddress(uint8_t newadr);

#endif // ! MB_MULTIPLE_ADR

/// send a Modbus response.
void modbus_sendMessage(uint8_t packtop);

/// send a Modbus exception.
void modbus_sendException(uint8_t exceptionCode);

/// discard the current transaction
void modbus_reset(void);

// get Modbus state
uint8_t modbus_getBusState(void);

/// get number of bits/registers requested
uint16_t modbus_requestedAmount(void);

/// get address of the first requested bit/register
uint16_t modbus_requestedAddress(void);

/// handle single/multiple input/coil reading and single/multiple coil writing.
uint8_t modbus_exchangeBits(uint8_t *ptrToInArray, uint16_t startAddress, uint16_t size);

/// handle single/multiple register reading and single/multiple register writing.
uint8_t modbus_exchangeRegisters(uint16_t *ptrToInArray, uint16_t startAddress, uint16_t size);

/// UART receive interrupt
ISR_HANDLER(UART_RXNE_ISR, _UART_R_RXNE_VECTOR_);

/// shared UART transmit TXE & TC interrupt
ISR_HANDLER(UART_TXE_ISR, _UART_T_TXE_VECTOR_);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // YAMBISTM8_H
