/************************************************************************
  \file       yaMBSistm8.c
  \title:     Yet another (small) Modbus (server) implementation for the STM8.
  \authors:   Max Brueggemann (original AVR implementation), Georg Icking-Konert (STM8 port)

  \brief implementation of Modbus RTU functions/macros

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

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "yaMBSistm8.h"   // Modbus handler


/*----------------------------------------------------------
    MODULE VARIABLES
----------------------------------------------------------*/

static volatile uint8_t   _modbus_busState       = 0;
static volatile uint16_t  _modbus_dataPos        = 0;
static volatile uint16_t  _modbus_timer          = 0;
static volatile uint8_t   _modbus_packetTopIndex = 7;
#ifndef MB_MULTIPLE_ADR
  static volatile uint8_t _modbus_address = 0x00;
#endif


/*----------------------------------------------------------
    MODULE FUNCTIONS
----------------------------------------------------------*/

// only for half-duplex RS485 communication
#if PHYSICAL_TYPE == 485

  /**
    \fn void _modbus_transceiver_txen(void)

    \brief enable RS485 transmission

    Enable RS485 transmission by setting EN pin high

    Note: module function
  */
  void _modbus_transceiver_txen(void)
  {
    TRANSCEIVER_ENABLE_PORT.ODR.byte |= TRANSCEIVER_ENABLE_PIN;
  }


  /**
    \fn void _modbus_transceiver_rxen(void)

    \brief enable RS485 reception

    Enable RS485 reception by setting EN pin low

    Note: module function
  */
  void _modbus_transceiver_rxen(void)
  {
    TRANSCEIVER_ENABLE_PORT.ODR.byte &= ~(TRANSCEIVER_ENABLE_PIN);
  }
#endif



/**
  \fn void _modbus_saveLocation(void)

  \brief save address and amount

  Save Modbus starting address and number of coils/registers to
  global variables.

  Note: module function
*/
void _modbus_saveLocation(void)
{
  g_modbus_dataLocation  = (uint16_t) (g_modbus_buffer[3]) | (((uint16_t) (g_modbus_buffer[2])) << 8);
  if ((g_modbus_buffer[1] == MB_FC_WRITE_REGISTER) || (g_modbus_buffer[1] == MB_FC_WRITE_COIL))
    g_modbus_dataAmount = 1;
  else
    g_modbus_dataAmount = (uint16_t) (g_modbus_buffer[5]) | (((uint16_t) (g_modbus_buffer[4])) << 8);

} //_modbus_saveLocation()



/**
  \fn void _modbus_listBitCopy(uint8_t *source, uint16_t sourceNr, uint8_t *target, uint16_t targetNr)

  \brief copy a single bit from one char to another char

  \param  source     source array
  \param  sourceNr   source bit index
  \param  target     target array
  \param  targetNr   target bit index

  Copy a single bit from one char to another char (or arrays thereof)

  Note: module function
*/
void _modbus_listBitCopy(uint8_t *source, uint16_t sourceNr, uint8_t *target, uint16_t targetNr)
{
  if (*(source+(sourceNr/8)) & (1 << (sourceNr-((sourceNr/8)*8))))
  {
    *(target+(targetNr/8)) |= (1 << (targetNr-((targetNr/8)*8)));
  }
  else
  {
    *(target+(targetNr/8)) &= ~(1 << (targetNr-((targetNr/8)*8)));
  }

} // _modbus_listBitCopy()



/**
  \fn void _modbus_intToRegister(uint16_t *inreg, uint8_t *outreg, uint8_t amount)

  \brief copy a single or multiple bytes from one array of bytes to an array of 16-bit-words

  \param  inreg      source array (16b)
  \param  outreg     target array (8b)
  \param  amount     number of words to copy

  Copy a single or multiple bytes from one array of bytes to an array of 16-bit-words

  Note: module function
*/
void _modbus_intToRegister(uint16_t *inreg, uint8_t *outreg, uint8_t amount)
{
  uint8_t   c;
  for (c = 0; c < amount; c++)
  {
      *(outreg+c*2)   = (uint8_t)(*(inreg+c) >> 8);
      *(outreg+1+c*2) = (uint8_t)(*(inreg+c));
  }

} // _modbus_intToRegister()



/**
  \fn void _modbus_registerToInt(uint8_t *inreg, uint16_t *outreg, uint8_t amount)

  \brief copy a single or multiple 16-bit-words from one array of integers to an array of bytes

  \param  inreg      source array (8b)
  \param  outreg     target array (16b)
  \param  amount     number of words to copy

  Copy a single or multiple 16-bit-words from one array of integers to an array of bytes

  Note: module function
*/
void _modbus_registerToInt(uint8_t *inreg, uint16_t *outreg, uint8_t amount)
{
  uint8_t   c;
  for (c = 0; c < amount; c++)
  {
    *(outreg+c) = (*(inreg+c*2) << 8) + *(inreg+1+c*2);
  }

} // _modbus_registerToInt()



/**
  \fn uint8_t modbus_isInRange(uint16_t addr)

  \brief check if data location addr is touched by current command

  \param  addr   address of the data object

  \return  1=location addr is touched by current command, else 0

  Check if data location addr is touched by current command

  Note: module function
*/
uint8_t _modbus_isInRange(uint16_t addr)
{
  if ((g_modbus_dataLocation <= addr) && (addr < (g_modbus_dataLocation + g_modbus_dataAmount)))
    return 1;
  return 0;

} // _modbus_isInRange()



/**
  \fn uint8_t modbus_isRangeInRange(uint16_t startAddr, uint16_t lastAddr)

  \brief check if range of data locations is touched by current command

  \param  startAddr   address of first data object in range
  \param  lastAddr    address of last data object in range

  \return  1=range is touched by current command, else 0

  Check if range of data locations is touched by current command

  Note: module function
*/
uint8_t _modbus_isRangeInRange(uint16_t startAddr, uint16_t lastAddr)
{
  if (_modbus_isInRange(startAddr) && _modbus_isInRange(lastAddr))
    return 1;
  return 0;

} //_modbus_isRangeInRange()



/**
  \fn uint8_t _modbus_crc16(uint8_t *ptrToArray,uint8_t inputSize)

  \brief Modbus 16 Bit CRC check

  \param  ptrToArray   array to check
  \param  inputSize    size of array

  \return  1=CRC check is ok, else 0

  A fairly simple Modbus compliant 16 Bit CRC algorithm. Returns 1 if the CRC
  check is ok. Else return 0 and append the calculated CRC bytes to the data array.

  Note: module function
*/
uint8_t _modbus_crc16(uint8_t *ptrToArray, uint8_t inputSize)
{
  uint16_t  out = 0xffff;
  uint16_t  carry;
  uint8_t   n, l;

  inputSize++;
  for (l = 0; l < inputSize; l++)
  {
    out ^= ptrToArray[l];
    for (n = 0; n < 8; n++)
    {
      carry = out & 1;
      out >>= 1;
      if (carry)
        out ^= 0xA001;
    } // for n
  } // for l

  // CRC16 check ok
  if ((ptrToArray[inputSize] == (out % 256)) && (ptrToArray[inputSize+1] == (out/256)))
  {
    return 1;
  }

  // CRC16 check failed
  else
  {
    ptrToArray[inputSize]   = out%256;    // append Lo
    ptrToArray[inputSize+1] = out/256;    // append Hi
    return 0;
  }

} // _modbus_crc16()



/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void modbus_init(uint32_t BR)

  \brief initialize serial communication

  \param  BR   Serial baudrate [Baud]

  initialize the UART for Modbus. For RS485 also configure transceiver
  enable pin. Call this function only once.

  Note: set correct UART and system frequency in config.h
*/
void modbus_init(uint32_t BR)
{
  uint16_t  val16;

  // for low-power device activate UART clock
  #if defined(FAMILY_STM8L)
    sfr_CLK.PCKENR1.PCKEN15 = 1;
  #endif

  // set UART behaviour
  sfr_UART.CR1.byte = UART_CR1_RESET_VALUE;  // enable UART, 8 data bits, no parity control
  sfr_UART.CR2.byte = UART_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver
  sfr_UART.CR3.byte = UART_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) FSYS_FREQ)/BR);
  sfr_UART.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);

  // enable sender & receiver
  sfr_UART.CR2.REN   = 1;   // enable receiver
  sfr_UART.CR2.TEN   = 1;   // enable sender
  sfr_UART.CR2.RIEN  = 1;   // enable Rx interrupt. TxE interrupt is enabled in modbus_sendMessage()

  // for half-duplex RS485 communication initialize transmit enable pin
  #if PHYSICAL_TYPE == 485
    TRANSCEIVER_ENABLE_PORT.DDR.byte |= TRANSCEIVER_ENABLE_PIN;    // input(=0) or output(=1)
    TRANSCEIVER_ENABLE_PORT.CR1.byte |= TRANSCEIVER_ENABLE_PIN;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    TRANSCEIVER_ENABLE_PORT.CR2.byte |= TRANSCEIVER_ENABLE_PIN;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    _modbus_transceiver_rxen();
  #endif

  // init Modbus state machine
  _modbus_busState=(1 << MB_STATE_TIMER_ACTIVE);
  _modbus_dataPos = 0;

} // modbus_init()



// this only applies to single address mode.
#ifndef MB_MULTIPLE_ADR

  /**
    \fn uint8_t modbus_getAddress(void)

    \brief read device Modbus address

    \return Modbus address

    Read device Modbus address. Only in single address mode.
  */
  uint8_t modbus_getAddress(void)
  {
    return _modbus_address;

  } // modbus_getAddress()


  /**
    \fn void modbus_setAddress(uint8_t newadr)

    \brief set device Modbus address

    \param  newadr  new device Modbus address

    Set device Modbus address. Only in single address mode.
  */
  void modbus_setAddress(uint8_t newadr)
  {
    _modbus_address = newadr;

  } // modbus_setAddress()

#endif // ! MB_MULTIPLE_ADR



/**
  \fn void modbus_sendMessage(uint8_t packtop)

  \brief send a response

  \param  packtop   position of the last byte containing data

  Send a Modbus response. Function modbus_sendException() is a good usage example.
*/
void modbus_sendMessage(uint8_t packtop)
{
  _modbus_packetTopIndex = packtop+2;
  _modbus_crc16((uint8_t*) g_modbus_buffer, packtop);
  _modbus_dataPos = 0;

  // for RS485 enable transmission
  #if PHYSICAL_TYPE == 485
    _modbus_transceiver_txen();
  #endif

  // set new Modbus state machine state
  _modbus_busState &= ~(1 << MB_STATE_RECEIVE_COMPLETED);
  _modbus_busState |= (1 << MB_STATE_TRANSMIT_REQUESTED);

  // enable TxE interrupt
  sfr_UART.CR2.TIEN = 1;

} // modbus_sendMessage()



/**
  \fn void modbus_sendException(uint8_t exceptionCode)

  \brief send an exception response

  \param  exceptionCode   exception code to send.

  Send an exception response.
*/
void modbus_sendException(uint8_t exceptionCode)
{
  // construct data
  g_modbus_buffer[1] |= (1 << 7);      // setting MSB of the function code (the exception flag)
  g_modbus_buffer[2] = exceptionCode;  // Exceptioncode. Also the last byte containing data

  // send Modbus response
  modbus_sendMessage(2);

} // modbus_sendException()



/**
  \fn void modbus_reset(void)

  \brief revert to receiving state

  Revert to receiving state.
*/
void modbus_reset(void)
{
  // stop receiving (error)
  _modbus_busState = (1 << MB_STATE_TIMER_ACTIVE);
  _modbus_timer    = 0;

} // modbus_reset()



/**
  \fn uint8_t modbus_getBusState(void)

  \brief get Modbus state

  \return state of Modbus state machine

  Get state of Modbus state machine
*/
uint8_t modbus_getBusState(void)
{
  return _modbus_busState;

} // modbus_getBusState()



/**
  \fn uint16_t modbus_requestedAmount(void)

  \brief get number of bits/registers requested

  \return number of bits/registers requested

  Get the amount of requested data objects (coils, discretes, registers)
*/
uint16_t modbus_requestedAmount(void)
{
  return g_modbus_dataAmount;

} // modbus_requestedAmount()



/**
  \fn uint16_t modbus_requestedAddress(void)

  \brief get address of the first requested data object

  \return address of the first requested data object

  Get the address of the first requested data object (coils, discretes, registers)
*/
uint16_t modbus_requestedAddress(void)
{
  return g_modbus_dataLocation;

} // modbus_requestedAddress()



/**
  \fn uint8_t modbus_exchangeBits(uint8_t *ptrToInArray, uint16_t startAddress, uint16_t size)

  \brief handle single/multiple input/coil reading and single/multiple coil writing

  \param  ptrToInArray    pointer to the user's data array containing bits
  \param  startAddress    address of the first bit in the supplied array
  \param  size            input array size in the requested format (bits)

  \return 1=ok, 0=fail

  Handle single/multiple input/coil reading and single/multiple coil writing.
*/
uint8_t modbus_exchangeBits(uint8_t *ptrToInArray, uint16_t startAddress, uint16_t size)
{
  uint16_t c;

  // check address range
  if ((g_modbus_dataLocation >= startAddress) && ((startAddress + size) >= (g_modbus_dataAmount + g_modbus_dataLocation)))
  {

    if ((g_modbus_buffer[1] == MB_FC_READ_DISCRETE_INPUT) || (g_modbus_buffer[1] == MB_FC_READ_COILS))
    {
      // if message buffer large enough
      if (g_modbus_dataAmount <= ((MB_MAX_FRAME_INDEX-4) * 8))
      {
        g_modbus_buffer[2] = (g_modbus_dataAmount/8);
        if ((g_modbus_dataAmount%8) > 0)
        {
          // fill last data byte with zeros
          g_modbus_buffer[(uint8_t)(g_modbus_dataAmount/8)+3] = 0x00;
          g_modbus_buffer[2]++;
        }
        for (c = 0; c < g_modbus_dataAmount; c++)
        {
          _modbus_listBitCopy(ptrToInArray, g_modbus_dataLocation-startAddress+c, (uint8_t*) (g_modbus_buffer+3), c);
        }
        if (g_modbus_buffer[0] != 0)
          modbus_sendMessage(g_modbus_buffer[2]+2);           // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
        return 1;
      }

      // too many bits requested within single request
      else
      {
        if (g_modbus_buffer[0] != 0)
          modbus_sendException(MB_EC_ILLEGAL_DATA_VALUE);     // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
      }

    } // MB_FC_READ_DISCRETE_INPUT || MB_FC_READ_COILS


    else if (g_modbus_buffer[1] == MB_FC_WRITE_MULTIPLE_COILS)
    {
      // if enough data received
      if (((g_modbus_buffer[6]*8) >= g_modbus_dataAmount) && ((_modbus_dataPos-9) >= g_modbus_buffer[6]))
      {
        for (c = 0; c < g_modbus_dataAmount; c++)
        {
          _modbus_listBitCopy((uint8_t*) (g_modbus_buffer+7), c, ptrToInArray, g_modbus_dataLocation-startAddress+c);
        }
        if (g_modbus_buffer[0] != 0)
          modbus_sendMessage(5);                              // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
        return 1;
      }

      // exception too few data bytes received
      else
      {
        if (g_modbus_buffer[0] != 0)
          modbus_sendException(MB_EC_ILLEGAL_DATA_VALUE);     // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
      }

    } // MB_FC_WRITE_MULTIPLE_COILS


    else if (g_modbus_buffer[1] == MB_FC_WRITE_COIL)
    {
      _modbus_listBitCopy((uint8_t*) (g_modbus_buffer+4),0,ptrToInArray,g_modbus_dataLocation-startAddress);
      if (g_modbus_buffer[0] != 0)
        modbus_sendMessage(5);                                // unicast -> send response
      else
        modbus_reset();                                       // broadcast -> skip response
      return 1;

    } // MB_FC_WRITE_COIL

    //modbus_sendException(MB_EC_SLAVE_DEVICE_FAILURE); //inanpropriate call of modbus_exchangeBits
    return 0;

  } // range check


  // address out of range
  else
  {
    if (g_modbus_buffer[0] != 0)
      modbus_sendException(MB_EC_ILLEGAL_DATA_VALUE);         // unicast -> send response
    else
      modbus_reset();                                         // broadcast -> skip response
    return 0;
  }

} // modbus_exchangeBits()



/**
  \fn uint8_t modbus_exchangeRegisters(uint16_t *ptrToInArray, uint16_t startAddress, uint16_t size)

  \brief handle single/multiple register reading and single/multiple register writing

  \param  ptrToInArray    pointer to the user's data array containing registers
  \param  startAddress    address of the first register in the supplied array
  \param  size            input array size in the requested format (16bit-registers)

  \return 1=ok, 0=fail

  Handle single/multiple register reading and single/multiple register writing.
*/
uint8_t modbus_exchangeRegisters(uint16_t *ptrToInArray, uint16_t startAddress, uint16_t size)
{
  // check address range
  if ((g_modbus_dataLocation >= startAddress) && ((startAddress+size) >= (g_modbus_dataAmount + g_modbus_dataLocation)))
  {
    if ((g_modbus_buffer[1] == MB_FC_READ_REGISTERS) || (g_modbus_buffer[1] == MB_FC_READ_INPUT_REGISTER) )
    {
      // if message buffer large enough
      if ((g_modbus_dataAmount*2) <= (MB_MAX_FRAME_INDEX-4))
      {
        g_modbus_buffer[2] = (uint8_t)(g_modbus_dataAmount*2);
        _modbus_intToRegister(ptrToInArray+(g_modbus_dataLocation-startAddress), (uint8_t*) (g_modbus_buffer+3), g_modbus_dataAmount);
        if (g_modbus_buffer[0] != 0)
          modbus_sendMessage(2+g_modbus_buffer[2]);           // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
        return 1;
      }

      // message buffer not large enough
      else
      {
        if (g_modbus_buffer[0] != 0)
          modbus_sendException(MB_EC_ILLEGAL_DATA_VALUE);     // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
      }

    } // MB_FC_READ_REGISTERS || MB_FC_READ_INPUT_REGISTER


    else if (g_modbus_buffer[1] == MB_FC_WRITE_MULTIPLE_REGISTERS)
    {
      // if enough data received
      if (((g_modbus_buffer[6]) >= g_modbus_dataAmount*2) && ((_modbus_dataPos-9) >= g_modbus_buffer[6]))
      {
        _modbus_registerToInt((uint8_t*) (g_modbus_buffer+7), ptrToInArray+(g_modbus_dataLocation-startAddress), (uint8_t)(g_modbus_dataAmount));
        if (g_modbus_buffer[0] != 0)
          modbus_sendMessage(5);                              // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
        return 1;
      }

      // exception too few data bytes received
      else
      {
        if (g_modbus_buffer[0] != 0)
          modbus_sendException(MB_EC_ILLEGAL_DATA_VALUE);     // unicast -> send response
        else
          modbus_reset();                                     // broadcast -> skip response
      }

    } // MB_FC_WRITE_MULTIPLE_REGISTERS


    else if (g_modbus_buffer[1] == MB_FC_WRITE_REGISTER)
    {
      _modbus_registerToInt((uint8_t*) (g_modbus_buffer+4), ptrToInArray+(g_modbus_dataLocation-startAddress), 1);
      if (g_modbus_buffer[0] != 0)
        modbus_sendMessage(5);                                // unicast -> send response
      else
        modbus_reset();                                       // broadcast -> skip response
      return 1;

    } // MB_FC_WRITE_REGISTER

    //modbus_sendException(MB_EC_SLAVE_DEVICE_FAILURE); //inapropriate call of modbus_exchangeRegisters
    return 0;

  } // range check

  // address out of range
  else
  {
    if (g_modbus_buffer[0] != 0)
      modbus_sendException(MB_EC_ILLEGAL_DATA_VALUE);         // unicast -> send response
    else
      modbus_reset();                                         // broadcast -> skip response
    return 0;
  }

} // modbus_exchangeRegisters()



/**
  \fn void UART_RXNE_ISR(void)

  \brief ISR for UART RXNE (receive)

  Interrupt service routine for UART receive (RXNE).

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART_RXNE_ISR, _UART_R_RXNE_VECTOR_)
{
  uint8_t data;

  // clear pending ISR flag
  sfr_UART.SR.RXNE = 0;

  // copy received byte
  data = sfr_UART.DR.byte;

  // reset Modbus timeout timer
  _modbus_timer = 0;

  // if Modbus state machine is in state MB_STATE_RECEIVING, continue reception of frame
  if (!(_modbus_busState & (1 << MB_STATE_RECEIVE_COMPLETED)) && !(_modbus_busState & (1 << MB_STATE_TRANSMIT_REQUESTED)) && \
      !(_modbus_busState & (1 << MB_STATE_TRANSMITTING))      && (_modbus_busState & (1 << MB_STATE_RECEIVING)) && \
      !(_modbus_busState & (1 << MB_STATE_BUS_TIMEOUT)))
  {
    // receive pointer exceeds buffer size
    if (_modbus_dataPos > MB_MAX_FRAME_INDEX)
      modbus_reset();

    // copy data to Modbus buffer
    else
    {
      g_modbus_buffer[_modbus_dataPos] = data;
      _modbus_dataPos++; //TODO: maybe prevent this from exceeding 255?
    }

  } // MB_STATE_RECEIVING


  // if Modbus state machine is in state MB_STATE_BUS_TIMEOUT, start reception of new frame
  else if (!(_modbus_busState & (1 << MB_STATE_RECEIVE_COMPLETED)) && !(_modbus_busState & (1 << MB_STATE_TRANSMIT_REQUESTED)) && \
           !(_modbus_busState & (1 << MB_STATE_TRANSMITTING))      && !(_modbus_busState & (1 << MB_STATE_RECEIVING)) && \
           (_modbus_busState & (1 << MB_STATE_BUS_TIMEOUT)))
  {
     // copy data to Modbus buffer
     g_modbus_buffer[0] = data;
     _modbus_dataPos = 1;

     // set new Modbus state
     _modbus_busState = ((1 << MB_STATE_RECEIVING) | (1 << MB_STATE_TIMER_ACTIVE));

  } // MB_STATE_BUS_TIMEOUT

} // UART_RXNE_ISR()



/**
  \fn void UART_RXNE_ISR(void)

  \brief shared ISR for UART TXE and TXC

  Shared interrupt service routine for UART transmit buffer empty (TXE)
  and transmission complete (TC).

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART_TXE_ISR, _UART_T_TXE_VECTOR_)
{
  // send next byte in buffer
  if ((sfr_UART.CR2.TIEN == 1) && (sfr_UART.SR.TXE))
  {
    // set new Modbus state
    _modbus_busState &= ~(1 << MB_STATE_TRANSMIT_REQUESTED);
    _modbus_busState |= (1 << MB_STATE_TRANSMITTING);

    // send byte
    sfr_UART.DR.byte = g_modbus_buffer[_modbus_dataPos];

    // increase data pointer. If finished, disable TXE and enable TC interrupt
    _modbus_dataPos++;
    if (_modbus_dataPos == (_modbus_packetTopIndex+1))
    {
      sfr_UART.CR2.TIEN  = 0;   // disable this TXE interrupt
      sfr_UART.SR.TC     = 0;   // clear pending TC flag
      sfr_UART.CR2.TCIEN = 1;   // enable TC interrupt
    }

    // clear TXE flag
    sfr_UART.SR.TXE = 0;

  } // TXE interrupt


  // transmission of last byte is complete, clean up
  if ((sfr_UART.CR2.TCIEN == 1) && (sfr_UART.SR.TC))
  {
    // for RS495, enable reception
    #if (PHYSICAL_TYPE == 485)
      _modbus_transceiver_rxen();
    #endif

    // reset Modbus state
    modbus_reset();

    // disable this TC interrupt
    sfr_UART.CR2.TCIEN = 0;

    // clear TC flag
    sfr_UART.SR.TC = 0;

  } // TC interrupt

} // UART_TXE_ISR()



/**
  \fn void modbus_tickTimer(void)

  \brief low-level Modbus handler

  Low-level Modbus protocol handler. Is called from within 1ms timer ISR.

  Note: 1ms is not fully Modbus compliant, but reduces interrupt load.
  If changed back to 100us, also revert Modbus timeouts in file yaMBSistm8.h
*/
void modbus_tickTimer(void)
{
  // Modbus state MB_STATE_TIMER_ACTIVE
  if (_modbus_busState & (1 << MB_STATE_TIMER_ACTIVE))
  {
    _modbus_timer++;

    // Modbus state MB_STATE_RECEIVING
    if (_modbus_busState & (1 << MB_STATE_RECEIVING))
    {
      // character timeout detected
      if ((_modbus_timer == MB_INTER_CHAR_TIMEOUT))
      {
        _modbus_busState |= (1 << MB_STATE_GAP_DETECTED);
      }

      // no character timeout
      else
      {
        // frame timeout detected -> end of message
        if ((_modbus_timer == MB_INTER_FRAME_DELAY_RECEIVE_END))
        {
          // only for multiple/all address mode
          #ifdef MB_MULTIPLE_ADR

            // CRC16 check ok
            if (_modbus_crc16(g_modbus_buffer,_modbus_dataPos-3))
            {
              _modbus_saveLocation();
              _modbus_busState = (1 << MB_STATE_RECEIVE_COMPLETED);
            }

            // CRC16 check failed
            else
              modbus_reset();

          #endif // MB_MULTIPLE_ADR

          // only for single address mode
          #ifndef MB_MULTIPLE_ADR

            // if message ID matches or is broadcast (ID=0), and CRC16 is ok
            if (((g_modbus_buffer[0] == _modbus_address) || (g_modbus_buffer[0] == 0x00)) && (_modbus_crc16((uint8_t*) g_modbus_buffer, _modbus_dataPos-3)))
            {
              _modbus_saveLocation();
              _modbus_busState = (1 << MB_STATE_RECEIVE_COMPLETED);
            }

            // message not for us or CRC16 check failed
            else
              modbus_reset();

          #endif // ! MB_MULTIPLE_ADR

        } // end of message

      } // no character timeout

    } // MB_STATE_RECEIVING

    // not yet in MB_STATE_RECEIVING
    else
    {
      // inter-frame timeout detected
      if (_modbus_timer == MB_INTER_FRAME_DELAY_RECEIVE_START)
        _modbus_busState |= (1 << MB_STATE_BUS_TIMEOUT);
    }

  } // MB_STATE_TIMER_ACTIVE

} // modbus_tickTimer()

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
