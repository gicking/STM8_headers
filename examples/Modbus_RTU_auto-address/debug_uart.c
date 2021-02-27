/**
  \file debug_uart.c

  \author G. Icking-Konert
  \date 2020-05-24
  \version 0.1

  \brief implementation of debug UART functions/macros using FIFO and interrupts

  implementation of debug UART functions and macros using FIFO and interrupts
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"

// only if debug UART is used
#ifdef sfr_UART_DEBUG

#include "debug_uart.h"




/*-----------------------------------------------------------------------------
    MODULE VARIABLES (for clarity module internal variables start with "m_")
-----------------------------------------------------------------------------*/

/// reserve debug UART receive FIFO buffer
fifo_t  m_Rx_Fifo = { {0}, 0, 0, 0, 0 };

/// reserve debug UART transmit FIFO buffer
fifo_t  m_Tx_Fifo = { {0}, 0, 0, 0, 0 };


/**
  \fn int putchar(int byte)

  \brief output routine for printf()

  \param[in]  data   byte to send

  \return  sent byte

  implementation of putchar() for printf(), using selected output channel.
  Use send routine set via putchar_attach()
  Return type depends on used compiler (see respective stdio.h)
*/
#if defined(__CSMC__)
  char putchar(char data) {
#else // Standard C
  int putchar(int data) {
#endif

  // queue byte in Tx FIFO
  UART_debug_send_byte(data);

  // return sent byte
  return(data);

} // putchar



/**
  \fn void UART_debug_begin(uint32_t BR)

  \brief initialize debug UART for interrupt based communication

  \param[in]  BR    baudrate [Baud]

  initialize debug UART for interrupt based communication with specified baudrate.
  Use 1 start, 8 data and 1 stop bit; no parity or flow control.
*/
void UART_debug_begin(uint32_t BR) {

  uint16_t  val16;

  // set debug UART behaviour
  sfr_UART_DEBUG.CR1.byte = UART_CR1_RESET_VALUE;  // enable UART, 8 data bits, no parity control
  sfr_UART_DEBUG.CR2.byte = UART_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver
  sfr_UART_DEBUG.CR3.byte = UART_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) FSYS_FREQ)/BR);
  sfr_UART_DEBUG.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART_DEBUG.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);

  // enable transmission
  sfr_UART_DEBUG.CR2.REN  = 1;  // enable receiver
  sfr_UART_DEBUG.CR2.TEN  = 1;  // enable sender

  // init FIFOs for receive and transmit
  fifo_init(&m_Rx_Fifo);
  fifo_init(&m_Tx_Fifo);

  // enable Rx interrupt. Tx interrupt is enabled in UART_debug_send*()
  sfr_UART_DEBUG.CR2.RIEN = 1;

} // UART_debug_begin



/**
  \fn void UART_debug_send_byte(uint8_t data)

  \brief send byte via debug UART

  \param[in]  byte   data to send

  send byte via debug UART.

  Use FIFO for sending:
   - store data into the Tx FIFO buffer
   - enable "Tx buffer empty" interrupt
   - actual transmission is handled by TXE ISR
*/
void  UART_debug_send_byte(uint8_t data) {

  // wait until FIFO has free space
  while(FIFO_FULL(m_Tx_Fifo));

  // disable "Tx empty interrupt" while manipulating Tx FIFO
  sfr_UART_DEBUG.CR2.TIEN = 0;

  // store byte in software FIFO
  fifo_enqueue(&m_Tx_Fifo, data);

  // enable "Tx empty interrupt" to resume sending data
  sfr_UART_DEBUG.CR2.TIEN = 1;

} // UART_debug_send_byte



/**
  \fn void UART_debug_send_buf(uint16_t num, uint8_t *data)

  \brief send array of bytes via debug UART

  \param[in]  num    buf size in bytes
  \param[in]  data   bytes to send

  send array of bytes via debug UART.

  Use FIFO for sending:
   - stores data into the Tx FIFO software buffer
   - enable the "Tx buffer empty" interrupt
   - actual transmission is handled by TXE ISR
*/
void UART_debug_send_buf(uint16_t num, uint8_t *data) {

  uint16_t i;

  // wait until FIFO has enough free space
  while((FIFO_BUFFER_SIZE - m_Tx_Fifo.numBytes) < num);

  // disable "Tx empty interrupt" while manipulating Tx FIFO
  sfr_UART_DEBUG.CR2.TIEN = 0;

  // store bytes in software FIFO
  for (i=0; i<num; i++)
    fifo_enqueue(&m_Tx_Fifo, data[i]);

  // enable "Tx empty interrupt" to resume sending data
  sfr_UART_DEBUG.CR2.TIEN = 1;

} // UART_debug_send_buf



/**
  \fn uint8_t UART_debug_check_Rx(void)

  \brief check if data was received via debug UART

  \return  1 = data in FIFO

  check if receive FIFO buffer contains data
*/
uint8_t UART_debug_check_Rx(void) {

  uint8_t   result;

  // check if FIFO contains data
  result = FIFO_NOT_EMPTY(m_Rx_Fifo);

  // return FIFO status
  return(result);

} // UART_debug_check_Rx



/**
  \fn uint8_t UART_debug_receive(void)

  \brief read data from debug UART receive FIFO

  \return  oldest, not treated data

  Read data from receive FIFO:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - remove oldest element from FIFO
*/
uint8_t UART_debug_receive(void) {

  uint8_t   data;

  // disable "Rx full interrupt" while manipulating Rx FIFO
  sfr_UART_DEBUG.CR2.RIEN = 0;

  // get oldest FIFO element (or -128 if FIFO is empty)
  data = fifo_dequeue(&m_Rx_Fifo);

  // re-enable "Rx full interrupt"
  sfr_UART_DEBUG.CR2.RIEN = 1;

  // return the FIFO data
  return(data);

} // UART_debug_receive



/**
  \fn uint8_t UART_debug_peek(void)

  \brief peek next received byte from debug UART (keep data in FIFO)

  \return  oldest, not treated data

  Peek data in receive FIFO:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - Rx FIFO is not altered
*/
uint8_t UART_debug_peek(void) {

  uint8_t   data=0;

  // disable "Rx full interrupt" while manipulating Rx FIFO
  sfr_UART_DEBUG.CR2.RIEN = 0;

  // get oldest FIFO element (or -128 if FIFO is empty)
  data = fifo_peek(&m_Rx_Fifo);

  // re-enable "Rx full interrupt"
  sfr_UART_DEBUG.CR2.RIEN = 1;

  // return the FIFO data
  return(data);

} // UART_debug_peek



/**
  \fn void UART_DEBUG_RXNE_ISR(void)

  \brief ISR for debug UART receive

  interrupt service routine for debug UART receive

  Actions:
    - called if data received via debug UART
    - copy data from HW buffer to FIFO

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART_DEBUG_RXNE_ISR, _UART_DEBUG_R_RXNE_VECTOR_)
{
  uint8_t   data;

  // clearing of ISR flag not required for STM8
  sfr_UART_DEBUG.SR.RXNE = 0;

  // read byte from UART buffer
  data = sfr_UART_DEBUG.DR.byte;

  // add a new byte to the FIFO buffer
  fifo_enqueue(&m_Rx_Fifo, data);

  return;

} // UART_DEBUG_RXNE_ISR



/**
  \fn void UART_DEBUG_TXE_ISR(void)

  \brief ISR for debug UART transmit

  Actions:
    - called if Tx HW buffer is empty
    - checks if FIFO constains data
    - if yes, move oldest element from FIFO to Tx buffer
    - if FIFO is empty, disable this interrupt

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART_DEBUG_TXE_ISR, _UART_DEBUG_T_TXE_VECTOR_)
{
  uint8_t   data;

  // clearing of ISR flag not required for STM8
  sfr_UART_DEBUG.SR.TXE = 0;

  // if Tx FIFO contains data, get oldest element and send it
  if (FIFO_NOT_EMPTY(m_Tx_Fifo)) {

    // get Tx byte from FIFO
    data = fifo_dequeue(&m_Tx_Fifo);

    // send byte
    sfr_UART_DEBUG.DR.byte = data;

  } // Tx FIFO not empty

  // if FIFO is now empty, deactivate this interrupt
  if (!(FIFO_NOT_EMPTY(m_Tx_Fifo)))
    sfr_UART_DEBUG.CR2.TIEN = 0;

  return;

} // UART_DEBUG_TXE_ISR

// only if debug UART is used
#endif // sfr_UART_DEBUG

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
