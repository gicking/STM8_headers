/**
  \file uart1.c

  \author G. Icking-Konert
  \date 2020-05-24
  \version 0.1

  \brief implementation of UART1 functions/macros using FIFO and interrupts

  implementation of UART1 functions and macros using FIFO and interrupts
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdio.h>
#include "uart1.h"


/*-----------------------------------------------------------------------------
    MODULE VARIABLES (for clarity module internal variables start with "m_")
-----------------------------------------------------------------------------*/

/// reserve UART1 receive FIFO buffer
volatile fifo_t  m_Rx_Fifo = { {0}, 0, 0, 0, 0 };

/// reserve UART1 transmit FIFO buffer
volatile fifo_t  m_Tx_Fifo = { {0}, 0, 0, 0, 0 };


/**
  \fn void UART1_begin(uint32_t BR)

  \brief initialize UART1 for interrupt based communication

  \param[in]  BR    baudrate [Baud]

  initialize UART1 for interrupt based communication with specified baudrate.
  Use 1 start, 8 data and 1 stop bit; no parity or flow control.
*/
void UART1_begin(uint32_t BR) {

  uint16_t  val16;

  // set UART1 behaviour
  sfr_UART1.CR1.byte = sfr_UART1_CR1_RESET_VALUE;  // enable UART1, 8 data bits, no parity control
  sfr_UART1.CR2.byte = sfr_UART1_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver
  sfr_UART1.CR3.byte = sfr_UART1_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
  sfr_UART1.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART1.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);

  // enable transmission
  sfr_UART1.CR2.REN  = 1;  // enable receiver
  sfr_UART1.CR2.TEN  = 1;  // enable sender

  // init FIFOs for receive and transmit
  fifo_init((fifo_t*) (&m_Rx_Fifo));
  fifo_init((fifo_t*) (&m_Tx_Fifo));

  // enable Rx interrupt. Tx interrupt is enabled in uart1_send()
  sfr_UART1.CR2.RIEN = 1;

} // UART1_begin



/**
  \fn void UART1_send_byte(uint8_t data)

  \brief send byte via UART1

  \param[in]  byte   data to send

  send byte via UART1.

  Use FIFO for sending:
   - store data into the Tx FIFO buffer
   - enable "Tx buffer empty" interrupt
   - actual transmission is handled by TXE ISR
*/
void  UART1_send_byte(uint8_t data) {

  // wait until FIFO has free space
  while(FIFO_FULL(m_Tx_Fifo));

  // disable "Tx empty interrupt" while manipulating Tx FIFO
  sfr_UART1.CR2.TIEN = 0;

  // store byte in software FIFO
  fifo_enqueue((fifo_t*) (&m_Tx_Fifo), data);

  // enable "Tx empty interrupt" to resume sending data
  sfr_UART1.CR2.TIEN = 1;

} // UART1_send_byte



/**
  \fn void UART1_send_buf(uint16_t num, uint8_t *data)

  \brief send array of bytes via UART1

  \param[in]  num    buf size in bytes
  \param[in]  data   bytes to send

  send array of bytes via UART1.

  Use FIFO for sending:
   - stores data into the Tx FIFO software buffer
   - enable the "Tx buffer empty" interrupt
   - actual transmission is handled by TXE ISR
*/
void UART1_send_buf(uint16_t num, uint8_t *data) {

  uint16_t i;

  // wait until FIFO has enough free space
  while((FIFO_BUFFER_SIZE - m_Tx_Fifo.numBytes) < num);

  // disable "Tx empty interrupt" while manipulating Tx FIFO
  sfr_UART1.CR2.TIEN = 0;

  // store bytes in software FIFO
  for (i=0; i<num; i++)
    fifo_enqueue((fifo_t*) (&m_Tx_Fifo), data[i]);

  // enable "Tx empty interrupt" to resume sending data
  sfr_UART1.CR2.TIEN = 1;

} // UART1_send_buf



/**
  \fn uint8_t UART1_check_Rx(void)

  \brief check if data was received via UART1

  \return  1 = data in FIFO

  check if receive FIFO buffer contains data
*/
uint8_t UART1_check_Rx(void) {

  uint8_t   result;

  // check if FIFO contains data
  result = FIFO_NOT_EMPTY(m_Rx_Fifo);

  // return FIFO status
  return(result);

} // UART1_check_Rx



/**
  \fn uint8_t UART1_receive(void)

  \brief read data from UART1 receive FIFO

  \return  oldest, not treated data

  Read data from receive FIFO:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - remove oldest element from FIFO
*/
uint8_t UART1_receive(void) {

  uint8_t   data;

  // disable "Rx full interrupt" while manipulating Rx FIFO
  sfr_UART1.CR2.RIEN = 0;

  // get oldest FIFO element (or -128 if FIFO is empty)
  data = fifo_dequeue((fifo_t*) (&m_Rx_Fifo));

  // re-enable "Rx full interrupt"
  sfr_UART1.CR2.RIEN = 1;

  // return the FIFO data
  return(data);

} // UART1_receive



/**
  \fn uint8_t UART1_peek(void)

  \brief peek next received byte from UART1 (keep data in FIFO)

  \return  oldest, not treated data

  Peek data in receive FIFO:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - Rx FIFO is not altered
*/
uint8_t UART1_peek(void) {

  uint8_t   data=0;

  // disable "Rx full interrupt" while manipulating Rx FIFO
  sfr_UART1.CR2.RIEN = 0;

  // get oldest FIFO element (or -128 if FIFO is empty)
  data = fifo_peek((fifo_t*) (&m_Rx_Fifo));

  // re-enable "Rx full interrupt"
  sfr_UART1.CR2.RIEN = 1;

  // return the FIFO data
  return(data);

} // UART1_peek



/**
  \fn void UART1_RXNE_ISR(void)

  \brief ISR for UART1 receive

  interrupt service routine for UART1 receive

  Actions:
    - called if data received via UART1
    - copy data from HW buffer to FIFO

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART1_RXNE_ISR, _UART1_R_RXNE_VECTOR_)
{
  uint8_t   data;

  // clearing of ISR flag not required for STM8
  sfr_UART1.SR.RXNE = 0;

  // read byte from UART buffer
  data = sfr_UART1.DR.byte;

  // add a new byte to the FIFO buffer
  fifo_enqueue((fifo_t*) (&m_Rx_Fifo), data);

  return;

} // UART1_RXNE_ISR



/**
  \fn void UART1_TXE_ISR(void)

  \brief ISR for UART1 transmit

  Actions:
    - called if Tx HW buffer is empty
    - checks if FIFO constains data
    - if yes, move oldest element from FIFO to Tx buffer
    - if FIFO is empty, disable this interrupt

  Note:
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(UART1_TXE_ISR, _UART1_T_TXE_VECTOR_)
{
  uint8_t   data;

  // clearing of ISR flag not required for STM8
  sfr_UART1.SR.TXE = 0;

  // if Tx FIFO contains data, get oldest element and send it
  if (FIFO_NOT_EMPTY(m_Tx_Fifo)) {

    // get Tx byte from FIFO
    data = fifo_dequeue((fifo_t*) (&m_Tx_Fifo));

    // send byte
    sfr_UART1.DR.byte = data;

  } // Tx FIFO not empty

  // if FIFO is now empty, deactivate this interrupt
  if (!(FIFO_NOT_EMPTY(m_Tx_Fifo)))
    sfr_UART1.CR2.TIEN = 0;

  return;

} // UART1_TXE_ISR

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
