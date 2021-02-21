/*
 *  Created: 21.02.2021
 *  Author: Max Brueggemann (original AVR implementation), Georg Icking-Konert (STM8 port)
 *
 *  An example project implementing a simple modbus RTU client device using an
 *  STM8S or STM8L running at 16MHz.
 *
 *  Baudrate: 115.2kBaud, 8 data bits, 1 stop bit, no parity, optionally RS485
 *
 *	This code is going to:
 *	1. read holding registers 0 to 3 from client device 1
 *	2. wait 1 second
 *	3. write the value x to register 0 at client device 1
 *	4. increment x
 *	5. wait 1 second
 *	and then start again at 1.
 *
 *  Note: yaMBSistm8 has been solely written to be a modbus server library.
 *	  While it is possible to use it as a modbus master, it cannot be
 *	  done without going a little deeper into the modbus protocol.
 *	  This example shall serve as a guideline for those who want to do
 *	  it anyway :-)
*/

 /*----------------------------------------------------------
     INCLUDE FILES
 ----------------------------------------------------------*/

#include "config.h"
#define _MAIN_          // required for global variables
  #include "yaMBSistm8.h"
  #include "timer4.h"
#undef _MAIN_


/*----------------------------------------------------------
    MACROS/DEFINES
----------------------------------------------------------*/

/// number of registers to read or write
#define  NUM_REGS   8

/// check for command completed
#define receiveOkay ( modbus_getBusState() & (1 << MB_STATE_RECEIVE_COMPLETED) )


/*----------------------------------------------------------
    VARIABLES
----------------------------------------------------------*/

uint16_t holdingRegisters[NUM_REGS];


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

void read_registers(uint8_t slaveid, uint16_t address, uint8_t amount)
{
  // assert inter-frame pause
  delay(2);

  // construct frame
  modbus_setAddress(slaveid);
  g_modbus_buffer[0] = slaveid;
  g_modbus_buffer[1] = 0x03;
  g_modbus_buffer[2] = (address >> 8) & 0xFF;
  g_modbus_buffer[3] = address & 0xFF;
  g_modbus_buffer[4] = 0x00;
  g_modbus_buffer[5] = amount;

  // send frame
  modbus_sendMessage(5);

} // read_registers()


void write_register(uint8_t slaveid, uint16_t address, uint16_t value)
{
  // assert inter-frame pause
  delay(2);

  // construct frame
  modbus_setAddress(slaveid);
  g_modbus_buffer[0] = slaveid;
  g_modbus_buffer[1] = 0x06;
  g_modbus_buffer[2] = (address >> 8) & 0xFF;
  g_modbus_buffer[3] = address & 0xFF;
  g_modbus_buffer[4] = (value >> 8) & 0xFF;
  g_modbus_buffer[5] = value & 0xFF;

  // send frame
  modbus_sendMessage(5);

} // write_register()



/**
  \fn void main(void)

  \brief main routine

  Main routine. Initialize controller and handle Modbus communication
*/
int main(void)
{
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;

  // set to dummy address
  modbus_setAddress(1);

  // configure UART. In case of RS485 also initialize EN pin
  modbus_init(115200L);

  // init 1ms interrupt for SW clock and Modbus low-level handler
  TIM4_init();

  // enable interrupts
  ENABLE_INTERRUPTS();


  // main loop
  while(1)
  {
    // wait 1s
    delay(1000);

    ///////
    // read 4 registers
    ///////
    read_registers(1, 0, 4);

    // wait for client response, time out after 1s
    uint8_t breaker = 100;
    while(!receiveOkay && breaker)
    {
      breaker--;
      delay(10);
    }

    // if this fails, there was either no response or a crc error
    if (receiveOkay)
    {
      // client responded with an error code
      if(g_modbus_buffer[1] & 0x80)
      {
        // handle the error
      }

      // no error responsed
      else
      {
        // g_modbus_buffer[2] should be 8 (4 registers => 8 bytes). You might want to check this at this point.
        for(uint8_t x = 0; x < 4; x++)
        {
          holdingRegisters[x] = (g_modbus_buffer[3+x*2] << 8) + g_modbus_buffer[4+x*2];
          // do sth with the acquired data.
        }

      } // no error responsed

    } // receiveOkay


    // wait 1s
    delay(1000);

    ///////
    // write 1 register
    ///////
    static uint16_t incr = 0;
    incr++;
    write_register(1, 0, incr);

    // wait for client response, time out after 1s
    breaker = 100;
    while(!receiveOkay && breaker)
    {
      breaker--;
      delay(10);
    }

    // if this fails, there was either no response or a crc error
    if(receiveOkay)
    {
      //client responded with an error code
      if (g_modbus_buffer[1] & 0x80)
      {
        // handle the error
      }

    } // receiveOkay

  } // main loop

} // main()

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
