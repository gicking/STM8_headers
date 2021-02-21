/*
 *  Created: 21.02.2021
 *  Author: Max Brueggemann (original AVR implementation), Georg Icking-Konert (STM8 port)
 *
 *  An example project implementing a simple modbus RTU client device using an
 *  STM8S or STM8L running at 16MHz.
 *
 *  Baudrate: 115.2kBaud, 8 data bits, 1 stop bit, no parity, optionally RS485
 *
 *  Your busmaster can read/write the following data:
 *  coils: 0..7
 *  discrete inputs: 0..7
 *  input registers: 0..NUM_REGS
 *  holding registers: 0..NUM_REGS
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


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/

/**
  \fn void setOutputs(uint8_t states)

  \brief set digital outputs

  \param  states containing up to 8 digital output states

  Set up to 8 digital outputs. Is project specific.
*/
void setOutputs(uint8_t states)
{
  if (states)
    LED_PORT.ODR.byte |= LED_PIN;
  else
    LED_PORT.ODR.byte &= ~LED_PIN;

} // setOutputs()



/**
  \fn uint8_t readInputs(void)

  \brief read digital inputs

  \return byte containing up to 8 digital input states

  Read up to 8 digital inputs. Is project specific.
*/
uint8_t readInputs(void)
{
  uint8_t ins = 0x00;

  return ins;

} // readInputs()



/**
  \fn void io_init(void)

  \brief initialize GPIOs

  initialize GPIOs. Is project specific.
*/
void io_init(void)
{
  // configure LED pin as output
  LED_PORT.DDR.byte |= LED_PIN;    // input(=0) or output(=1)
  LED_PORT.CR1.byte |= LED_PIN;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  LED_PORT.CR2.byte |= LED_PIN;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  LED_PORT.ODR.byte |= LED_PIN;    // switch off LEDs

} // io_init()



/**
  \fn void modbus_handler(void)

  \brief execute Modbus commands

  Execute Modbus command. Should be called often to assert no command is lost.
  It is proposed not to call this function from an ISR.
*/
void modbus_handler(void)
{
  uint8_t instate = 0;
  uint8_t outstate = 0;
  uint16_t inputRegisters[NUM_REGS];
  uint16_t holdingRegisters[NUM_REGS];

  // A Modbus command is pending
  if (modbus_getBusState() & (1<<MB_STATE_RECEIVE_COMPLETED))
  {
    // excute Modbus command
    switch(g_modbus_buffer[1])
    {
      // read coils or digital outputs
      case MB_FC_READ_COILS:
        modbus_exchangeBits(&outstate,0,8);
        break;

      // read digital inputs
      case MB_FC_READ_DISCRETE_INPUT:
        instate = readInputs();
        modbus_exchangeBits(&instate,0,8);
        break;

      // read registers or analog outputs
      case MB_FC_READ_REGISTERS:
        //holdingRegisters[0] = millis()/1000;
        modbus_exchangeRegisters(holdingRegisters, 0, NUM_REGS);
        break;

      // read analog inputs
      case MB_FC_READ_INPUT_REGISTER:
        inputRegisters[0] = (millis() / 1000);
        modbus_exchangeRegisters(inputRegisters, 0, NUM_REGS);
        break;

      // write single coil or output
      case MB_FC_WRITE_COIL:
        modbus_exchangeBits(&outstate, 0, 8);
        setOutputs(outstate);
        break;

      // write single register
      case MB_FC_WRITE_REGISTER:
        modbus_exchangeRegisters(holdingRegisters, 0, NUM_REGS);
        break;

      // write multiple coils or outputs
      case MB_FC_WRITE_MULTIPLE_COILS:
        modbus_exchangeBits(&outstate, 0, 8);
        //setOutputs(outstate);
        break;

      // write multiple registers
      case MB_FC_WRITE_MULTIPLE_REGISTERS:
        modbus_exchangeRegisters(holdingRegisters, 0, NUM_REGS);
        setOutputs(holdingRegisters[0]);
        break;

      default:
        modbus_sendException(MB_EC_ILLEGAL_FUNCTION);
        break;

    } // switch(g_modbus_buffer[1])

  } // command pending

} // modbus_handler()



/**
  \fn void main(void)

  \brief main routine

  Main routine. Initialize controller and poll for Modbus commands.
*/
void main(void)
{
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;

  // initialize GPIOs
  io_init();

  // in single address mode set address
  #ifndef MB_MULTIPLE_ADR
    modbus_setAddress(clientAddress);
  #endif

  // configure UART. In case of RS485 also initialize EN pin
  modbus_init(115200L);

  // init 1ms interrupt for SW clock and Modbus low-level handler
  TIM4_init();

  // enable interrupts
  ENABLE_INTERRUPTS();


  // main loop
  while(1)
  {
    // execute Modbus commands
    modbus_handler();

  } // main loop

} // main()

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
