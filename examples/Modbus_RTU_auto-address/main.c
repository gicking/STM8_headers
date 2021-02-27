/*
 A simple example project for remote controlling a STM via Modbus RTU and RS485.
 Also supports randomization of Modbus IDs via broadcast write.

 Used "top level" protocol:
   - all registers are 16-bit (Modbus standard)
   - only Modbus functions WRITE_MULTIPLE_REGISTERS and MB_FC_READ_INPUT_REGISTER used
   - read/write only allowed from/to address 0
   - command code is in reg[0], parameters and return values in reg[1..NUM_REGS-1]
   - a pending command is indicated via bit 15 (highest bit) set
   - after completion of a command, bit 15 is cleared
   - in case of an error on client side, bit 14 is set and the error code is stored in reg[1]
*/

 /*----------------------------------------------------------
     INCLUDE FILES
 ----------------------------------------------------------*/

#include "config.h"
#include <stdlib.h>
#define _MAIN_          // required for global variables
  #include "yaMBSistm8.h"
  #include "timer4.h"
  #ifdef sfr_UART_DEBUG
    #include "debug_uart.h"
  #endif
  #include "client_commands.h"
#undef _MAIN_


/*----------------------------------------------------------
    MACROS/DEFINES
----------------------------------------------------------*/

/// number of registers to read or write
#define  NUM_REGS   16


/*----------------------------------------------------------
    MODULE VARIABLES
----------------------------------------------------------*/

// registers and states to read or write
uint16_t      reg[NUM_REGS];


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/

/**
  \fn void setLED(uint8_t states)

  \brief set LED state

  \param  states containing up to 8 digital output states

  Set up to 8 digital outputs. Is project specific. Here only LED
*/
void setLED(uint8_t states)
{
  if (states)
    LED_PORT.ODR.byte |= LED_PIN;
  else
    LED_PORT.ODR.byte &= ~LED_PIN;

} // setOutputs()



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
  // A Modbus command is pending
  if (modbus_getBusState() & (1<<MB_STATE_RECEIVE_COMPLETED))
  {
    // excute Modbus command. Only read/write registers implemented
    // commands are in reg[0], parameters/responses in [1..NUM_REGS-1]
    switch(g_modbus_buffer[1])
    {
      // read multiple registers. Only start address 0 allowed
      case MB_FC_READ_INPUT_REGISTER:
        if (modbus_requestedAddress() != 0)
          modbus_sendException(MB_EC_ILLEGAL_DATA_ADDRESS);
        else
          modbus_exchangeRegisters(reg, 0, NUM_REGS);
        break;

      // write multiple registers. Only start address 0 allowed
      case MB_FC_WRITE_MULTIPLE_REGISTERS:
        if (modbus_requestedAddress() != 0)
          modbus_sendException(MB_EC_ILLEGAL_DATA_ADDRESS);
        else
          modbus_exchangeRegisters(reg, 0, NUM_REGS);
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
  uint32_t    randomSeed = 0;   // required for "real" random numbers

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

  // configure Modbus UART. In case of RS485 also initialize EN pin
  modbus_init(19200L);

  // configure debug UART if used
  #ifdef sfr_UART_DEBUG
    UART_debug_begin(115200L);
  #endif

  // init 1ms interrupt for SW clock and Modbus low-level handler
  TIM4_init();

  // enable interrupts
  ENABLE_INTERRUPTS();


  // main loop
  while(1)
  {
    //////////////
    // execute time controlled tasks
    //////////////

    // dummy


    //////////////
    // handle Modbus commands
    //////////////

    // execute Modbus handler
    modbus_handler();

    // check if command is pending (bit15 is set)
    if (reg[0] & 0x8000)
    {
      // on first command received, store micros() as random seed for PRNG
      if (randomSeed == 0)
      {
        randomSeed = micros();
        srand(randomSeed);
      }

      // execute command received via Modbus
      switch (reg[0])
      {
        // set state of LED
        case CMD_SET_LED:

          // check number of parameters (command + state)
          if (modbus_requestedAmount() != 2)
          {
            reg[0] |= 0x4000;                       // indicate error (set bit 14)
            reg[1] = ERROR_PARAMETER_NUMBER;        // set error code in reg[1]

            #ifdef sfr_UART_DEBUG
              printf("CMD_SET_LED: wrong number of parameters (%d vs. 1)\n", (int) (modbus_requestedAmount()-1));
            #endif
          }

          // no error -> execute command
          else
          {
            setLED(reg[1]);

            #ifdef sfr_UART_DEBUG
              printf("set LED: %d\n", (int) (reg[1]));
            #endif
          }
          break; // CMD_SET_LED


          // randomize register content
          case CMD_REG_RANDOMIZE:

            // check number of parameters (command)
            if (modbus_requestedAmount() != 1)
            {
              reg[0] |= 0x4000;                       // indicate error (set bit 14)
              reg[1] = ERROR_PARAMETER_NUMBER;        // set error code in reg[1]

              #ifdef sfr_UART_DEBUG
                printf("CMD_REG_RANDOMIZE: wrong number of parameters (%d vs. 0)\n", (int) (modbus_requestedAmount()-1));
              #endif
            }

            // no error -> execute command
            else
            {
              for (int i=1; i<NUM_REGS; i++)
                reg[i] = rand();

              #ifdef sfr_UART_DEBUG
                printf("registers randomized\n");
              #endif
              break; // CMD_REG_RANDOMIZE
            }


        // randomize Modbus ID
        case CMD_ID_RANDOMIZE:

          // check number of parameters (command + id)
          if (modbus_requestedAmount() != 2)
          {
            reg[0] |= 0x4000;                       // indicate error (set bit 14)
            reg[1] = ERROR_PARAMETER_NUMBER;        // set error code in reg[1]

            #ifdef sfr_UART_DEBUG
              printf("CMD_ID_RANDOMIZE: wrong number of parameters (%d vs. 0)\n", (int) (modbus_requestedAmount()-1));
            #endif
          }

          // no error -> execute command
          else
          {
            // if ID matches, set sew ID
            if (reg[1] == modbus_getAddress())
            {
              modbus_setAddress(1 + (rand() % 246));      // set new ID

              #ifdef sfr_UART_DEBUG
                printf("new ID: 0x%04X\n", modbus_getAddress());
              #endif
            }
            break; // CMD_ID_RANDOMIZE
          }


        // command code unknown -> set error
        default:
          reg[0] |= 0x4000;                       // indicate error (set bit 14)
          reg[1] = ERROR_ILLEGAL_COMMAND;         // set error code in reg[1]

          #ifdef sfr_UART_DEBUG
            printf("illegal command: 0x%04X\n", reg[0]);
          #endif
        break;
      }

      // clear "command pending" bit (=bit15)
      reg[0] &= ~(0x8000);

    } // command is pending

  } // main loop

} // main()

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
