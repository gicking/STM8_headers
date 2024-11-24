/**********************
  Use port interrupt on two pins of same port (PD3+4)
  
  Supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Setup:
    - connect board pins A0 (=PB0=output) and 6 (=PD3=input)
    - connect board pins A1 (=PB1=output) and 5 (=PD4=input)
  
  Functionality:
    - Init:
      - configure USB/UART with 19.2kBaud
      - configure PB0 and PB1 as output
      - configure PD3 and PD4 for port interrupt. Set sensitivity via EDGE_SENSITIVITY
    - Loop:
      - toggle pins PB0 and PB1 every 2s, with 1s phase shift inbetween
      - in port ISR, write byte to UART, depending on event

  Note:
    - For interrupt on multiple port pins, use change interrupt and handle edge in ISR
    - For interrupt on single port pin, just configure sfr_ITC_EXTI accordingly
**********************/

/*----------------------------------------------------------
  INCLUDE FILES
----------------------------------------------------------*/

#include "../../include/STM8S105K6.h"
#if defined(SDUINO)
  //#include "../../include/STM8S105K6.h"
#else
  #error board not supported yet
#endif


/*----------------------------------------------------------
  MACROS
----------------------------------------------------------*/

// select edge sensititvity (0=low-level (careful!), 1=rising, 2=falling, 3=change)
#define EDGE_SENSITIVITY  3


/*----------------------------------------------------------
  GLOBAL FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void delay_500ms(void)
   
  \brief halt code for ~0.5s using NOP (w/o timer)
   
  Halt code for ~0.5s using NOP (w/o timer). Timing measured for SDCC v4.2.0 @ fCPU=16MHz.
*/
void delay_500ms(void)
{
  uint32_t count;

  // simple wait
  for (count=0; count<900000L; count++)
    NOP();

} // delay_500ms()


/**
  \fn void send_uart(char)
   
  \brief send byte via UART
   
  send byte via UART
*/
void send_uart(char c)
{
  // wait until Tx buffer empty
  while (!(sfr_UART2.SR.TXE));

  // write byte to Tx buffer
  sfr_UART2.DR.byte = c;

} // send_uart()


/**
  \fn void PORTE_ISR(void)
   
  \brief ISR for port E interrupt
   
  interrupt service routine for port E external interrupt.

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(PORTD_ISR, _EXTI3_VECTOR_)
{
  // interrupt on low level (careful!). Only check for ISR-pins=low, not previous state
  #if (EDGE_SENSITIVITY == 0)

    // get current state of port D once for speed
    uint8_t state_curr = sfr_PORTD.IDR.byte;

    // if PD3=low react accordingly
    if (!(state_curr & PIN3))
    {
      // act on PD3 interrupt. Here simply write '3' to UART
      send_uart('3');
    }

    // if PD4=low react accordingly
    if (!(state_curr & PIN4))
    {
      // act on PD3 interrupt. Here simply write '3' to UART
      send_uart('3');
    }
  
    return;


  // interrupt on rising edge. Check if respective ISR pin is high and compare with previous state
  #elif (EDGE_SENSITIVITY == 1)

    uint8_t         state_curr;           // current port state
    static uint8_t  state_old = 0xFF;     // previous port state 

    // get current port state only once for speed
    state_curr = sfr_PORTD.IDR.byte;

    // check if PD3=high and pin status changed since last call
    if ((state_curr & PIN3) && ((state_curr & PIN3) != (state_old & PIN3)))
    {
      // act on PD3 rising edge. Here send 'A' via UART
      send_uart('A');

    } // PD3 rising edge

    // check if PD4=high and pin status changed since last call
    if ((state_curr & PIN4) && ((state_curr & PIN4) != (state_old & PIN4)))
    {
      // act on PD4 rising edge. Here send 'B' via UART
      send_uart('B');

    } // PD4 rising edge
  
    // remember port state for next call
    state_old = state_curr;
  
    return;


  // interrupt on falling edge. Check if respective ISR pin is low and compare with previous state
  #elif (EDGE_SENSITIVITY == 2)

    uint8_t         state_curr;           // current port state
    static uint8_t  state_old = 0xFF;     // previous port state 

    // get current port state only once for speed
    state_curr = sfr_PORTD.IDR.byte;

    // check if PD3=low and pin status changed since last call
    if ((!(state_curr & PIN3)) && ((state_curr & PIN3) != (state_old & PIN3)))
    {
      // act on PD3 falling edge. Here send 'a' via UART
      send_uart('a');

    } // PD3 falling edge

    // check if PD4=low and pin status changed since last call
    if ((!(state_curr & PIN4)) && ((state_curr & PIN4) != (state_old & PIN4)))
    {
      // act on PD4 falling edge. Here send 'b' via UART
      send_uart('b');

    } // PD4 falling edge
  
    // remember port state for next call
    state_old = state_curr;
  
    return;


  // interrupt on rising or falling edge. Check if respective pin state has changed
  #elif (EDGE_SENSITIVITY == 3)

    uint8_t         state_curr;           // current port state
    static uint8_t  state_old = 0xFF;     // previous port state 

    // get current port state only once for speed
    state_curr = sfr_PORTD.IDR.byte;

    // check if PD3 status changed since last call
    if ((state_curr & PIN3) != (state_old & PIN3))
    {
      // PD3 is high (-> rising). Send 'A' via UART
      if (state_curr & PIN3)
        send_uart('A');

      // PD3 is low (-> falling). Send 'a' via UART
      else
        send_uart('a');

    } // PD3 changed

    // check if PD4 status changed since last call
    if ((state_curr & PIN4) != (state_old & PIN4))
    {
      // PD4 is high (-> rising). Send 'B' via UART
      if (state_curr & PIN4)
        send_uart('B');

      // PD4 is low (-> falling). Send 'b' via UART
      else
        send_uart('b');

    } // PD4 changed
  
    // remember port state for next call
    state_old = state_curr;
  
    return;


  // illegal port sensitivity -> error
  #else
    #error EDGE_SENSITIVITY out of range 0..3
  #endif

} // PORTD_ISR()


/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
  
  // set UART baudrate (note: BRR2 must be written before BRR1!), then enable Tx & Rx
  uint16_t val16 = (uint16_t) (((uint32_t) 16000000L)/19200);
  sfr_UART2.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  sfr_UART2.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  sfr_UART2.CR2.REN  = 1;  // enable receiver
  sfr_UART2.CR2.TEN  = 1;  // enable sender
  
  // configure pins PB0 + PB1 as output (generate signals for ISRs)
  sfr_PORTB.DDR.byte |= (PIN0 | PIN1);    // input(=0) or output(=1)
  sfr_PORTB.CR1.byte |= (PIN0 | PIN1);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTB.CR2.byte |= (PIN0 | PIN1);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // configure pins PD3 + PD4 as input pull-up with interrupt
  sfr_PORTD.DDR.byte &= ~(PIN3 | PIN4);   // input(=0) or output(=1)
  sfr_PORTD.CR1.byte |= (PIN3 | PIN4);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTD.CR2.byte |= (PIN3 | PIN4);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // configure edge sensitivity for port D (all 8 pins) . For edge triggered only use change, handle in ISR! 
  #if (EDGE_SENSITIVITY == 0)
    sfr_ITC_EXTI.CR1.PDIS = 0;
  #else
    sfr_ITC_EXTI.CR1.PDIS = 3;
  #endif

  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // main loop
  while(1)
  {
    // toggle pins PB0 and PB1 with ~0.5s delay
    sfr_PORTB.ODR.ODR0 ^= 1;
    delay_500ms();
    sfr_PORTB.ODR.ODR1 ^= 1;
    delay_500ms();

  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/