/**********************
  Use TLI interrupt on pin PD7 (=muBoard "automode" switch)
  
  supported hardware:
    - muDuino (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
  Functionality:
    - configure 2 LED pins
    - toggle LED1 in TLI ISR (switch "automode") 
    - poll TLI pin and mirror to LED2
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "../../include/STM8S207MB.h"

// define pins
#define PIN_TLI    sfr_PORTD.IDR.IDR7
#define LED_GREEN  sfr_PORTH.ODR.ODR2
#define LED_RED    sfr_PORTH.ODR.ODR3


/**
  \fn void TLI_ISR(void)
   
  \brief ISR for top level / pin interrupt
   
  interrupt service routine for top level / pin interrupt.

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(TLI_ISR, _TLI_VECTOR_)
{
  // toggle LED state
  LED_RED ^= 1;

  return;

} // TLI_ISR



/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
  
  // configure pin PD7 (=switch "automode") as input pull-up with interrupt
  sfr_PORTD.DDR.DDR7 = 0;     // input(=0) or output(=1)
  sfr_PORTD.CR1.C17  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTD.CR2.C27  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // set TLI edge sensitivity
  sfr_ITC.CR2.TLIS = 1;      // rising
  //sfr_ITC.CR2.TLIS = 0;      // falling

  // configure LED pins PH2+PH3 as output
  sfr_PORTH.DDR.byte = (1 << 2) | (1 << 3);     // input(=0) or output(=1)
  sfr_PORTH.CR1.byte = (1 << 2) | (1 << 3);     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTH.CR2.byte = (1 << 2) | (1 << 3);     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // mirror TLI pin to green LED via polling
  while(1)
    LED_GREEN = PIN_TLI;

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
