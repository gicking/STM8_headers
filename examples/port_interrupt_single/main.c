/**********************
  Use port interrupt on pin PE5 (=muBoard button)
  
  supported hardware:
    - muDuino (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
  Functionality:
    - configure 2 LED pins
    - toggle LED1 in port E ISR
    - poll button pin and mirror to LED2
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "../../include/STM8S207MB.h"

// define pins
#define PIN_BUTTON  sfr_PORTE.IDR.IDR5
#define LED_GREEN   sfr_PORTH.ODR.ODR2
#define LED_RED     sfr_PORTH.ODR.ODR3


/**
  \fn void PORTE_ISR(void)
   
  \brief ISR for port E interrupt
   
  interrupt service routine for port E external interrupt.

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(PORTE_ISR, _EXTI4_VECTOR_)
{
  // toggle LED state
  LED_RED ^= 1;

  return;

} // PORTE_ISR



/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
  
  // configure pin PE5 (=button) as input pull-up with interrupt
  sfr_PORTE.DDR.DDR5 = 0;     // input(=0) or output(=1)
  sfr_PORTE.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTE.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // configure edge sensitivity for port E (all 8 pins) 
  //sfr_ITC.CR2.PEIS = 0;       // low level (careful!)
  //sfr_ITC.CR2.PEIS = 1;       // rising
  //sfr_ITC.CR2.PEIS = 2;       // falling
  sfr_ITC.CR2.PEIS = 3;       // change

  // configure LED pins PH2+PH3 as output
  sfr_PORTH.DDR.byte = (1 << 2) | (1 << 3);     // input(=0) or output(=1)
  sfr_PORTH.CR1.byte = (1 << 2) | (1 << 3);     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTH.CR2.byte = (1 << 2) | (1 << 3);     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // main loop
  while(1)
  {
    // simple wait ~500ms @ 16MHz
    for (uint32_t i=0; i<300000L; i++)
      NOP();

    // toggle green LED as life beat
    LED_GREEN ^= 1;

  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
