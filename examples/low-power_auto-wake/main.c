/**********************
  Enter power-saving modes with auto-wake via 
    - any interrupt (lowPower_Wait)
    - external interrupt or auto-wake (lowPower_HaltAWU)  
    - external interrupt (lowPower_Halt)
  
  supported hardware:
    - muDuino (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
  Functionality:
    - configure wake pin as input pull-up with interrupt on falling edge
    - configure LED output pins
    - enter power-down mode with wake options port-ISR or AWU
    - indicate wake event and active status via LEDs
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"
#define _MAIN_          // required for global variables
  #include "awu.h"
  #include "power_saving.h"
#undef _MAIN_


/**
  \fn void PORTE_ISR(void)
   
  \brief ISR for port E
   
  interrupt service routine for port E

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(PORTE_ISR, _EXTI4_VECTOR_) {
 
  // dummy
  
} // PORTE_ISR



/////////////////
//    main routine
/////////////////
void main (void) {

  uint32_t  i;
  int       j;
  
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // configure pin PE5 (=button) as input pull-up with interrupt
  sfr_PORTE.DDR.DDR5 = 0;     // input(=0) or output(=1)
  sfr_PORTE.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTE.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // configure edge sensitivity for port E (all 8 pins) 
  sfr_ITC.CR2.PEIS = 2;       // falling

  // configure LED pins PH2 (green) & PH3 (red) as output
  sfr_PORTH.DDR.byte = (1 << 2) | (1 << 3);     // input(=0) or output(=1)
  sfr_PORTH.CR1.byte = (1 << 2) | (1 << 3);     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTH.CR2.byte = (1 << 2) | (1 << 3);     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  sfr_PORTH.ODR.byte = (1 << 2) | (1 << 3);     // LED off
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  

  // main loop
  while(1) {
  
    // blink green LED a few times to show awake status
    for (j=0; j<8; j++) {
      sfr_PORTH.ODR.ODR2 ^= 1;
      for (i=0; i<60000L; i++)
        NOP();
    }

    // enter power saving mode (sorted by decreasing power consumption)
    //lowPower_Wait();          // enter WAIT mode, wake via button
    //lowPower_Halt();          // enter HALT mode, wake via button
    lowPower_HaltAWU(2000);   // enter active HALT mode, wake via button, or latest after 2s

    // toggle red LED in AWU-ISR to indicate wake

  }

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
