/**********************
  Generate a PWM on PD2/TIM3_CH1 (=pin 3 on sduino)
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - generate a PWM on sduino pin 3 (=PD2/TIM3_CH1)
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "timer3.h"
#undef _MAIN_


/////////////////
//    main routine
/////////////////
void main (void) {

  uint32_t    i;
  uint16_t    duty;
	
	
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // set PD2 (=TIM3_CH1) to output
  sfr_PORTD.DDR.DDR2 = 1;
  sfr_PORTD.CR1.C12  = 1;
  sfr_PORTD.CR2.C22  = 1;
  
  // init TIM3 for PWM generation. Set frequency [0.01Hz]
  TIM3_init();
  TIM3_setFrequency(10000);  // =100Hz
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // main loop
  while(1) {
  
    // loop over duty cycle [0.1%]
    for (duty=0; duty<=1000; duty+=10) {

      // set duty cycle [0.1%] of TIM3_CH1 (=PD2)
      TIM3_setDutyCycle(1, duty);
      
      // simple wait
      for (i=0; i<30000L; i++)
        NOP();
    
    } // duty ramp
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
