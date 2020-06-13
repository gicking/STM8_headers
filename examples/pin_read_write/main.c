/**********************
  Read and write pin states w/o interrupts
   
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - configure pin 2 as input, pin 13(=LED) as output
    - poll pin 2 and mirror to pin 13
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "../../include/STM8S105K6.h"


/////////////////
//    main routine
/////////////////
void main (void) {

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // configure LED pin (=PC5) as output
  sfr_PORTC.DDR.DDR5 = 1;    // input(=0) or output(=1)
  sfr_PORTC.CR1.C15  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.C25  = 1;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    
  // configure pin 2 (=PD7) as input pull-up
  sfr_PORTD.DDR.DDR7 = 0;    // input(=0) or output(=1)
  sfr_PORTD.CR1.C17  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTD.CR2.C27  = 0;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  

  // main loop
  while(1) {
	
    // mirror pin 2 to pin 13
    sfr_PORTC.ODR.ODR5 = sfr_PORTD.IDR.IDR7;
        
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
