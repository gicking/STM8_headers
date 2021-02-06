/**********************
  Read and write pin states w/o interrupts
   
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - STM8L Discovery board (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)
  
  Functionality:
    - configure pin 2 as input, pin 13(=LED) as output
    - poll pin 2 and mirror to pin 13
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#define STM8L_DISCOVERY
//#define SDUINO

#if defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
#else
  #error undefined board
#endif


/////////////////
//    main routine
/////////////////
void main (void) {

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  #if defined(STM8L_DISCOVERY)

    // configure LED pin (=PE7) as output
    sfr_PORTE.DDR.DDR7 = 1;     // input(=0) or output(=1)
    sfr_PORTE.CR1.C17  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTE.CR2.C27  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
    // configure button (=PC1) as input pull-up
    sfr_PORTC.DDR.DDR1 = 0;    // input(=0) or output(=1)
    sfr_PORTC.CR1.C11  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C21  = 0;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope  
  
  #elif defined(SDUINO)
  
    // configure LED pin D13 (=PC5) as output
    sfr_PORTC.DDR.DDR5 = 1;    // input(=0) or output(=1)
    sfr_PORTC.CR1.C15  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C25  = 1;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
    // configure pin D2 (=PD7) as input pull-up
    sfr_PORTD.DDR.DDR7 = 0;    // input(=0) or output(=1)
    sfr_PORTD.CR1.C17  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTD.CR2.C27  = 0;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  #endif 

  // main loop
  while(1) {
	
    // mirror input pin to output pin
   #if defined(STM8L_DISCOVERY)
     sfr_PORTE.ODR.ODR7 = sfr_PORTC.IDR.IDR1;
   #elif defined(SDUINO)
     sfr_PORTC.ODR.ODR5 = sfr_PORTD.IDR.IDR7;
   #endif
      
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
