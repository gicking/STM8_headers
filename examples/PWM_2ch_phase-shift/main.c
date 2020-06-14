/**********************
  Generate 2xPWM with 180deg phase-shift
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - configure timer 1 for up-down counter with 50kHz frequency
    - generate 2x PWM on pins Sduino pins 8(=PC1=TIM1_CH1) and 9(=PC3=TIM1_CH3)
    - ramp up/down duty cycle
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"

// TIM1 reload value for 50kHz center-aligned PWM
#define FREQUENCY  50000L
#define RELOAD     (16000000L / FREQUENCY / 2)


/**
  \fn void setDutyCycle(uint16_t duty)
   
  \brief set duty cycles of TIM1_CH1 and TIM1_CH3 
   
  \param duty    duty cycle (0...RELOAD)
  
  set duty cycles of TIM1_CH1 and TIM1_CH3 with 180 degree phase shift.
  Use buffered compare registers -> DC is updated on next over-/underflow event.

  No range check is performed and no dead-time is used -> use with care!
*/
void setDutyCycle(uint16_t duty) {

  // set duty cycle of TIM1_CH1
  sfr_TIM1.CCR1H.byte  = (uint8_t) (duty >> 8);
  sfr_TIM1.CCR1L.byte  = (uint8_t) duty;

  // set inverse duty cycle of TIM1_CH3
  duty = RELOAD - duty;
  sfr_TIM1.CCR3H.byte  = (uint8_t) (duty >> 8);
  sfr_TIM1.CCR3L.byte  = (uint8_t) duty;
    
} // setDutyCycle



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
    
  // set PC1(=TIM1_CH1) and PC3(=TIM1_CH3) to output
  sfr_PORTC.DDR.byte = (uint8_t) ((1 << 1) | (1 << 3));   // input(=0) or output(=1)
  sfr_PORTC.CR1.byte = (uint8_t) ((1 << 1) | (1 << 3));   // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.byte = (uint8_t) ((1 << 1) | (1 << 3));   // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // init TIM1
  sfr_TIM1.CR1.CMS     = 3;                               // PWM center-aligned mode 3
  sfr_TIM1.CR1.ARPE    = 1;                               // buffer period register to avoid glitches when changing frequency
  sfr_TIM1.CR1.OPM     = 0;                               // continuous PWM mode
  sfr_TIM1.PSCRH.byte  = 0;                               // set TIM1 prescaler to 1 --> fTIM1=16MHz
  sfr_TIM1.PSCRL.byte  = 0;
  sfr_TIM1.ARRH.byte   = (uint8_t) (RELOAD >> 8);         // set TIM1 auto-reload value = PWM period
  sfr_TIM1.ARRL.byte   = (uint8_t) RELOAD;
  sfr_TIM1.CCER1.CC1P  = 0;                               // set CH1 active high 
  sfr_TIM1.CCER2.CC3P  = 1;                               // set CH3 active low 
  sfr_TIM1.CCER1.CC1E  = 1;                               // enable CH1 output
  sfr_TIM1.CCER2.CC3E  = 1;                               // enable CH3 output
  sfr_TIM1.CCMR1.OC1M  = 6;                               // use PWM mode 1
  sfr_TIM1.CCMR3.OC3M  = 6;
  sfr_TIM1.CCMR1.OC1PE = 1;                               // buffer compare register to avoid glitches when changing duty cycle
  sfr_TIM1.CCMR3.OC3PE = 1;
  sfr_TIM1.BKR.MOE     = 1;                               // main output enable
  sfr_TIM1.CR1.CEN     = 1;                               // start TIM1
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // main loop
  while(1) {
  
    // slowly ramp-up duty cycle (0...RELOAD)
    for (duty=0; duty<=RELOAD; duty+=1) {

      // set duty cycles for both channels
      setDutyCycle(duty);
      
      // simple wait
      for (i=0; i<30000L; i++)
        NOP();
    
    } // duty ramp
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
