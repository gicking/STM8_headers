/**********************
  Project to demonstrate beeper

  supported hardware:
    - NUCLEO-8S208RB (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)

  Functionality:
    - activate beeper output via option bytes
    - generate different frequencies on BEEP pin
    - no LSI calibration (yet)
    - no interrupts
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"
#include "beeper.h"
#include "option_bytes.h"


// simple wait ~500ms @ 16MHz
void delay(void)
{
  uint32_t  i;
  for (i=0; i<300000L; i++)
    NOP();
}



/////////////////
//    main routine
/////////////////
void main (void)
{
  uint8_t   OPT, NOPT;


  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;


  // if ARF7 (BEEPER) not set or NOPT not inverse of OPT, change option bytes
  OPT  = sfr_OPT.OPT2.byte;
  NOPT = sfr_OPT.NOPT2.byte;
  if ((!(OPT & (1<<7))) || ((OPT ^ NOPT) != 0xFF))
  {
    OPT |= (1<<7);
    NOPT = 0xFF ^ OPT;
    write_option_byte((uint16_t)(&(sfr_OPT.OPT2)),  OPT);
    write_option_byte((uint16_t)(&(sfr_OPT.NOPT2)), NOPT);
    SW_RESET();
  }

  // set clock prescaler and enable beeper
  // Note: LSI calibration not yet supported!
  beeper_divider(14);
  beeper_freq(2);
  beeper_enable();


  // configure LED pin as output
  sfr_PORTC.DDR.DDR5 = 1;    // input(=0) or output(=1)
  sfr_PORTC.CR1.C15  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.C25  = 1;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // main loop
  while(1) {

    // toggle LED
    sfr_PORTC.ODR.ODR5 ^= 1;

    // generate 1kHz
    beeper_freq(0);
    delay();

    // generate 2kHz
    beeper_freq(1);
    delay();

    // generate 4kHz
    beeper_freq(2);
    delay();

  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
