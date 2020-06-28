/**********************
  Periodically print text to 2x16 char LCD attached to I2C
   
  supported hardware:
    - muBoard (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
    - Batron BTHQ21605V-COG-FSRE-I2C 2X16 (Farnell 1220409). Pins PE1/SCL, PE2/SDA, and PE3/LCD reset
  
  Functionality:
    - initialize I2C bus
    - initialize and reset LCD display
    - periodically print time to LCD  
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "i2c.h"
  #include "lcd-BTHQ21605V.h"
#undef _MAIN_


/////////////////
//    main routine
/////////////////
void main(void) {

  uint32_t   i;
  int        count=0;
  char       str[20];

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init I2C bus
  i2c_init();

  // reset and init LCD display
  BTHQ21605V_lcd_init(&sfr_PORTE, 3);

  // LED connected to PH3
  sfr_PORTH.DDR.DDR3 = 1;     // input(=0) or output(=1)
  sfr_PORTH.CR1.C13  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTH.CR2.C23  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // clear display
  BTHQ21605V_lcd_clear();
    
  // main loop
  while(1) {
  
    // print time to LCD
    sprintf(str, "count = %d ", count++);
    BTHQ21605V_lcd_print(2, 1, str);
    
    // blink LED
    sfr_PORTH.ODR.ODR3 ^= 1;
    
    // simple wait
    for (i=0; i<300000L; i++)
      NOP();
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
