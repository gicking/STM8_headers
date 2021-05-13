/**********************
  Benchmark for comparison of Biquad algorithms performance from
  
  https://github.com/jaydcarlson/microcontroller-test-code/blob/master/STM8/projects/biquad/main.c
**********************/

/*----------------------------------------------------------
    SELECT BOARD
----------------------------------------------------------*/
#define STM8S_DISCOVERY


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"
#else
  #error undefined board
#endif



/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/

#define NUM_SAMPLES	64
volatile int16_t in[NUM_SAMPLES];
volatile int16_t out[NUM_SAMPLES];

const int16_t a0 = 16384;
const int16_t a1 = -32768;
const int16_t a2 = 16384;
const int16_t b1 = -25576;
const int16_t b2 = 10508;

int16_t z1, z2;
int16_t outTemp;
int16_t inTemp;


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

void main (void)
{
  // GO REAL FAST!
  sfr_CLK.CKDIVR.byte = 0x00;
  
	// init testpin PD0 / CN4 pin 5 / LED
  sfr_PORTD.DDR.DDR0 = 1;    // input(=0) or output(=1)
  sfr_PORTD.CR1.C10  = 1;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTD.CR2.C20  = 1;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // main loop
  while (1)
  {
    uint8_t i;
    sfr_PORTD.ODR.byte = 0x01;  // one cycle
    for (i = 0; i < NUM_SAMPLES; i++) {
      inTemp = in[i];
      outTemp = inTemp * a0 + z1;
      z1 = inTemp * a1 + z2 - b1 * outTemp;
      z2 = inTemp * a2 - b2 * outTemp;
      out[i] = outTemp;
    }
    sfr_PORTD.ODR.byte = 0x00;  // one cycle
  }

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
