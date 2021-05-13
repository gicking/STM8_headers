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

// optimization options (0..2), see mail by Philipp Krause on 2021-05-13
#define OPTIMIZATION 0

#define NUM_SAMPLES	64
volatile int16_t in[NUM_SAMPLES];
volatile int16_t out[NUM_SAMPLES];

#if (OPTIMIZATION==0)
  const int16_t a0 = 16384;
  const int16_t a1 = -32768;
  const int16_t a2 = 16384;
  const int16_t b1 = -25576;
  const int16_t b2 = 10508;

#else
  #define a0 16384
  #define a1 -32768
  #define a2 16384
  #define b1 -25576
  #define b2 10508
#endif


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

    // original
    #if (OPTIMIZATION==0) || (OPTIMIZATION==1)
    for (i = 0; i < NUM_SAMPLES; i++) {
      inTemp = in[i];
      outTemp = inTemp * a0 + z1;
      z1 = inTemp * a1 + z2 - b1 * outTemp;
      z2 = inTemp * a2 - b2 * outTemp;
      out[i] = outTemp;
    }

    // optimized with bitshift (by Philipp Krause)
    #elif (OPTIMIZATION==2)
    for (i = 0; i < NUM_SAMPLES; i++) {
      inTemp = in[i];
      outTemp = (inTemp << 14) + z1;
      int tmp = z2 - b1 * z1;
      z2 = (inTemp << 14) - b2 * z1;
      z1 = (inTemp & 1) ? tmp ^ 0x8000 : tmp;
      out[i] = outTemp;
    }
    
    // error
    #else
      #error unknown optimization option 
    #endif
    
    sfr_PORTD.ODR.byte = 0x00;  // one cycle
  }

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
