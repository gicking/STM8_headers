/**********************
  Blink project with 1ms interrupts
  Mix with functions/headers of the STM8S Standard Peripheral Library (SPL)
  (SDCC patch available from https://github.com/gicking/STM8-SPL_SDCC_patch)
  
  supported hardware:
    - STM8S Discovery board (https://www.st.com/en/evaluation-tools/stm8s-discovery.html)
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
  
  Functionality:
    - blink LED in TIM4 ISR
   
  Notes: 
   - due to license concerns SPL sources are not includes. For Cosmic & IAR 
     copy stm8s.h, stm8s_clk.c/.h, stm8s_gpio.c/.h from SPL here
**********************/

/*----------------------------------------------------------
    SELECT BOARD
----------------------------------------------------------*/
//#define STM8S_DISCOVERY
#define SDUINO


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
//#include <stdint.h>
#if defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
#else
  #error undefined board
#endif

// SPL headers. Configure modules to include in Makefile and stm8s_conf.h
#include "stm8s_conf.h"



/**
  \fn void TIM4_UPD_ISR(void)
   
  \brief ISR for timer 4 (1ms master clock)
   
  interrupt service routine for timer TIM4.
  Used for 1ms master clock and optional user function

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_)
{

  static uint16_t count = 0;
  
  // blink LED
  count++;
  if (count == 500) {
    count = 0;

    // toggle LED
    #if defined(STM8S_DISCOVERY)
      //sfr_PORTD.ODR.byte ^= (1 << 0);    // byte read/write
      GPIO_WriteReverse(GPIOD, (GPIO_Pin_TypeDef)GPIO_PIN_0);
    #else
      //sfr_PORTC.ODR.ODR5 ^= 1;           // bitfield read/write
      GPIO_WriteReverse(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_5);
    #endif

  } // 500ms
  
  // clear timer 4 interrupt flag
  sfr_TIM4.SR.UIF = 0;
  
  return;

} // TIM4_UPD_ISR


/**
  \fn void TIM4_init(void)
   
  \brief init timer 4 for 1ms master clock with interrupt
   
  init 8-bit timer TIM4 with 1ms tick. Is used for SW master clock
  via below interrupt.
*/
void TIM4_init(void) {

  // stop the timer
  sfr_TIM4.CR1.CEN = 0;
  
  // clear counter
  sfr_TIM4.CNTR.byte = 0x00;

  // auto-reload value buffered
  sfr_TIM4.CR1.ARPE = 1;

  // clear pending events
  sfr_TIM4.EGR.byte  = 0x00;

  // set clock to 16Mhz/2^6 = 250kHz -> 4us period
  sfr_TIM4.PSCR.PSC = 6;

  // set autoreload value for 1ms (=250*4us)
  sfr_TIM4.ARR.byte  = 250;

  // enable timer 4 interrupt
  sfr_TIM4.IER.UIE = 1;
  
  // start the timer
  sfr_TIM4.CR1.CEN = 1;
  
} // TIM4_init



/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  //sfr_CLK.CKDIVR.byte = 0x00;
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  
  // configure LED pin as output
  #if defined(STM8S_DISCOVERY)                // STM8S Discovery: LED connected to PD0
    //sfr_PORTD.DDR.byte = (uint8_t) (1 << 0);    // input(=0) or output(=1)
    //sfr_PORTD.CR1.byte = (uint8_t) (1 << 0);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    //sfr_PORTD.CR2.C20  = 1;                     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    GPIO_Init(GPIOD, (GPIO_Pin_TypeDef)GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);
  #else
    //sfr_PORTC.DDR.byte = (uint8_t) (1 << 5);    // input(=0) or output(=1)
    //sfr_PORTC.CR1.byte = (uint8_t) (1 << 5);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    //sfr_PORTC.CR2.C25  = 1;                     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    GPIO_Init(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
  #endif
  
  // init 1ms interrupt
  TIM4_init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // dummy main loop. Action happens in ISR TIM4 TIM4_UPD 
  while(1);

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
