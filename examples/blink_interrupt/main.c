/**********************
  Blink project with 1ms interrupts 
  
  supported hardware:
    - STM8S Discovery board (https://www.st.com/en/evaluation-tools/stm8s-discovery.html)
    - STM8L Discovery board (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - muDuino (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
  Functionality:
    - blink LED in TIM4 ISR
**********************/

/*----------------------------------------------------------
    SELECT BOARD
----------------------------------------------------------*/
//#define STM8S_DISCOVERY
//#define STM8L_DISCOVERY
#define SDUINO
//#define MUDUINO


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"
  #define LED_PORT   sfr_PORTD
  #define LED_PIN    0
#elif defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    7
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    5
#elif defined(MUDUINO)
  #include "../../include/STM8S207MB.h"
  #define LED_PORT   sfr_PORTH
  #define LED_PIN    2
#else
  #error undefined board
#endif



/**
  \fn void TIM4_UPD_ISR(void)
   
  \brief ISR for timer 4 (1ms master clock)
   
  interrupt service routine for timer TIM4.
  Used for 1ms master clock and optional user function

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
#if defined(STM8S_DISCOVERY) || defined(SDUINO)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_)
#else
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_UIF_VECTOR_)
#endif
{

  static uint16_t count = 0;
  
  // blink LED
  count++;
  if (count == 500) {
    count = 0;

    // toggle LED
    LED_PORT.ODR.byte ^= (1 << LED_PIN);    

  } // 500ms
  
  // clear timer 4 interrupt flag
  #if defined(STM8L_DISCOVERY)
    sfr_TIM4.SR1.UIF = 0;
  #else
    sfr_TIM4.SR.UIF = 0;
  #endif
  
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
  sfr_CLK.CKDIVR.byte = 0x00;
  
  // for low-power device activate TIM4 clock
  #if defined(STM8L_DISCOVERY)
    sfr_CLK.PCKENR1.PCKEN12 = 1;
  #endif
   
  // configure LED pin as output
  LED_PORT.DDR.byte = (uint8_t) (1 << LED_PIN);    // input(=0) or output(=1)
  LED_PORT.CR1.byte = (uint8_t) (1 << LED_PIN);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  LED_PORT.CR2.byte = (uint8_t) (1 << LED_PIN);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
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
