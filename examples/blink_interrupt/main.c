/**********************
  Blink project with 1ms interrupts 
  
  supported hardware:
    - STM8S Discovery board (https://www.st.com/en/evaluation-tools/stm8s-discovery.html)
    - STM8L Discovery board (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)
    - NUCLEO-8S207K8 (https://www.st.com/en/evaluation-tools/nucleo-8s207k8.html)
    - NUCLEO-8S208RB (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)
    - STM8-SO8 STM8S001 Discovery board (https://www.st.com/en/evaluation-tools/stm8-so8-disco.html)
    - STM8-SO8 STM8L001 Discovery board (https://www.st.com/en/evaluation-tools/stm8-so8-disco.html)
    - STM8-SO8 STM8S050 Discovery board (https://www.st.com/en/evaluation-tools/stm8-so8-disco.html)
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - muBoard (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
  Functionality:
    - blink LED in TIM4 ISR
**********************/

/*----------------------------------------------------------
    INCLUDE FILES / MACROS
----------------------------------------------------------*/

// STM8 device header & LED pin
#if defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"
  #define LED_PORT   sfr_PORTD
  #define LED_PIN    PIN0
#elif defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN7
#elif defined(NUCLEO_8S207K8)
  #include "../../include/STM8S207K8.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(NUCLEO_8S208RB)
  #include "../../include/STM8S208RB.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(STM8_SO8_DISCO_STM8S001)
  #include "../../include/STM8S001J3.h"
  #define LED_PORT   sfr_PORTA
  #define LED_PIN    PIN3
#elif defined(STM8_SO8_DISCO_STM8L001)
  #include "../../include/STM8L001J3.h"
  #define LED_PORT   sfr_PORTB
  #define LED_PIN    PIN3
#elif defined(STM8_SO8_DISCO_STM8L050)
  #include "../../include/STM8L050J3.h"
  #define LED_PORT   sfr_PORTB
  #define LED_PIN    PIN3
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(MUBOARD)
  #include "../../include/STM8S207MB.h"
  #define LED_PORT   sfr_PORTH
  #define LED_PIN    PIN2
#else
  #error undefined board
#endif

// required for global variables
#define _MAIN_
  // no modules with globals required
#undef _MAIN_


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void TIM4_UPD_ISR(void)
   
  \brief ISR for timer 4 (1ms master clock)
   
  interrupt service routine for timer TIM4.
  Used for 1ms master clock and optional user function

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
#if defined(_TIM4_OVR_UIF_VECTOR_)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_)
#elif defined(_TIM4_UIF_VECTOR_)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_UIF_VECTOR_)
#else
  #error TIM4 vector undefined
#endif
{

  static uint16_t count = 0;
  
  // blink LED
  count++;
  if (count == 500) {
    count = 0;

    // toggle LED
    LED_PORT.ODR.byte ^= LED_PIN;    

  } // 500ms
  
  // clear timer 4 interrupt flag
  #if defined(FAMILY_STM8S)
    sfr_TIM4.SR.UIF = 0;
  #else
    sfr_TIM4.SR1.UIF = 0;
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

  // for low-power device activate TIM4 clock
  #if defined(FAMILY_STM8L)
    sfr_CLK.PCKENR1.PCKEN12 = 1;
  #endif
   
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
  
  // configure LED pin as output
  LED_PORT.DDR.byte = LED_PIN;    // input(=0) or output(=1)
  LED_PORT.CR1.byte = LED_PIN;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  LED_PORT.CR2.byte = LED_PIN;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // init 1ms interrupt
  TIM4_init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  
    
  // dummy main loop. Action happens in ISR TIM4 TIM4_UPD 
  while(1) {
    WAIT_FOR_INTERRUPT();
  }
  
} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
